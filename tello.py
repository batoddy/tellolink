# Copyright 2026 batoddy
# Licensed under the Apache License, Version 2.0 (see LICENSE).
# Protocol logic and CRC tables derived from TelloPy (https://github.com/hanyazou/TelloPy),
# Copyright 2018 Hanyazou, Apache License 2.0; rewritten and modified (see NOTICE).
"""Tello connection: sockets, receive threads, commands, events and the latest data from the drone.

Wire behavior follows TelloPy (connection, stick sending, video request, log ack, time reply).
"""
import collections
import logging
import queue
import socket
import threading
import time

from . import protocol as p

log = logging.getLogger('tellolink')

TELLO_ADDR = ('192.168.10.1', 8889)
CMD_PORT = 9000             # commands and telemetry (local)
VIDEO_PORT = 6038           # video (local)
RECV_TIMEOUT = 2.0          # connection is considered lost if no packet arrives for this long
VIDEO_TIMEOUT = 1.0         # video is requested again if none arrives for this long
VIDEO_REQUEST_EVERY = 2.0   # while video is flowing, the start (SPS/PPS) request is repeated at this interval
ACK_TIMEOUT = 1.5           # how long to wait for a reply to Wi-Fi settings (3 tries)
EXPOSURE = 0                # TelloPy defaults; sent together with the video request
VIDEO_ENCODER_RATE = 4


class Tello:
    """A single Tello. For multiple drones, give each one the address of its own network adapter."""

    # Events listened to with subscribe(). For data events, data is the record just written to the field of the same name.
    EVENT_CONNECTED = 'connected'               # data None
    EVENT_DISCONNECTED = 'disconnected'         # data None; no packet for 2 s
    EVENT_STATUS = 'status'                     # Status          (flight status packet)
    EVENT_WIFI = 'wifi'                         # Wifi            (Wi-Fi packet)
    EVENT_IMU = 'imu'                           # Imu             (the ones below come from the log packet)
    EVENT_GYRO = 'gyro'                         # Gyro
    EVENT_MVO = 'mvo'                           # Mvo
    EVENT_TOF = 'tof'                           # Tof
    EVENT_NAV = 'nav'                           # Nav
    EVENT_OSD = 'osd'                           # Osd
    EVENT_STICKS = 'sticks'                     # Sticks
    EVENT_ATTITUDE_TARGET = 'attitude_target'   # AttitudeTarget
    EVENT_MOTORS = 'motors'                     # Motors
    EVENT_BATTERY = 'battery'                   # Battery
    EVENT_UNSOLVED = 'unsolved'                 # UnsolvedRecord  (undecoded records)
    EVENT_FRAME = 'frame'                       # Frame           (from the video thread)

    def __init__(self, local_ip=None):
        self.local_ip = local_ip

        # Latest data from the drone. The library overwrites these as packets arrive;
        # None if nothing has arrived yet. Each is a record class from protocol.py.
        self.status = None          # Status          10 Hz
        self.imu = None             # Imu             10 Hz
        self.gyro = None            # Gyro            20 Hz, airborne only
        self.mvo = None             # Mvo              5 Hz
        self.tof = None             # Tof              5 Hz
        self.nav = None             # Nav             10 Hz
        self.osd = None             # Osd            ~1.7 Hz
        self.sticks = None          # Sticks           5 Hz
        self.attitude_target = None # AttitudeTarget 2.5 Hz, while motors are running
        self.motors = None          # Motors          10 Hz, while motors are running
        self.battery = None         # Battery          5 Hz
        self.wifi = None            # Wifi             1 Hz
        self.unsolved = {}          # record id -> UnsolvedRecord; the latest of the undecoded records
        self.frame = None           # Frame          ~30 Hz, written in the video thread

        self._handlers = {}                 # event -> [handler, ...]
        self._handlers_lock = threading.Lock()

        self._sock = None
        self._video_sock = None
        self._threads = []
        self._running = False
        self._state = 'disconnected'        # 'connecting', 'connected'
        self._connected = threading.Event()
        self._seq = 0x01e4
        self._seq_lock = threading.Lock()
        self._sticks = (0.0, 0.0, 0.0, 0.0)  # roll, pitch, throttle, yaw
        self._acks = {}                     # (command, sequence no.) -> Event; only for Wi-Fi settings
        self._log_parser = p.LogParser()
        self._frames = p.FrameAssembler()
        self._decode_queue = queue.Queue()

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    def connect(self, timeout=10):
        """Connects and starts the video. Raises TimeoutError if time runs out; keeps trying in the background."""
        if self._sock is None:
            addr = self.local_ip or ''
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind((addr, CMD_PORT))
            self._sock.settimeout(RECV_TIMEOUT)
            self._video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._video_sock.bind((addr, VIDEO_PORT))
            self._video_sock.settimeout(VIDEO_TIMEOUT)
            self._video_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512 * 1024)
            self._running = True
            for target in (self._recv_loop, self._video_loop, self._decode_loop):
                t = threading.Thread(target=target, daemon=True)
                t.start()
                self._threads.append(t)
        if self._state == 'disconnected':
            self._state = 'connecting'
            self._send(p.CONN_REQ)
        if not self._connected.wait(timeout):
            raise TimeoutError('Tello did not respond')

    def close(self):
        """Stops the threads and closes the sockets. Does not land the drone."""
        self._running = False
        self._state = 'disconnected'
        self._connected.clear()
        for s in (self._sock, self._video_sock):
            if s is not None:
                s.close()
        self._decode_queue.put(None)
        for t in self._threads:
            t.join(3.0)
        self._threads = []
        self._sock = self._video_sock = None

    @property
    def connected(self):
        return self._connected.is_set()

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------
    def takeoff(self):
        self._send_cmd(p.TAKEOFF_CMD)

    def land(self):
        self._send_cmd(p.LAND_CMD, b'\x00')

    def flip(self, direction='forward'):
        """Flip: 'forward', 'back', 'left', 'right', 'forward_left', 'forward_right', 'back_left', 'back_right'.
        The drone must be airborne and the battery sufficient (the drone refuses if it is low)."""
        self._send_cmd(p.FLIP_CMD, bytes([p.FLIP_DIRECTIONS[direction]]), 0x70)

    def emergency(self):
        """Stops the motors immediately; if airborne, the drone falls."""
        self._send(p.EMERGENCY)

    def set_sticks(self, pitch, roll, yaw, throttle):
        """−1…1. Values are sent in every packet until new ones are given."""
        def clip(v):
            return max(-1.0, min(1.0, float(v)))
        self._sticks = (clip(roll), clip(pitch), clip(throttle), clip(yaw))

    def set_altitude_limit(self, m):
        """Altitude limit, meters (integer)."""
        self._send_cmd(p.SET_ALT_LIMIT_CMD, bytes([int(m), 0]))
        self._send_cmd(p.ALT_LIMIT_MSG)         # follow-up query like TelloPy; its reply is not used

    def set_low_battery_threshold(self, percent):
        self._send_cmd(p.LOW_BAT_THRESHOLD_CMD, bytes([int(percent)]))
        self._send_cmd(p.LOW_BAT_THRESHOLD_MSG)

    def set_wifi_name(self, name):
        """Wi-Fi name (ASCII). Takes effect when the drone is restarted. Raises TimeoutError if there is no reply."""
        self._send_and_wait(p.SSID_CMD, name.encode('ascii'))

    def set_wifi_password(self, password):
        """Wi-Fi password (ASCII, at most 19 characters). Takes effect when the drone is restarted."""
        payload = password.encode('ascii')
        if len(payload) > 19:
            raise ValueError('password can be at most 19 characters')
        self._send_and_wait(p.SSID_PASSWORD_CMD, payload + bytes(19 - len(payload)))

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------
    def subscribe(self, event, handler):
        """handler(drone, data) is called on every event; drone is the Tello object the event came from.
        Runs in the library's thread (EVENT_FRAME in the video thread): do not do long work."""
        with self._handlers_lock:
            self._handlers.setdefault(event, []).append(handler)

    def unsubscribe(self, event, handler):
        with self._handlers_lock:
            if handler in self._handlers.get(event, []):
                self._handlers[event].remove(handler)

    def _emit(self, event, data):
        with self._handlers_lock:
            handlers = list(self._handlers.get(event, ()))
        for handler in handlers:
            try:
                handler(self, data)
            except Exception:
                log.exception('%s handler error', event)

    # ------------------------------------------------------------------
    # Sending
    # ------------------------------------------------------------------
    def _send(self, buf):
        sock = self._sock
        if sock is None:
            return False
        try:
            sock.sendto(buf, TELLO_ADDR)
            return True
        except OSError as e:
            log.debug('send failed: %s', e)
            return False

    def _next_seq(self):
        with self._seq_lock:
            self._seq = (self._seq + 1) & 0xffff or 1
            return self._seq

    def _send_cmd(self, cmd, payload=b'', pkt_type=0x68):
        self._send(p.build_packet(cmd, pkt_type, payload, self._next_seq()))

    def _send_and_wait(self, cmd, payload, tries=3):
        for _ in range(tries):
            seq = self._next_seq()
            done = self._acks[(cmd, seq)] = threading.Event()
            self._send(p.build_packet(cmd, 0x68, payload, seq))
            ok = done.wait(ACK_TIMEOUT)
            self._acks.pop((cmd, seq), None)
            if ok:
                return
        raise TimeoutError('Tello did not acknowledge the command')

    def _send_sticks(self):
        roll, pitch, throttle, yaw = self._sticks
        self._send_cmd(p.STICK_CMD, p.stick_bytes(roll, pitch, throttle, yaw) + p.time_bytes(), 0x60)

    def _send_time(self):
        self._send_cmd(p.TIME_CMD, b'\x00' + p.time_bytes(), 0x50)

    def _request_video(self):
        self._send_cmd(p.EXPOSURE_CMD, bytes([EXPOSURE]), 0x48)
        self._send_cmd(p.VIDEO_ENCODER_RATE_CMD, bytes([VIDEO_ENCODER_RATE]))
        self._send_cmd(p.VIDEO_START_CMD, b'', 0x60)

    # ------------------------------------------------------------------
    # Receiving
    # ------------------------------------------------------------------
    def _recv_loop(self):
        while self._running:
            if self._state == 'connected':
                self._send_sticks()         # like TelloPy: one stick packet per received packet
            try:
                data, _ = self._sock.recvfrom(2000)
            except socket.timeout:
                if self._state != 'disconnected':
                    was_connected = self._state == 'connected'
                    self._state = 'connecting'
                    self._connected.clear()
                    self._send(p.CONN_REQ)
                    if was_connected:
                        self._emit(self.EVENT_DISCONNECTED, None)
                continue
            except (OSError, AttributeError):
                if not self._running:
                    break
                continue
            try:
                self._process_packet(data, time.monotonic())
            except Exception:
                log.exception('could not process packet')

    def _process_packet(self, data, recv_time):
        if data[:9] == p.CONN_ACK:
            self._request_video()
            if self._state == 'connecting':
                self._state = 'connected'
                self._connected.set()
                self._send_time()
                self._emit(self.EVENT_CONNECTED, None)
            return
        if not p.check_packet(data):
            return
        cmd = data[5] | (data[6] << 8)
        done = self._acks.get((cmd, data[7] | (data[8] << 8)))
        if done is not None:
            done.set()
        payload = data[9:-2]
        if cmd == p.LOG_DATA_MSG:
            # A log packet holds several records at different rates (e.g. IMU + ToF + MVO);
            # each record is written to its own field and fires its own event.
            for record_id, tick, raw in self._log_parser.feed(data[10:-2]):
                record = p.decode_record(record_id, tick, raw, recv_time)
                if record is not None:
                    name = p.RECORDS[record_id][0]
                    setattr(self, name, record)
                    self._emit(name, record)
                else:
                    record = p.UnsolvedRecord(recv_time, tick, record_id, raw)
                    self.unsolved = {**self.unsolved, record_id: record}   # new dict: so a reader's copy does not change
                    self._emit(self.EVENT_UNSOLVED, record)
        elif cmd == p.FLIGHT_MSG:
            status = p.parse_status(payload, recv_time)
            if status is not None:
                self.status = status
                self._emit(self.EVENT_STATUS, status)
        elif cmd == p.WIFI_MSG and payload:
            self.wifi = p.parse_wifi(payload, recv_time)
            self._emit(self.EVENT_WIFI, self.wifi)
        elif cmd == p.LOG_HEADER_MSG:
            self._send_cmd(p.LOG_HEADER_MSG, bytes([0, data[9], data[10]]), 0x50)
        elif cmd == p.TIME_CMD and data[4] == 0x88 and len(data) == 11:
            self._send_time()               # the drone is asking for the time (empty, type 0x88)

    def _video_loop(self):
        last_request = None
        while self._running:
            try:
                data, _ = self._video_sock.recvfrom(2048)
            except socket.timeout:
                if self._state == 'connected':
                    self._request_video()
                continue
            except (OSError, AttributeError):
                if not self._running:
                    break
                continue
            self._video_packet(data, time.monotonic())
            now = time.monotonic()
            if last_request is None:
                last_request = now
            elif now - last_request > VIDEO_REQUEST_EVERY:
                self._send_cmd(p.VIDEO_START_CMD, b'', 0x60)
                last_request = now

    def _video_packet(self, data, recv_time):
        frame = self._frames.add(data, recv_time)
        if frame is not None:
            self._decode_queue.put(frame)

    def _decode_loop(self):
        import av   # only for video

        codec = av.CodecContext.create('h264', 'r')
        waiting = collections.OrderedDict()     # pts -> (bytes, recv_time)
        pts = 0
        while True:
            item = self._decode_queue.get()
            if item is None:
                break
            data, recv_time = item
            pts += 1
            waiting[pts] = item
            while len(waiting) > 64:
                waiting.popitem(last=False)
            packet = av.Packet(data)
            packet.pts = pts
            try:
                frames = codec.decode(packet)
            except av.error.FFmpegError:
                continue                        # a frame that cannot be decoded is skipped
            for f in frames:
                data, recv_time = waiting.pop(f.pts, item)
                image = f.to_ndarray(format='bgr24')
                self.frame = p.Frame(recv_time, None, image, data, bool(f.key_frame))
                self._emit(self.EVENT_FRAME, self.frame)
