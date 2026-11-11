"""Tello bağlantısı: soketler, alım thread'leri, komutlar, olaylar ve drone'dan gelen son veriler.

Tel davranışı TelloPy'deki gibidir (bağlantı, çubuk gönderimi, video isteği, günlük onayı, saat yanıtı).
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
CMD_PORT = 9000             # komut ve telemetri (yerel)
VIDEO_PORT = 6038           # video (yerel)
RECV_TIMEOUT = 2.0          # bu kadar paket gelmezse bağlantı kopmuş sayılır
VIDEO_TIMEOUT = 1.0         # bu kadar video gelmezse video yeniden istenir
VIDEO_REQUEST_EVERY = 2.0   # video akarken başlatma (SPS/PPS) isteği bu aralıkla tekrarlanır
ACK_TIMEOUT = 1.5           # Wi-Fi ayarlarında yanıt bekleme süresi (3 deneme)
EXPOSURE = 0                # TelloPy'nin varsayılanları; video isteğiyle birlikte gönderilir
VIDEO_ENCODER_RATE = 4


class Tello:
    """Tek bir Tello. Birden çok drone için her birine kendi ağ adaptörünün adresini verin."""

    # subscribe() ile dinlenen olaylar. Veri olaylarında data, aynı adlı alana az önce yazılan kayıttır.
    EVENT_CONNECTED = 'connected'               # data None
    EVENT_DISCONNECTED = 'disconnected'         # data None; 2 s paket gelmedi
    EVENT_STATUS = 'status'                     # Status          (uçuş durumu paketi)
    EVENT_WIFI = 'wifi'                         # Wifi            (Wi-Fi paketi)
    EVENT_IMU = 'imu'                           # Imu             (bundan sonrakiler günlük paketinden)
    EVENT_GYRO = 'gyro'                         # Gyro
    EVENT_MVO = 'mvo'                           # Mvo
    EVENT_TOF = 'tof'                           # Tof
    EVENT_NAV = 'nav'                           # Nav
    EVENT_OSD = 'osd'                           # Osd
    EVENT_STICKS = 'sticks'                     # Sticks
    EVENT_ATTITUDE_TARGET = 'attitude_target'   # AttitudeTarget
    EVENT_MOTORS = 'motors'                     # Motors
    EVENT_BATTERY = 'battery'                   # Battery
    EVENT_UNSOLVED = 'unsolved'                 # UnsolvedRecord  (çözülmemiş kayıtlar)
    EVENT_FRAME = 'frame'                       # Frame           (video thread'inden)

    def __init__(self, local_ip=None):
        self.local_ip = local_ip

        # Drone'dan gelen son veriler. Paket geldikçe kütüphane bunların üzerine yazar;
        # hiç gelmemiş olan None'dır. Her biri protocol.py'deki kayıt sınıfıdır.
        self.status = None          # Status          10 Hz
        self.imu = None             # Imu             10 Hz
        self.gyro = None            # Gyro            20 Hz, sadece havada
        self.mvo = None             # Mvo              5 Hz
        self.tof = None             # Tof              5 Hz
        self.nav = None             # Nav             10 Hz
        self.osd = None             # Osd            ~1.7 Hz
        self.sticks = None          # Sticks           5 Hz
        self.attitude_target = None # AttitudeTarget 2.5 Hz, motorlar çalışırken
        self.motors = None          # Motors          10 Hz, motorlar çalışırken
        self.battery = None         # Battery          5 Hz
        self.wifi = None            # Wifi             1 Hz
        self.unsolved = {}          # kayıt id -> UnsolvedRecord; çözülmemiş kayıtların sonuncusu
        self.frame = None           # Frame          ~30 Hz, video thread'inde yazılır

        self._handlers = {}                 # olay -> [handler, ...]
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
        self._acks = {}                     # (komut, sıra no) -> Event; yalnız Wi-Fi ayarları için
        self._log_parser = p.LogParser()
        self._frames = p.FrameAssembler()
        self._decode_queue = queue.Queue()

    # ------------------------------------------------------------------
    # Bağlantı
    # ------------------------------------------------------------------
    def connect(self, timeout=10):
        """Bağlanır ve videoyu başlatır. Süre dolarsa TimeoutError; arka planda denemeye devam eder."""
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
            raise TimeoutError('Tello yanit vermedi')

    def close(self):
        """Thread'leri durdurur, soketleri kapatır. Drone'u indirmez."""
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
    # Komutlar
    # ------------------------------------------------------------------
    def takeoff(self):
        self._send_cmd(p.TAKEOFF_CMD)

    def land(self):
        self._send_cmd(p.LAND_CMD, b'\x00')

    def flip(self, direction='forward'):
        """Takla: 'forward', 'back', 'left', 'right', 'forward_left', 'forward_right', 'back_left', 'back_right'.
        Drone havada olmalı ve pil yeterli olmalı (düşükse drone yapmaz)."""
        self._send_cmd(p.FLIP_CMD, bytes([p.FLIP_DIRECTIONS[direction]]), 0x70)

    def emergency(self):
        """Motorları hemen durdurur; havadaysa drone düşer."""
        self._send(p.EMERGENCY)

    def set_sticks(self, pitch, roll, yaw, throttle):
        """−1…1. Değerler yenisi verilene kadar her pakette gönderilir."""
        def clip(v):
            return max(-1.0, min(1.0, float(v)))
        self._sticks = (clip(roll), clip(pitch), clip(throttle), clip(yaw))

    def set_altitude_limit(self, m):
        """Yükseklik sınırı, metre (tam sayı)."""
        self._send_cmd(p.SET_ALT_LIMIT_CMD, bytes([int(m), 0]))
        self._send_cmd(p.ALT_LIMIT_MSG)         # TelloPy gibi ardından sorgu; yanıtı kullanılmaz

    def set_low_battery_threshold(self, percent):
        self._send_cmd(p.LOW_BAT_THRESHOLD_CMD, bytes([int(percent)]))
        self._send_cmd(p.LOW_BAT_THRESHOLD_MSG)

    def set_wifi_name(self, name):
        """Wi-Fi adı (ASCII). Drone yeniden açılınca geçerli olur. Yanıt gelmezse TimeoutError."""
        self._send_and_wait(p.SSID_CMD, name.encode('ascii'))

    def set_wifi_password(self, password):
        """Wi-Fi şifresi (ASCII, en çok 19 karakter). Drone yeniden açılınca geçerli olur."""
        payload = password.encode('ascii')
        if len(payload) > 19:
            raise ValueError('sifre en cok 19 karakter olabilir')
        self._send_and_wait(p.SSID_PASSWORD_CMD, payload + bytes(19 - len(payload)))

    # ------------------------------------------------------------------
    # Olaylar
    # ------------------------------------------------------------------
    def subscribe(self, event, handler):
        """handler(drone, data) her olayda çağrılır; drone olayın geldiği Tello nesnesidir.
        Kütüphanenin thread'inde çalışır (EVENT_FRAME video thread'inde): uzun iş yapmayın."""
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
                log.exception('%s handler hatasi', event)

    # ------------------------------------------------------------------
    # Gönderme
    # ------------------------------------------------------------------
    def _send(self, buf):
        sock = self._sock
        if sock is None:
            return False
        try:
            sock.sendto(buf, TELLO_ADDR)
            return True
        except OSError as e:
            log.debug('gonderilemedi: %s', e)
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
        raise TimeoutError('Tello komutu onaylamadi')

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
    # Alım
    # ------------------------------------------------------------------
    def _recv_loop(self):
        while self._running:
            if self._state == 'connected':
                self._send_sticks()         # TelloPy gibi: alınan her paket başına bir çubuk paketi
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
                log.exception('paket islenemedi')

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
            # Bir günlük paketinde farklı hızlardaki birkaç kayıt olur (ör. IMU + ToF + MVO);
            # her kayıt kendi alanına yazılır ve kendi olayını tetikler.
            for record_id, tick, raw in self._log_parser.feed(data[10:-2]):
                record = p.decode_record(record_id, tick, raw, recv_time)
                if record is not None:
                    name = p.RECORDS[record_id][0]
                    setattr(self, name, record)
                    self._emit(name, record)
                else:
                    record = p.UnsolvedRecord(recv_time, tick, record_id, raw)
                    self.unsolved = {**self.unsolved, record_id: record}   # yeni sözlük: okuyanınki değişmesin
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
            self._send_time()               # drone saati soruyor (boş, tip 0x88)

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
        import av   # yalnızca görüntü için

        codec = av.CodecContext.create('h264', 'r')
        waiting = collections.OrderedDict()     # pts -> (baytlar, recv_time)
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
                continue                        # çözülemeyen kare atlanır
            for f in frames:
                data, recv_time = waiting.pop(f.pts, item)
                image = f.to_ndarray(format='bgr24')
                self.frame = p.Frame(recv_time, None, image, data, bool(f.key_frame))
                self._emit(self.EVENT_FRAME, self.frame)
