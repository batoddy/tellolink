"""tellolink testleri: kayıtlı iki uçuş TelloPy ile karşılaştırılır, giden baytlar ve video çözme sınanır.

Çalıştırma (proje kökünde): python -m unittest discover -s tellolink/tests -t .
Gerçek drone gerekmez; soketler sahte.
"""
import datetime
import json
import math
import threading
import time
import types
import unittest
from pathlib import Path
from unittest import mock

from tellolink import protocol as p
from tellolink.tello import Tello

ROOT = Path(__file__).resolve().parents[2]
FLIGHTS = [ROOT / 'kayitlar' / 'paket_dinleme_20261111_105319' / 'paketler.jsonl',
           ROOT / 'kayitlar' / 'paket_dinleme_20261111_143727' / 'paketler.jsonl']
FIXED_NOW = datetime.datetime(2026, 11, 11, 14, 37, 27, 678901)


class FakeSock:
    """Gönderilenleri biriktirir; hiçbir şey almaz."""

    def __init__(self, *args, **kwargs):
        self.sent = []

    def sendto(self, buf, addr):
        self.sent.append(bytes(buf))

    def __getattr__(self, name):        # bind, settimeout, setsockopt...
        return lambda *a, **k: None


fixed_datetime = types.SimpleNamespace(datetime=types.SimpleNamespace(now=lambda: FIXED_NOW))


def make_tellopy():
    """Soketi sahte, thread'leri başlatılmayan bir TelloPy nesnesi."""
    import socket
    import tellopy
    from tellopy._internal import tello as tp_tello
    fake_socket = types.SimpleNamespace(socket=FakeSock, AF_INET=0, SOCK_DGRAM=0, SOL_SOCKET=0, SO_RCVBUF=0,
                                        timeout=socket.timeout, error=OSError)
    fake_threading = types.SimpleNamespace(Thread=lambda **k: types.SimpleNamespace(start=lambda: None),
                                           Lock=threading.Lock, Event=threading.Event)
    with mock.patch.object(tp_tello, 'socket', fake_socket), mock.patch.object(tp_tello, 'threading', fake_threading):
        drone = tellopy.Tello()
    drone.set_loglevel(drone.LOG_WARN)
    return drone


def make_tellolink():
    drone = Tello()
    drone._sock = FakeSock()
    return drone


def fixed_time():
    from tellopy._internal import protocol as tp_protocol
    return (mock.patch.object(tp_protocol, 'datetime', fixed_datetime),
            mock.patch.object(p, 'datetime', fixed_datetime))


def same(a, b):
    if isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b):
        return True
    if isinstance(a, (tuple, list)):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    return a == b


# yeni alan -> (TelloPy alanı, ölçek): yeni = eski / ölçek
STATUS_FIELDS = {'height': ('height', 10), 'vel_x': ('north_speed', 10), 'vel_y': ('east_speed', 10),
                 'vel_z': ('ground_speed', 10), 'fly_time': ('fly_time', 10),
                 'battery_percent': ('battery_percentage', 1), 'battery_voltage': ('drone_fly_time_left', 1000),
                 'battery_left': ('drone_battery_left', 1)}
RENAMED = {('imu', 'vel_x'): 'vg_x', ('imu', 'vel_y'): 'vg_y', ('imu', 'vel_z'): 'vg_z'}
SCALED = {('tof', 'distance'): 1000}
KIND_CLASS = {'imu': 'LogImuAtti', 'gyro': 'LogGyro', 'mvo': 'LogNewMvoFeedback', 'tof': 'LogTof',
              'nav': 'LogImuEx', 'osd': 'LogOsdGeneral', 'sticks': 'LogController',
              'attitude_target': 'LogAttitudeControl', 'motors': 'LogMotorCtrl', 'battery': 'LogBattery'}


class RecordedFlights(unittest.TestCase):
    """İki gerçek uçuşun her paketi iki kütüphaneden geçirilir; her alan ve her giden paket karşılaştırılır."""

    def run_flight(self, path):
        import tellopy
        packets = []
        with open(path, encoding='utf-8') as f:
            for line in f:
                r = json.loads(line)
                if r['tur'] == 'paket':
                    packets.append((r['recv_time'], bytes.fromhex(r['hex'])))

        old = make_tellopy()
        old.state = old.STATE_CONNECTING
        old.video_enabled = True
        got = {'status': [], 'wifi': [], 'records': []}

        def on_flight(event, sender, data, **kw): got['status'].append(data)
        def on_wifi(event, sender, data, recv_time, **kw): got['wifi'].append((recv_time, bytes(data)))
        def on_record(event, sender, data, **kw): got['records'].append(data)
        handlers = [(old.EVENT_FLIGHT_DATA, on_flight), (old.EVENT_WIFI, on_wifi)]
        handlers += [(e, on_record) for e in (old.EVENT_SAMPLE_IMU, old.EVENT_SAMPLE_GYRO, old.EVENT_SAMPLE_TOF,
                                              old.EVENT_SAMPLE_MVO, old.EVENT_SAMPLE_CONTROL, old.EVENT_SAMPLE_RAW)]
        for event, handler in handlers:
            old.subscribe(event, handler)

        new = make_tellolink()
        new._state = 'connecting'
        # Her olayı sırasıyla yakala; olay geldiğinde aynı adlı alana o kaydın yazılmış olması gerekir.
        mine = {kind: [] for kind in ['status', 'wifi', 'unsolved'] + list(KIND_CLASS)}
        def on_event(drone, data, kind):
            if kind == 'unsolved':
                self.assertIs(drone.unsolved[data.record_id], data)
            else:
                self.assertIs(getattr(drone, kind), data)
            mine[kind].append(data)
        for kind in mine:
            new.subscribe(kind, lambda drone, data, kind=kind: on_event(drone, data, kind))
        p1, p2 = fixed_time()
        try:
            with p1, p2:
                for recv_time, data in packets:
                    old._Tello__process_packet(data, recv_time)
                    new._process_packet(data, recv_time)
        finally:
            for event, handler in handlers:
                old.unsubscribe(event, handler)
            old.quit()

        # Bütünlük: hiçbir paket ve kayıt CRC'den düşmedi
        self.assertEqual(sum(1 for _, d in packets if d[0] == 0xcc and not p.check_packet(d)), 0)
        self.assertEqual(new._log_parser.crc_errors, 0)
        self.assertEqual(new._log_parser.skipped_bytes, 0)
        self.assertEqual(old.log_data.crc_errors, 0)

        # Uçuş durumu
        self.assertEqual(len(mine['status']), len(got['status']))
        for a, b in zip(mine['status'], got['status']):
            self.assertEqual(a.recv_time, b.recv_time)
            self.assertIsNone(a.tick)
            for field in p.Status.__dataclass_fields__:
                if field in ('recv_time', 'tick'):
                    continue
                old_name, scale = STATUS_FIELDS.get(field, (field, 1))
                expected = getattr(b, old_name) / scale if scale != 1 else getattr(b, old_name)
                self.assertEqual(getattr(a, field), expected, field)

        # Wi-Fi
        self.assertEqual([(w.recv_time, w.strength, w.disturb) for w in mine['wifi']],
                         [(t, d[0], d[1]) for t, d in got['wifi']])

        # Günlük kayıtları: çözülmeyenler (bilinmeyen id ya da kısa gelen) EVENT_UNSOLVED ile, ham hâliyle
        old_records = got['records']
        kind_id = {kind: record_id for record_id, (kind, _) in p.RECORDS.items()}
        solved = {(r.recv_time, r.tick, kind_id[kind]) for kind in KIND_CLASS for r in mine[kind]}
        old_unsolved = sorted(((r.recv_time, r.tick, r.record_id, r.payload) for r in old_records
                               if (r.recv_time, r.tick, r.record_id) not in solved))
        mine_unsolved = sorted((r.recv_time, r.tick, r.record_id, r.payload) for r in mine['unsolved'])
        self.assertEqual(mine_unsolved, old_unsolved)

        counts = {}
        for kind, cls_name in KIND_CLASS.items():
            cls = getattr(tellopy, cls_name)
            olds = [r for r in old_records if type(r) is cls]
            news = mine[kind]
            counts[kind] = len(news)
            self.assertEqual(len(news), len(olds), kind)
            for a, b in zip(news, olds):
                self.assertEqual((a.recv_time, a.tick), (b.recv_time, b.tick))
                for field in type(a).__dataclass_fields__:
                    if field in ('recv_time', 'tick', 'image', 'data'):
                        continue
                    value = getattr(a, field)
                    if value is None:       # kısa kayıt: TelloPy 0 bırakır
                        continue
                    expected = getattr(b, RENAMED.get((kind, field), field))
                    if (kind, field) in SCALED:
                        expected = expected / SCALED[(kind, field)]
                    self.assertTrue(same(value, expected), '%s.%s: %r != %r' % (kind, field, value, expected))

        # Paketlere tepki olarak gönderilenler (video isteği, saat, günlük onayı) bayt bayt aynı
        self.assertEqual(new._sock.sent, old.sock.sent)
        return len(packets), len(mine['unsolved']), counts, len(new._sock.sent)

    def test_flights(self):
        for path in FLIGHTS:
            if not path.exists():
                self.skipTest('kayit yok: %s' % path)
            with self.subTest(flight=path.parent.name):
                n_packets, n_records, counts, n_sent = self.run_flight(path)
                print('\n%s: %d paket, %d kayit, %d giden paket, %s' % (path.parent.name, n_packets, n_records,
                                                                       n_sent, counts))


class SentPackets(unittest.TestCase):
    """Komutların baytları TelloPy'nin aynı komut için ürettiğiyle aynı mı?"""

    def test_commands(self):
        old = make_tellopy()
        new = make_tellolink()
        p1, p2 = fixed_time()
        try:
            with p1, p2:
                old._Tello__send_conn_req(); new._send(p.CONN_REQ)
                old.takeoff(); new.takeoff()
                old.land(); new.land()
                old.emergency(); new.emergency()
                for name in ('forward', 'back', 'left', 'right', 'forwardleft', 'forwardright', 'backleft', 'backright'):
                    getattr(old, 'flip_' + name)()
                    new.flip(name.replace('dleft', 'd_left').replace('dright', 'd_right')
                             .replace('kleft', 'k_left').replace('kright', 'k_right'))
                old.start_video(); new._request_video()
                for pitch, roll, yaw, throttle in [(0, 0, 0, 0), (0.3, -0.5, 1.0, -0.25), (-1, 1, -1, 1),
                                                   (0.999, -0.001, 0.5, 0.2)]:
                    old.set_pitch(pitch); old.set_roll(roll); old.set_yaw(yaw); old.set_throttle(throttle)
                    old._Tello__send_stick_command()
                    new.set_sticks(pitch=pitch, roll=roll, yaw=yaw, throttle=throttle)
                    new._send_sticks()
                old.set_alt_limit(30); new.set_altitude_limit(30)
                old.set_low_bat_threshold(25); new.set_low_battery_threshold(25)
                old._Tello__send_time_command(); new._send_time()
                self.call_acked(old, lambda: old.set_ssid('TELLO-ABC'), lambda pkt: old._Tello__process_packet(pkt, time.monotonic()))
                self.call_acked(new, lambda: new.set_wifi_name('TELLO-ABC'), lambda pkt: new._process_packet(pkt, time.monotonic()))
                self.call_acked(old, lambda: old.set_password('12345678'), lambda pkt: old._Tello__process_packet(pkt, time.monotonic()))
                self.call_acked(new, lambda: new.set_wifi_password('12345678'), lambda pkt: new._process_packet(pkt, time.monotonic()))
        finally:
            old.quit()
        self.assertEqual(len(new._sock.sent), len(old.sock.sent))
        for i, (a, b) in enumerate(zip(new._sock.sent, old.sock.sent)):
            self.assertEqual(a.hex(), b.hex(), 'paket %d' % i)

    def call_acked(self, drone, call, feed):
        """Komutu bir thread'de çağırır; gönderilen paketi drone'un yanıtıymış gibi geri verir."""
        sock = drone.sock if hasattr(drone, 'sock') else drone._sock
        n = len(sock.sent)
        t = threading.Thread(target=call)
        t.start()
        while len(sock.sent) == n:
            time.sleep(0.01)
        feed(sock.sent[-1])
        t.join(5)
        self.assertFalse(t.is_alive())
        self.assertEqual(len(sock.sent), n + 1)     # tek denemede onaylandı


def synthetic_tello_frames(n=30, gop=10, width=320, height=240):
    """libx264 ile kısa bir akış; Tello gibi her NAL (SPS, PPS, dilim) ayrı kare. (baytlar, renk) listesi."""
    import av
    import numpy as np
    from fractions import Fraction
    enc = av.CodecContext.create('libx264', 'w')
    enc.width, enc.height, enc.pix_fmt = width, height, 'yuv420p'
    enc.time_base = Fraction(1, 30)
    enc.gop_size = gop
    enc.options = {'tune': 'zerolatency', 'threads': '1', 'bf': '0', 'keyint_min': str(gop), 'sc_threshold': '0'}
    rng = np.random.default_rng(0)
    out = []
    for i in range(n + 1):
        color = None
        if i < n:
            color = ((i * 37) % 200 + 20, (i * 71) % 200 + 20, (i * 113) % 200 + 20)   # B, G, R
            img = np.empty((height, width, 3), np.uint8)
            img[:] = color
            img[:40] = rng.integers(0, 255, (40, width, 3), dtype=np.uint8)            # kareyi büyütmek için gürültü
            frame = av.VideoFrame.from_ndarray(img, format='bgr24')
            frame.pts = i
            packets = enc.encode(frame)
        else:
            packets = enc.encode(None)
        for pkt in packets:
            data = bytes(pkt)
            nals = [b'\x00\x00\x00\x01' + x.rstrip(b'\x00') for x in data.split(b'\x00\x00\x01') if x.strip(b'\x00')]
            for nal in nals:
                if nal[4] & 0x1f in (7, 8):
                    out.append((nal, None))
                elif nal[4] & 0x1f in (1, 5):
                    out.append((nal, int(pkt.pts)))
    return out


def to_udp(frames):
    """Kareleri Tello video paketlerine böler: [kare no, paket no | 0x80 son paket] + en çok 1460 byte."""
    packets = []
    for no, (data, _) in enumerate(frames):
        chunks = [data[i:i + 1460] for i in range(0, len(data), 1460)]
        for i, chunk in enumerate(chunks):
            packets.append((no, i, bytes([no & 0xff, i | (0x80 if i == len(chunks) - 1 else 0)]) + chunk))
    return packets


class Video(unittest.TestCase):
    """Sentetik H.264 akışı Tello paketlerine bölünüp çözücüden geçirilir."""

    def decode(self, packets):
        drone = Tello()
        frames = []
        drone.subscribe(drone.EVENT_FRAME, lambda drone, data: frames.append(data))
        t = threading.Thread(target=drone._decode_loop)
        t.start()
        for recv_time, pkt in packets:
            drone._video_packet(pkt, recv_time)
        drone._decode_queue.put(None)
        t.join(10)
        return frames

    def test_decode(self):
        frames = synthetic_tello_frames()
        pictures = [i for i, (_, pts) in enumerate(frames) if pts is not None]
        self.assertEqual(len(pictures), 30)
        udp = to_udp(frames)
        self.assertGreater(len(udp), len(frames))      # çok paketli kareler var
        decoded = self.decode([(1000.0 + no, pkt) for no, _, pkt in udp])
        self.assertEqual(len(decoded), 30)
        for f, idx in zip(decoded, pictures):
            data, pts = frames[idx]
            self.assertEqual(f.recv_time, 1000.0 + idx)     # karenin kendi baytlarının varışı
            self.assertEqual(f.data, data)
            self.assertIsNone(f.tick)
            self.assertEqual(f.image.shape, (240, 320, 3))
            self.assertEqual(f.keyframe, pts % 10 == 0)
            color = ((pts * 37) % 200 + 20, (pts * 71) % 200 + 20, (pts * 113) % 200 + 20)
            mean = f.image[60:].reshape(-1, 3).mean(axis=0)
            self.assertTrue(all(abs(m - c) < 6 for m, c in zip(mean, color)), (mean, color))

    def test_loss_and_late_start(self):
        frames = synthetic_tello_frames()
        udp = to_udp(frames)
        pictures = [i for i, (_, pts) in enumerate(frames) if pts is not None]
        start = pictures[1]                     # ilk anahtar kareden ve SPS/PPS'ten sonra başla
        lost = pictures[15]                     # bu karenin bir paketi kaybolsun
        lost_chunks = [i for no, i, _ in udp if no == lost]
        self.assertGreater(len(lost_chunks), 1)
        packets = [(1000.0 + no, pkt) for no, i, pkt in udp if no >= start and not (no == lost and i == 1)]
        decoded = self.decode(packets)
        times = [f.recv_time for f in decoded]
        self.assertNotIn(1000.0 + lost, times)           # eksik kare verilmez
        self.assertEqual(times[0], 1000.0 + pictures[10])  # ilk anahtar kareye kadar hiçbir şey çıkmaz
        self.assertTrue(decoded[0].keyframe)
        self.assertEqual(len(decoded), 30 - 10 - 1)


if __name__ == '__main__':
    unittest.main()
