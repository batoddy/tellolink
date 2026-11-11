"""Tello paketlerini kurma ve çözme. Soket ve thread yok; yalnızca baytlar.

Byte konumları ve birimler TELLO_VERI_RAPORU.md'den.
Eksenler: hız ve konum alanları (status, imu.vel, lin_acc, mvo, nav, osd) DÜNYA ekseninde: x = yaw 0° yönü,
y = yaw +90° yönü, z aşağı. Gövdeye göre değildir; drone dönünce ileri hızı x'ten y'ye geçer.
Ham acc_* ve gyro_* gövde ekseninde.
"""
import datetime
import struct
from dataclasses import dataclass

# Mesaj kodları
WIFI_MSG = 0x001A
SSID_CMD = 0x0012
SSID_PASSWORD_CMD = 0x0014
VIDEO_ENCODER_RATE_CMD = 0x0020
VIDEO_START_CMD = 0x0025
EXPOSURE_CMD = 0x0034
TIME_CMD = 0x0046
STICK_CMD = 0x0050
TAKEOFF_CMD = 0x0054
LAND_CMD = 0x0055
FLIP_CMD = 0x005c
FLIGHT_MSG = 0x0056
SET_ALT_LIMIT_CMD = 0x0058
LOG_HEADER_MSG = 0x1050
LOG_DATA_MSG = 0x1051
LOW_BAT_THRESHOLD_CMD = 0x1055
ALT_LIMIT_MSG = 0x1056
LOW_BAT_THRESHOLD_MSG = 0x1057

# Bağlantı isteği: "conn_req:" + 9617 (BCD). TelloPy ile aynı.
CONN_REQ = b'conn_req:\x96\x17'
CONN_ACK = b'conn_ack:'
EMERGENCY = b'emergency'

# Takla yönleri (TelloPy ve Go sürücüsüyle aynı numaralar)
FLIP_DIRECTIONS = {'forward': 0, 'left': 1, 'back': 2, 'right': 3,
                   'forward_left': 4, 'back_left': 5, 'back_right': 6, 'forward_right': 7}


# ---------------------------------------------------------------------------
# CRC
# ---------------------------------------------------------------------------
_CRC8_TABLE = bytes([
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
])

_CRC16_TABLE = (
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
)

def crc8(buf):
    crc = 0x77
    for v in buf:
        crc = _CRC8_TABLE[(crc ^ v) & 0xff]
    return crc


def crc16(buf):
    crc = 0x3692
    for v in buf:
        crc = _CRC16_TABLE[(crc ^ v) & 0xff] ^ (crc >> 8)
    return crc


# ---------------------------------------------------------------------------
# Giden paketler
# ---------------------------------------------------------------------------
def build_packet(cmd, pkt_type, payload=b'', seq=0):
    """0xCC | uzunluk<<3 (2 byte) | crc8 | tip | komut (2) | sıra no (2) | içerik | crc16 (2)."""
    buf = bytearray([0xcc, 0, 0, 0, pkt_type, cmd & 0xff, (cmd >> 8) & 0xff, seq & 0xff, (seq >> 8) & 0xff])
    buf += payload
    size = (len(buf) + 2) << 3
    buf[1], buf[2] = size & 0xff, (size >> 8) & 0xff
    buf[3] = crc8(buf[0:3])
    buf += struct.pack('<H', crc16(buf))
    return bytes(buf)


def time_bytes(now=None):
    """Saat, dakika, saniye, ms'nin alt ve üst byte'ı; her biri 2 byte (TelloPy add_time ile aynı)."""
    if now is None:
        now = datetime.datetime.now()
    ms = now.microsecond // 1000
    return struct.pack('<5H', now.hour, now.minute, now.second, ms & 0xff, (ms >> 8) & 0xff)


def stick_bytes(roll, pitch, throttle, yaw):
    """Dört eksen (−1…1), her biri 11 bit, 6 byte'a paketlenir. Hızlı mod biti (44) hep 0."""
    a = [int(1024 + 660.0 * v) & 0x7ff for v in (roll, pitch, throttle, yaw)]
    packed = a[0] | (a[1] << 11) | (a[2] << 22) | (a[3] << 33)
    return packed.to_bytes(8, 'little')[:6]


# ---------------------------------------------------------------------------
# Gelen paketler
# ---------------------------------------------------------------------------
def check_packet(data):
    """Uzunluk alanı, başlık crc8'i ve sondaki crc16 doğru mu?"""
    return (len(data) >= 11 and data[0] == 0xcc
            and (data[1] | (data[2] << 8)) >> 3 == len(data)
            and crc8(data[0:3]) == data[3]
            and crc16(data[:-2]) == (data[-2] | (data[-1] << 8)))


# ---------------------------------------------------------------------------
# Kayıtlar. Ortak alanlar: recv_time (time.monotonic, paketin varışı), tick (drone sayacı).
# ---------------------------------------------------------------------------
@dataclass
class Record:
    recv_time: float
    tick: int | None


@dataclass
class Status(Record):
    """Uçuş durumu (mesaj 0x0056, 10 Hz). tick yok."""
    height: float               # m (telde dm)
    vel_x: float                # m/s, dünya x (yaw 0° yönü) (telde dm/s; TelloPy: north_speed)
    vel_y: float                # m/s, dünya y (yaw +90° yönü) (TelloPy: east_speed)
    vel_z: float                # m/s, aşağı + (TelloPy: ground_speed)
    fly_time: float             # s, yalnızca havada artar (telde 0.1 s)
    battery_percent: int
    battery_voltage: float      # V (telde mV; TelloPy: drone_fly_time_left)
    battery_left: int           # anlamı bilinmiyor, hep 0 (TelloPy: drone_battery_left)
    fly_mode: int               # 6 yerde/havada sabit, 11 kalkış, 12 iniş
    em_sky: bool                # havada
    em_ground: bool             # yalnızca kalkışta motorlar dönmeye başlarken ~0.5 s; "yerde" DEĞİL
    em_open: bool               # motorlar dönüyor
    drone_hover: bool
    outage_recording: bool
    battery_low: bool
    battery_lower: bool
    factory_mode: bool
    imu_state: bool
    pressure_state: bool
    down_visual_state: bool
    power_state: bool
    battery_state: bool
    gravity_state: bool
    wind_state: bool
    imu_calibration_state: int
    throw_fly_timer: int
    camera_state: int
    electrical_machinery_state: int
    front_in: bool
    front_out: bool
    front_lsc: bool
    temperature_height: bool


@dataclass
class Wifi(Record):
    """Wi-Fi mesajı (0x001A, 1 Hz). tick yok."""
    strength: int               # sinyal gücü
    disturb: int | None         # parazit (~); mesaj kısaysa None


@dataclass
class Imu(Record):
    """LogImuAtti, kayıt 2048 (10 Hz)."""
    acc_x: float
    acc_y: float
    acc_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    q0: float                   # kuaterniyon, q0 = w
    q1: float
    q2: float
    q3: float
    lin_acc_x: float            # m/s², dünya ekseninde, yerçekimi çıkarılmış
    lin_acc_y: float
    lin_acc_z: float
    vel_x: float                # m/s (TelloPy: vg_x)
    vel_y: float
    vel_z: float
    baro_raw: float             # m, ham barometre irtifası (yalnız farkı anlamlı)
    baro: float                 # m, yumuşatılmış
    gyro_bias_x: float | None
    gyro_bias_y: float | None
    gyro_bias_z: float | None
    temperature: float | None   # °C


@dataclass
class Gyro(Record):
    """LogGyro, kayıt 1305 (20 Hz, yalnızca havada). Üç işlem aşaması, her biri (x, y, z)."""
    stages: tuple


@dataclass
class Mvo(Record):
    """LogNewMvoFeedback, kayıt 29 (5 Hz). Optik akış."""
    vel_x: float                # m/s (telde mm/s)
    vel_y: float
    vel_z: float
    pos_x: float                # m; kalkışta sıçrar
    pos_y: float
    pos_z: float
    height: float | None        # m, yerden
    flags: int | None           # geçerlilik bitleri


@dataclass
class Tof(Record):
    """LogTof, kayıt 16 (5 Hz)."""
    distance: float             # m (telde mm); yerde 0.1
    flag: int
    counter: int                # her kayıtta 4 artar


@dataclass
class Nav(Record):
    """LogImuEx, kayıt 2064 (10 Hz). Kalkışta sıçramayan konum."""
    vel_x: float                # m/s
    vel_y: float
    vel_z: float
    pos_x: float                # m
    pos_y: float
    pos_z: float                # m, −baro
    tof_vel: float              # m/s, yükselirken −
    height: float               # m, yerden
    flags: int
    counter: int


@dataclass
class Osd(Record):
    """LogOsdGeneral, kayıt 12 (1.7 Hz)."""
    height: float               # m
    vel_x: float                # m/s, 0.1 adımlı
    vel_y: float
    vel_z: float
    pitch: float                # derece
    roll: float
    yaw: float
    fly_mode: int
    tof_height: float           # m
    fly_time: float             # s
    yaw_rate: float             # derece/s


@dataclass
class Sticks(Record):
    """LogController, kayıt 1000 (5 Hz). Drone'un gördüğü çubuklar."""
    pitch: float
    roll: float
    yaw: float
    throttle: float
    fly_mode: int
    motors_on: int


@dataclass
class AttitudeTarget(Record):
    """LogAttitudeControl, kayıt 1304 (2.5 Hz). Radyan."""
    target_yaw: float
    target_roll: float
    target_pitch: float
    yaw: float
    roll: float
    pitch: float


@dataclass
class Motors(Record):
    """LogMotorCtrl, kayıt 1307 (10 Hz). PWM %, sıra: ön sağ, ön sol, arka sol, arka sağ."""
    pwm: tuple


@dataclass
class Battery(Record):
    """LogBattery, kayıt 1710 (5 Hz)."""
    voltage: float              # V


@dataclass
class UnsolvedRecord(Record):
    """Anlamı çözülmemiş (ya da çözülemeyecek kadar kısa gelmiş) bir günlük kaydı: id ve ham payload (XOR'u açılmış)."""
    record_id: int
    payload: bytes


@dataclass
class Frame(Record):
    """Çözülmüş video karesi. recv_time: karenin son UDP paketinin varışı. tick yok."""
    image: object               # numpy dizisi, yükseklik × genişlik × 3, BGR
    data: bytes                 # karenin ham H.264 baytları
    keyframe: bool


def parse_status(payload, recv_time):
    """Uçuş durumu içeriği (CRC hariç); 24 byte'tan kısaysa None."""
    if len(payload) < 24:
        return None
    height, vx, vy, vz, fly_time = struct.unpack_from('<5h', payload, 0)
    b10, calib, percent = payload[10], payload[11], payload[12]
    battery_left, millivolt = struct.unpack_from('<2h', payload, 13)
    b17, b22, b23 = payload[17], payload[22], payload[23]

    def bit(b, n):
        return bool((b >> n) & 1)

    return Status(
        recv_time, None,
        height=height / 10.0, vel_x=vx / 10.0, vel_y=vy / 10.0, vel_z=vz / 10.0, fly_time=fly_time / 10.0,
        battery_percent=percent, battery_voltage=millivolt / 1000.0, battery_left=battery_left,
        fly_mode=payload[18],
        em_sky=bit(b17, 0), em_ground=bit(b17, 1), em_open=bit(b17, 2), drone_hover=bit(b17, 3),
        outage_recording=bit(b17, 4), battery_low=bit(b17, 5), battery_lower=bit(b17, 6),
        factory_mode=bit(b17, 7),
        imu_state=bit(b10, 0), pressure_state=bit(b10, 1), down_visual_state=bit(b10, 2),
        power_state=bit(b10, 3), battery_state=bit(b10, 4), gravity_state=bit(b10, 5), wind_state=bit(b10, 7),
        imu_calibration_state=calib, throw_fly_timer=payload[19], camera_state=payload[20],
        electrical_machinery_state=payload[21],
        front_in=bit(b22, 0), front_out=bit(b22, 1), front_lsc=bit(b22, 2),
        temperature_height=bit(b23, 0))


def parse_wifi(payload, recv_time):
    return Wifi(recv_time, None, strength=payload[0], disturb=payload[1] if len(payload) > 1 else None)


# Kayıt çözücüleri: (recv_time, tick, payload) -> kayıt
def _imu(t, tick, d):
    extra = struct.unpack_from('<3f', d, 88) + (struct.unpack_from('<h', d, 106)[0] / 100.0,) \
        if len(d) >= 108 else (None,) * 4
    return Imu(t, tick, *struct.unpack_from('<3f', d, 20), *struct.unpack_from('<3f', d, 32),
               *struct.unpack_from('<4f', d, 48), *struct.unpack_from('<3f', d, 64),
               *struct.unpack_from('<3f', d, 76), struct.unpack_from('<f', d, 16)[0],
               struct.unpack_from('<f', d, 44)[0], *extra)


def _mvo(t, tick, d):
    vx, vy, vz = struct.unpack_from('<3h', d, 2)
    pos = struct.unpack_from('<3f', d, 8)
    height = flags = None
    if len(d) >= 77:
        height = -struct.unpack_from('<f', d, 68)[0]
        flags = d[76]
    return Mvo(t, tick, vx / 1000.0, vy / 1000.0, vz / 1000.0, *pos, height, flags)


def _gyro(t, tick, d):
    return Gyro(t, tick, tuple(struct.unpack_from('<3f', d, off) for off in (1, 13, 25)))


def _tof(t, tick, d):
    distance, flag, counter = struct.unpack_from('<hBB', d, 0)
    return Tof(t, tick, distance / 1000.0, flag, counter)


def _nav(t, tick, d):
    return Nav(t, tick, *struct.unpack_from('<8f', d, 0), *struct.unpack_from('<HH', d, 56))


def _osd(t, tick, d):
    height, vx, vy, vz, pitch, roll, yaw, fly_mode = struct.unpack_from('<7hB', d, 16)
    tof_height, fly_time = struct.unpack_from('<BH', d, 41)
    yaw_rate = struct.unpack_from('<h', d, 53)[0]
    return Osd(t, tick, height / 10.0, vx / 10.0, vy / 10.0, vz / 10.0, pitch / 10.0, roll / 10.0,
               yaw / 10.0, fly_mode, tof_height / 10.0, fly_time / 10.0, yaw_rate / 10.0)


def _sticks(t, tick, d):
    return Sticks(t, tick, *struct.unpack_from('<4fB', d, 4), d[22])


def _attitude_target(t, tick, d):
    return AttitudeTarget(t, tick, *struct.unpack_from('<3f', d, 2), *struct.unpack_from('<3f', d, 22))


def _motors(t, tick, d):
    return Motors(t, tick, tuple(x / 100.0 for x in struct.unpack_from('<4H', d, 0)))


def _battery(t, tick, d):
    return Battery(t, tick, struct.unpack_from('<f', d, 0)[0] / 1000.0)


# kayıt id -> (okuma fonksiyonundaki ad, çözücü)
RECORDS = {
    2048: ('imu', _imu),
    1305: ('gyro', _gyro),
    29: ('mvo', _mvo),
    16: ('tof', _tof),
    2064: ('nav', _nav),
    12: ('osd', _osd),
    1000: ('sticks', _sticks),
    1304: ('attitude_target', _attitude_target),
    1307: ('motors', _motors),
    1710: ('battery', _battery),
}


def decode_record(record_id, tick, payload, recv_time):
    """Bilinen bir kaydı çöz; bilinmiyorsa ya da kısa geldiyse None."""
    entry = RECORDS.get(record_id)
    if entry is None:
        return None
    try:
        return entry[1](recv_time, tick, payload)
    except (struct.error, IndexError):
        return None


class LogParser:
    """Günlük mesajlarını (0x1051) kayıtlara böler. TelloPy LogData.update ile aynı mantık.

    Kayıt: 0x55 | uzunluk (2) | crc8 | id (2) | tick (4) | payload (tick'in ilk byte'ıyla XOR) | crc16 (2).
    Drone kayıtları mesaj sınırına bakmadan böler; yarım kalan kayıt bir sonraki mesajla tamamlanır.
    """
    MIN_RECORD = 12
    MAX_RECORD = 4096

    def __init__(self):
        self.carry = b''
        self.crc_errors = 0         # crc16'sı yanlış olduğu için atılan kayıtlar
        self.skipped_bytes = 0      # sonraki kayıt başını bulmak için atlanan byte'lar

    def _header_at(self, body, pos):
        if body[pos] != 0x55 or crc8(body[pos:pos + 3]) != body[pos + 3]:
            return None
        length = struct.unpack_from('<H', body, pos + 1)[0]
        return length if self.MIN_RECORD <= length <= self.MAX_RECORD else None

    def feed(self, data):
        """Bir mesajın içeriği (paketin 10. byte'ından CRC'ye kadar). (id, tick, payload) listesi döner."""
        body = self.carry + bytes(data)
        self.carry = b''
        out = []
        pos = 0
        while pos < len(body):
            if len(body) - pos < 4:
                self.carry = body[pos:]
                break
            length = self._header_at(body, pos)
            if length is None:
                start = pos + 1
                while start < len(body) - 3 and self._header_at(body, start) is None:
                    start += 1
                if start >= len(body) - 3:
                    start = len(body)
                self.skipped_bytes += start - pos
                pos = start
                continue
            if len(body) < pos + length:
                self.carry = body[pos:]
                break
            rec = body[pos:pos + length]
            pos += length
            if crc16(rec[:-2]) != struct.unpack_from('<H', rec, length - 2)[0]:
                self.crc_errors += 1
                continue
            record_id, tick = struct.unpack_from('<HI', rec, 4)
            xor = rec[6]
            out.append((record_id, tick, bytes(x ^ xor for x in rec[10:-2])))
        return out


class FrameAssembler:
    """Video UDP paketlerinden kare toplar.

    Paket başlığı: [0] kare sıra no, [1] & 0x7F karedeki paket no, [1] & 0x80 karenin son paketi.
    Eksik paketli kare atılır; bir sonraki karenin ilk paketinden devam edilir.
    """

    def __init__(self):
        self.parts = None
        self.frame_no = 0
        self.next_index = 0

    def add(self, packet, recv_time):
        """Kare tamamlandıysa (baytlar, recv_time), yoksa None."""
        if len(packet) < 2:
            return None
        frame_no, index, last = packet[0], packet[1] & 0x7f, packet[1] & 0x80
        if index == 0:
            self.parts = [packet[2:]]
            self.frame_no = frame_no
        elif self.parts is not None and frame_no == self.frame_no and index == self.next_index:
            self.parts.append(packet[2:])
        else:
            self.parts = None       # kayıp: sonraki karenin başını bekle
            return None
        self.next_index = index + 1
        if last:
            data = b''.join(self.parts)
            self.parts = None
            return data, recv_time
        return None
