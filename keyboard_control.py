"""tellolink ile klavyeden uçurma: canlı görüntü, ayarlanabilir hızlar, takla ve canlı veri paneli.

Çalıştırma (bilgisayar drone'un Wi-Fi ağına bağlıyken):
    python tellolink/keyboard_control.py
    python tellolink/keyboard_control.py --speed 0.4 --vertical 0.3 --turn 0.5   # başlangıç hızları

Tuşlar:
    T          kalkış                    L        iniş
    W / S      ileri / geri              A / D    sola / sağa kay
    ↑ / ↓      yüksel / alçal            ← / →    sola / sağa dön
    Boşluk     basılıyken dur (havada asılı kal)
    F          takla (W/A/S/D basılıysa o yöne, değilse öne)
    1 / 2      hareket hızı − / +        3 / 4    yükselme hızı − / +
    5 / 6      dönüş hızı − / +
    I          veri panelini aç / kapat
    ESC        in ve çık (pencereyi kapatmak da aynı)

Gereksinim: pip install pygame av numpy
"""
import argparse
import math
import os
import sys
import time

import pygame

# Doğrudan dosya olarak çalıştırılınca da tellolink bulunsun (proje kökünü yola ekle).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tellolink import Tello

VIDEO = (960, 720)
PANEL_WIDTH = 400
FOOTER_HEIGHT = 48      # pencerenin altındaki tuş listesi
CONTROLS = [
    'T takeoff   L land   ESC land & quit   Space stop   F flip (+W/A/S/D: direction)',
    'W/S forward/back   A/D left/right   Up/Down rise/descend   Left/Right turn',
    '1/2 speed -/+   3/4 vertical -/+   5/6 turn -/+   I data panel',
]
FPS = 30
SPEED_STEP = 0.1
SPEED_MIN, SPEED_MAX = 0.1, 1.0
STALE = 2.0             # bu kadar saniyedir yenilenmeyen veri panelde gri gösterilir

# Hız tuşları: tuş -> (hangi hız, değişim)
SPEED_KEYS = {
    pygame.K_1: ('speed', -SPEED_STEP), pygame.K_2: ('speed', +SPEED_STEP),
    pygame.K_3: ('vertical', -SPEED_STEP), pygame.K_4: ('vertical', +SPEED_STEP),
    pygame.K_5: ('turn', -SPEED_STEP), pygame.K_6: ('turn', +SPEED_STEP),
}


def sticks_from_keys(pressed, speeds):
    """Basılı tuşlardan çubuk değerleri: (pitch, roll, yaw, throttle)."""
    if pressed[pygame.K_SPACE]:
        return 0.0, 0.0, 0.0, 0.0
    pitch = (pressed[pygame.K_w] - pressed[pygame.K_s]) * speeds['speed']
    roll = (pressed[pygame.K_d] - pressed[pygame.K_a]) * speeds['speed']
    yaw = (pressed[pygame.K_RIGHT] - pressed[pygame.K_LEFT]) * speeds['turn']
    throttle = (pressed[pygame.K_UP] - pressed[pygame.K_DOWN]) * speeds['vertical']
    return pitch, roll, yaw, throttle


def flip_direction(pressed):
    """F'ye basılırken tutulan yön tuşlarından takla yönü; hiçbiri yoksa öne."""
    fwd = 'forward' if pressed[pygame.K_w] else 'back' if pressed[pygame.K_s] else ''
    side = 'left' if pressed[pygame.K_a] else 'right' if pressed[pygame.K_d] else ''
    if fwd and side:
        return f'{fwd}_{side}'
    return fwd or side or 'forward'


def euler_deg(imu):
    """IMU kuaterniyonundan (roll, pitch, yaw) derece. Drone'un kendi açılarıyla (osd) 0.1° içinde aynı."""
    w, x, y, z = imu.q0, imu.q1, imu.q2, imu.q3
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def body_velocity(imu, vx, vy):
    """Dünya eksenindeki yatay hızı gövdeye çevirir: (ileri, sağa). Hızlar yaw 0° yönüne göre gelir."""
    yaw = math.radians(euler_deg(imu)[2])
    return math.cos(yaw) * vx + math.sin(yaw) * vy, -math.sin(yaw) * vx + math.cos(yaw) * vy


def xyz(a, b, c, fmt='{:+.2f}'):
    return ' '.join(fmt.format(v) for v in (a, b, c))


def top_lines(drone, speeds):
    """Görüntünün üstündeki satırlar: bağlantı, pil, yükseklik, hızlar."""
    speed_line = f"Speed {speeds['speed']:.1f}  Vertical {speeds['vertical']:.1f}  Turn {speeds['turn']:.1f}   (1-6 to change)"
    if not drone.connected:
        return ['Connecting to Tello... (is this computer on the Tello Wi-Fi?)', speed_line]
    s = drone.status
    if s is None:
        return ['Connected, waiting for data...', speed_line]
    state = 'flying' if s.em_sky else 'on the ground'
    lines = [f'Battery {s.battery_percent} %   Height {s.height:.1f} m   {state}', speed_line]
    if s.battery_low:
        lines.append('LOW BATTERY - land now (L)')
    if drone.frame is None:
        lines.append('Waiting for video...')
    return lines


def panel_sections(drone):
    """Sağ paneldeki bölümler: (başlık, kayıt, satırlar). Kayıt, eskiyse gri göstermek için."""
    out = []
    imu = drone.imu
    if imu:
        roll, pitch, yaw = euler_deg(imu)
        out.append(('Attitude (IMU, 10 Hz)', imu, [
            f'roll  {roll:+7.1f} deg', f'pitch {pitch:+7.1f} deg', f'yaw   {yaw:+7.1f} deg']))
        out.append(('IMU', imu, [
            f'acc      {xyz(imu.acc_x, imu.acc_y, imu.acc_z)} g',
            f'gyro     {xyz(imu.gyro_x, imu.gyro_y, imu.gyro_z)}',
            f'lin acc  {xyz(imu.lin_acc_x, imu.lin_acc_y, imu.lin_acc_z)} m/s2',
            f'vel      {xyz(imu.vel_x, imu.vel_y, imu.vel_z)} m/s (world)',
            'vel body fwd {:+.2f} right {:+.2f} m/s'.format(*body_velocity(imu, imu.vel_x, imu.vel_y)),
            f'baro {imu.baro:.2f} m   temp ' + (f'{imu.temperature:.1f} C' if imu.temperature is not None else '-')]))
    mvo = drone.mvo
    if mvo:
        out.append(('MVO (optical flow, 5 Hz)', mvo, [
            f'vel  {xyz(mvo.vel_x, mvo.vel_y, mvo.vel_z)} m/s (world)',
            f'pos  {xyz(mvo.pos_x, mvo.pos_y, mvo.pos_z)} m',
            'height ' + (f'{mvo.height:.2f} m' if mvo.height is not None else '-')]))
    nav = drone.nav
    if nav:
        out.append(('Nav (2064, 10 Hz)', nav, [
            f'vel  {xyz(nav.vel_x, nav.vel_y, nav.vel_z)} m/s (world)',
            f'pos  {xyz(nav.pos_x, nav.pos_y, nav.pos_z)} m',
            f'height {nav.height:.2f} m']))
    lines = []
    if drone.tof:
        lines.append(f'ToF distance {drone.tof.distance:.2f} m')
    if drone.battery:
        lines.append(f'battery {drone.battery.voltage:.2f} V')
    if drone.motors:
        lines.append('motors % ' + ' '.join(f'{v:.0f}' for v in drone.motors.pwm))
    if drone.wifi:
        lines.append(f'wifi strength {drone.wifi.strength}')
    if lines:
        out.append(('Other', None, lines))
    return out


def draw_panel(screen, rect, drone, font, small):
    pygame.draw.rect(screen, (15, 15, 15), rect)
    y = rect.top + 8
    now = time.monotonic()
    sections = panel_sections(drone)
    if not sections:
        screen.blit(font.render('No data yet', True, (150, 150, 150)), (rect.left + 10, y))
        return
    for title, record, lines in sections:
        stale = record is not None and now - record.recv_time > STALE
        screen.blit(font.render(title + ('  (old)' if stale else ''), True, (120, 200, 255)), (rect.left + 10, y))
        y += 24
        color = (130, 130, 130) if stale else (235, 235, 235)
        for line in lines:
            screen.blit(small.render(line, True, color), (rect.left + 16, y))
            y += 19
        y += 8


def draw_controls(screen, tiny):
    """Pencerenin altına tuş listesini küçük yazıyla yazar."""
    top = VIDEO[1]
    pygame.draw.rect(screen, (30, 30, 30), (0, top, screen.get_width(), FOOTER_HEIGHT))
    for i, line in enumerate(CONTROLS):
        screen.blit(tiny.render(line, True, (200, 200, 200)), (8, top + 4 + 14 * i))


def main():
    parser = argparse.ArgumentParser(description='Fly a Tello with the keyboard.')
    parser.add_argument('--speed', type=float, default=0.3, help='forward/back/side speed, 0.1-1 (default 0.3)')
    parser.add_argument('--vertical', type=float, default=0.3, help='up/down speed, 0.1-1 (default 0.3)')
    parser.add_argument('--turn', type=float, default=0.3, help='turn speed, 0.1-1 (default 0.3)')
    args = parser.parse_args()
    speeds = {k: min(SPEED_MAX, max(SPEED_MIN, getattr(args, k))) for k in ('speed', 'vertical', 'turn')}

    pygame.init()
    show_panel = True
    screen = pygame.display.set_mode((VIDEO[0] + PANEL_WIDTH, VIDEO[1] + FOOTER_HEIGHT))
    pygame.display.set_caption('Tello | T takeoff  L land  WASD move  arrows up/down/turn  F flip  1-6 speed  I panel  ESC quit')
    font = pygame.font.Font(None, 26)
    small = pygame.font.SysFont('consolas', 16) or font
    tiny = pygame.font.SysFont('consolas', 12) or small
    clock = pygame.time.Clock()

    drone = Tello()
    print('Connecting to Tello...')
    try:
        drone.connect(timeout=0)       # beklemeden dön; bağlantı arka planda kurulur
    except TimeoutError:
        pass
    except OSError as e:
        print(f'Could not open the connection: {e}')
        pygame.quit()
        return

    was_connected = False
    last_frame = None
    picture = None
    try:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_t and drone.connected:
                        print('Taking off...')
                        drone.takeoff()
                    elif event.key == pygame.K_l and drone.connected:
                        print('Landing...')
                        drone.land()
                    elif event.key == pygame.K_f and drone.connected:
                        direction = flip_direction(pygame.key.get_pressed())
                        print(f'Flip {direction}...')
                        drone.flip(direction)
                    elif event.key in SPEED_KEYS:
                        name, step = SPEED_KEYS[event.key]
                        speeds[name] = round(min(SPEED_MAX, max(SPEED_MIN, speeds[name] + step)), 1)
                    elif event.key == pygame.K_i:
                        show_panel = not show_panel
                        width = VIDEO[0] + (PANEL_WIDTH if show_panel else 0)
                        screen = pygame.display.set_mode((width, VIDEO[1] + FOOTER_HEIGHT))

            if drone.connected != was_connected:
                was_connected = drone.connected
                print('Connected.' if was_connected else 'Connection lost, reconnecting...')

            # Pencere odakta değilse drone dursun (başka pencerede yazarken uçmasın).
            if pygame.key.get_focused():
                drone.set_sticks(*sticks_from_keys(pygame.key.get_pressed(), speeds))
            else:
                drone.set_sticks(0, 0, 0, 0)

            # Yeni kare geldiyse resme çevir (BGR -> RGB).
            frame = drone.frame
            if frame is not None and frame is not last_frame:
                last_frame = frame
                rgb = frame.image[:, :, ::-1]
                picture = pygame.transform.scale(pygame.surfarray.make_surface(rgb.swapaxes(0, 1)), VIDEO)

            screen.fill((0, 0, 0))
            if picture is not None:
                screen.blit(picture, (0, 0))
            lines = top_lines(drone, speeds)
            pygame.draw.rect(screen, (20, 20, 20), (0, 0, VIDEO[0], 10 + 26 * len(lines)))
            for i, line in enumerate(lines):
                color = (255, 90, 90) if 'LOW' in line else (255, 255, 255)
                screen.blit(font.render(line, True, color), (10, 6 + 26 * i))
            if show_panel:
                draw_panel(screen, pygame.Rect(VIDEO[0], 0, PANEL_WIDTH, VIDEO[1]), drone, font, small)
            draw_controls(screen, tiny)
            pygame.display.flip()
            clock.tick(FPS)
    except KeyboardInterrupt:
        pass
    finally:
        drone.set_sticks(0, 0, 0, 0)
        if drone.connected and drone.status is not None and drone.status.em_sky:
            print('Landing before quit...')
            drone.land()
            time.sleep(5)
        drone.close()
        pygame.quit()
        print('Disconnected. Done.')


if __name__ == '__main__':
    main()
