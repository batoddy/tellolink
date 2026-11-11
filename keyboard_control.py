# Copyright 2026 batoddy
# Licensed under the Apache License, Version 2.0 (see LICENSE).
"""Fly with tellolink from the keyboard: live video, adjustable speeds, flips and a live data panel.

Usage (with the computer connected to the drone's Wi-Fi network):
    python tellolink/keyboard_control.py
    python tellolink/keyboard_control.py --speed 0.4 --vertical 0.3 --turn 0.5   # initial speeds

Keys:
    T          take off                  L        land
    W / S      forward / back            A / D    strafe left / right
    ↑ / ↓      rise / descend            ← / →    turn left / right
    Space      while held, stop (hover in place)
    F          flip (toward W/A/S/D if held, otherwise forward)
    1 / 2      move speed − / +          3 / 4    climb speed − / +
    5 / 6      turn speed − / +
    I          show / hide the data panel
    ESC        land and quit (closing the window does the same)

Requirements: pip install pygame av numpy
"""
import argparse
import math
import os
import sys
import time

import pygame

# Make tellolink importable when run directly as a file (add the project root to the path).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tellolink import Tello

VIDEO = (960, 720)
PANEL_WIDTH = 400
FOOTER_HEIGHT = 48      # key list at the bottom of the window
CONTROLS = [
    'T takeoff   L land   ESC land & quit   Space stop   F flip (+W/A/S/D: direction)',
    'W/S forward/back   A/D left/right   Up/Down rise/descend   Left/Right turn',
    '1/2 speed -/+   3/4 vertical -/+   5/6 turn -/+   I data panel',
]
FPS = 30
SPEED_STEP = 0.1
SPEED_MIN, SPEED_MAX = 0.1, 1.0
STALE = 2.0             # data not refreshed for this many seconds is shown gray in the panel

# Speed keys: key -> (which speed, change)
SPEED_KEYS = {
    pygame.K_1: ('speed', -SPEED_STEP), pygame.K_2: ('speed', +SPEED_STEP),
    pygame.K_3: ('vertical', -SPEED_STEP), pygame.K_4: ('vertical', +SPEED_STEP),
    pygame.K_5: ('turn', -SPEED_STEP), pygame.K_6: ('turn', +SPEED_STEP),
}


def sticks_from_keys(pressed, speeds):
    """Stick values from the pressed keys: (pitch, roll, yaw, throttle)."""
    if pressed[pygame.K_SPACE]:
        return 0.0, 0.0, 0.0, 0.0
    pitch = (pressed[pygame.K_w] - pressed[pygame.K_s]) * speeds['speed']
    roll = (pressed[pygame.K_d] - pressed[pygame.K_a]) * speeds['speed']
    yaw = (pressed[pygame.K_RIGHT] - pressed[pygame.K_LEFT]) * speeds['turn']
    throttle = (pressed[pygame.K_UP] - pressed[pygame.K_DOWN]) * speeds['vertical']
    return pitch, roll, yaw, throttle


def flip_direction(pressed):
    """Flip direction from the direction keys held while F is pressed; forward if none."""
    fwd = 'forward' if pressed[pygame.K_w] else 'back' if pressed[pygame.K_s] else ''
    side = 'left' if pressed[pygame.K_a] else 'right' if pressed[pygame.K_d] else ''
    if fwd and side:
        return f'{fwd}_{side}'
    return fwd or side or 'forward'


def euler_deg(imu):
    """(roll, pitch, yaw) in degrees from the IMU quaternion. Within 0.1° of the drone's own angles (osd)."""
    w, x, y, z = imu.q0, imu.q1, imu.q2, imu.q3
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def body_velocity(imu, vx, vy):
    """Converts the horizontal velocity from the world frame to the body frame: (forward, right). Velocities are relative to the yaw 0° direction."""
    yaw = math.radians(euler_deg(imu)[2])
    return math.cos(yaw) * vx + math.sin(yaw) * vy, -math.sin(yaw) * vx + math.cos(yaw) * vy


def xyz(a, b, c, fmt='{:+.2f}'):
    return ' '.join(fmt.format(v) for v in (a, b, c))


def top_lines(drone, speeds):
    """Lines at the top of the video: connection, battery, height, speeds."""
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
    """Sections of the right panel: (title, record, lines). The record is used to show stale data in gray."""
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
    """Draws the key list in small text at the bottom of the window."""
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
        drone.connect(timeout=0)       # return without waiting; the connection is made in the background
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

            # Stop the drone if the window is not focused (so it does not fly while typing in another window).
            if pygame.key.get_focused():
                drone.set_sticks(*sticks_from_keys(pygame.key.get_pressed(), speeds))
            else:
                drone.set_sticks(0, 0, 0, 0)

            # If a new frame has arrived, convert it to a picture (BGR -> RGB).
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
