# Copyright 2026 batoddy
# Licensed under the Apache License, Version 2.0 (see LICENSE).
"""tellolink example: connects, prints the data once a second for ~10 s, shows the video.

Usage:
    python tellolink/example.py            # does not take off, only reads
    python tellolink/example.py --fly      # takes off, waits 3 s, lands (keep 2 m of free space around)
"""
import argparse
import os
import sys
import time

# Make tellolink importable when run directly as a file (add the project root to the path).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tellolink import Tello

try:
    import cv2
except ImportError:
    cv2 = None      # without OpenCV the video is not shown, only its size is printed


def show(drone):
    """Shows the latest frame (does nothing without cv2)."""
    frame = drone.frame
    if frame is not None and cv2 is not None:
        cv2.imshow('tello', frame.image)
        cv2.waitKey(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fly', action='store_true', help='take off, wait, land')
    parser.add_argument('--ucus', dest='fly', action='store_true', help=argparse.SUPPRESS)   # old name
    args = parser.parse_args()

    drone = Tello()

    # Event: called after each IMU record is written to drone.imu (in the library's thread; keep it short).
    imu_count = [0]
    def on_imu(drone, data):
        imu_count[0] += 1
    drone.subscribe(drone.EVENT_IMU, on_imu)    # before connect, so the first records are not missed

    def on_disconnected(drone, data):
        print('Connection lost, reconnecting...')
    drone.subscribe(drone.EVENT_DISCONNECTED, on_disconnected)

    print('Connecting to Tello... (is this computer on the Tello Wi-Fi?)')
    try:
        drone.connect(timeout=10)
    except TimeoutError:
        print('Could not connect to Tello: no answer in 10 s. Check the Wi-Fi connection and that the drone is on.')
        drone.close()
        return
    except OSError as e:
        print(f'Could not connect to Tello: {e}')
        drone.close()
        return
    print('Connected.')
    try:
        if args.fly:
            # Takeoff: wait 3 s in the air, then land.
            print('Taking off...')
            drone.takeoff()
            time.sleep(3)

        print('Reading data for 10 s...' + ('' if cv2 is not None else ' (OpenCV not installed, video window disabled)'))

        imu_count[0] = 0
        end = time.monotonic() + 10
        next_print = time.monotonic() + 1
        while time.monotonic() < end:
            show(drone)
            if time.monotonic() >= next_print:
                next_print += 1
                status = drone.status
                imu = drone.imu
                nav = drone.nav
                tof = drone.tof
                battery = drone.battery
                frame = drone.frame
                count, imu_count[0] = imu_count[0], 0   # number of IMU records the callback received in the last 1 s
                print('---')
                if not (status or imu or frame):
                    print('no data yet')
                if status:
                    print(f'status: height {status.height:.1f} m, battery {status.battery_percent} %, '
                          f'{status.battery_voltage:.2f} V, flying {status.em_sky}')
                if imu:
                    print(f'imu: tick {imu.tick}, vel ({imu.vel_x:.2f}, {imu.vel_y:.2f}, {imu.vel_z:.2f}) m/s, '
                          f'{count} records in the last 1 s')
                if nav:
                    print(f'nav: pos ({nav.pos_x:.2f}, {nav.pos_y:.2f}) m, height {nav.height:.2f} m')
                if tof:
                    print(f'tof: {tof.distance:.3f} m')
                if battery:
                    print(f'battery: {battery.voltage:.2f} V')
                if frame:
                    print(f'frame: {frame.image.shape[1]}x{frame.image.shape[0]}, keyframe {frame.keyframe}')
            time.sleep(0.02)

        if args.fly:
            print('Landing...')
            drone.land()
            time.sleep(5)                       # wait for the landing to finish
    except KeyboardInterrupt:
        print('Stopped by user.')
    finally:
        if args.fly:
            drone.land()                        # send land even if an error occurred
        drone.close()
        if cv2 is not None:
            cv2.destroyAllWindows()
        print('Disconnected. Done.')


if __name__ == '__main__':
    main()
