"""tellolink örneği: bağlanır, ~10 s boyunca saniyede bir verileri yazdırır, görüntüyü gösterir.

Çalıştırma:
    python tellolink/example.py            # kalkmaz, sadece okur
    python tellolink/example.py --ucus     # kalkar, 3 s bekler, iner (çevrede 2 m boş alan olsun)
"""
import argparse
import os
import sys
import time

# Doğrudan dosya olarak çalıştırılınca da tellolink bulunsun (proje kökünü yola ekle).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tellolink import Tello

try:
    import cv2
except ImportError:
    cv2 = None      # OpenCV yoksa görüntü gösterilmez, sadece boyutu yazılır


def show(drone):
    """Son kareyi gösterir (cv2 yoksa bir şey yapmaz)."""
    frame = drone.frame
    if frame is not None and cv2 is not None:
        cv2.imshow('tello', frame.image)
        cv2.waitKey(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ucus', action='store_true', help='kalk, bekle, in')
    args = parser.parse_args()

    drone = Tello()

    # Olay: her IMU kaydı drone.imu'ya yazıldıktan sonra çağrılır (kütüphanenin thread'inde; kısa tutun).
    imu_count = [0]
    def on_imu(drone, data):
        imu_count[0] += 1
    drone.subscribe(drone.EVENT_IMU, on_imu)    # connect'ten önce: ilk kayıtlar kaçmasın

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
        if args.ucus:
            # Kalkış: 3 s havada bekle, sonra in.
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
                count, imu_count[0] = imu_count[0], 0   # son 1 s'de callback'e gelen IMU kaydı sayısı
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

        if args.ucus:
            print('Landing...')
            drone.land()
            time.sleep(5)                       # inişin bitmesini bekle
    except KeyboardInterrupt:
        print('Stopped by user.')
    finally:
        if args.ucus:
            drone.land()                        # hata olursa da iniş gönder
        drone.close()
        if cv2 is not None:
            cv2.destroyAllWindows()
        print('Disconnected. Done.')


if __name__ == '__main__':
    main()
