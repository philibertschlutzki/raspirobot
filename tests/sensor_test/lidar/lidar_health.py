#!/usr/bin/env python3
# RPLIDAR C1 Output-Tester (Raspberry Pi 5) – robuste Verbindung, Health-Check, CSV, Metriken
# Installation: pip install rplidar-roboticia
# C1: UART 460800 8N1 (USB/UART-Adapter), typische ~10 Hz, ~5000 Messungen/s, 12 m Reichweite

import argparse
import csv
import statistics
import sys
import time
from datetime import datetime

from rplidar import RPLidar, RPLidarException  # aus rplidar-roboticia

def human_time(ts: float) -> str:
    return datetime.fromtimestamp(ts).isoformat(timespec='milliseconds')

def connect_lidar(port: str, baud: int, timeout: float, retries: int = 1, pause: float = 0.8) -> RPLidar:
    """
    Stellt die Verbindung her und versucht bei Deskriptor-/Sync-Fehlern einmalig einen Reconnect.
    """
    attempt = 0
    last_exc = None
    while attempt <= retries:
        try:
            lidar = RPLidar(port, baudrate=baud, timeout=timeout)
            # get_info() triggert den Descriptor-/Header-Handshake und validiert die Session
            _ = lidar.get_info()
            return lidar
        except RPLidarException as e:
            last_exc = e
            # sauber schließen, kurze Pause, Retry
            try:
                if 'lidar' in locals() and lidar:
                    try:
                        lidar.stop()
                    except Exception:
                        pass
                    try:
                        lidar.stop_motor()
                    except Exception:
                        pass
                    lidar.disconnect()
            except Exception:
                pass
            if attempt < retries:
                time.sleep(pause)
            attempt += 1
    raise last_exc if last_exc else RuntimeError('Verbindungsfehler ohne detaillierte Exception')

def main():
    ap = argparse.ArgumentParser(description='RPLIDAR C1 Output-Tester (CSV + Metriken, robust)')
    ap.add_argument('--port', default='/dev/ttyUSB0', help='Serieller Port (z.B. /dev/ttyUSB0)')
    ap.add_argument('--baud', type=int, default=460800, help='Baudrate (C1: 460800)')
    ap.add_argument('--timeout', type=float, default=1.0, help='Serielle Timeout-Sekunden')
    ap.add_argument('--scans', type=int, default=10, help='Anzahl kompletter Scans (Umdrehungen)')
    ap.add_argument('--duration', type=float, default=0.0, help='Alternative: Dauer in Sekunden statt fester Scananzahl')
    ap.add_argument('--csv', default='rplidar_c1_test.csv', help='CSV-Ausgabe-Datei')
    ap.add_argument('--max-buf-meas', type=int, default=5000, help='Bufferlimit für iter_scans()')
    ap.add_argument('--min-len', type=int, default=5, help='Mindestanzahl Messpunkte pro Scan (iter_scans)')
    args = ap.parse_args()

    lidar = None
    csv_file = None
    writer = None

    total_points = 0
    valid_points = 0
    zero_points = 0
    distances_mm = []
    qualities = []
    angle_min = 360.0
    angle_max = 0.0
    scans_timestamps = []
    per_scan_counts = []

    try:
        print(f'Öffne LIDAR auf {args.port} @ {args.baud} Baud...')
        lidar = connect_lidar(args.port, args.baud, args.timeout, retries=1, pause=0.8)

        info = lidar.get_info()
        print('Geräteinfo:', info)
        health = lidar.get_health()
        print('Health:', health)
        status = health[0] if isinstance(health, tuple) else health.get('status', 'Unknown')
        if status == 'Error':
            print('Gerät meldet Fehlerzustand – bitte prüfen/neu starten.')
            return 2

        # Motorstart (bei C1 oft automatisch, Aufruf schadet nicht)
        try:
            lidar.start_motor()
        except Exception as e:
            print(f'Warnung: start_motor() nicht notwendig/fehlgeschlagen: {e}')

        time.sleep(0.4)

        # CSV
        csv_file = open(args.csv, 'w', newline='')
        writer = csv.writer(csv_file)
        writer.writerow(['ts_iso', 'ts_epoch', 'scan_idx', 'point_idx', 'angle_deg', 'distance_mm', 'quality'])

        print('Starte Scans...')
        start_time = time.time()
        # FIX: korrekter Parametername ohne Tippfehler
        iterator = lidar.iter_scans(max_buf_meas=args.max_buf_meas, min_len=args.min_len)

        scan_idx = 0
        while True:
            t0 = time.time()
            scan = next(iterator)
            t1 = time.time()

            scans_timestamps.append(t1)
            per_scan_counts.append(len(scan))

            for p_idx, (quality, angle, distance) in enumerate(scan):
                total_points += 1
                if distance > 0:
                    valid_points += 1
                    distances_mm.append(distance)
                    qualities.append(quality)
                    if angle < angle_min:
                        angle_min = angle
                    if angle > angle_max:
                        angle_max = angle
                else:
                    zero_points += 1

                writer.writerow([human_time(t1), f'{t1:.6f}', scan_idx, p_idx, f'{angle:.3f}', int(distance), int(quality)])

            print(f'Scan {scan_idx}: {len(scan)} Punkte in {(t1 - t0)*1000:.1f} ms')
            scan_idx += 1

            if args.duration > 0.0:
                if (time.time() - start_time) >= args.duration:
                    break
            else:
                if scan_idx >= args.scans:
                    break

        print('Stoppe LIDAR...')
        try:
            lidar.stop()
        except Exception:
            pass
        try:
            lidar.stop_motor()
        except Exception:
            pass

        elapsed = time.time() - start_time
        scans_done = len(per_scan_counts)
        scans_per_sec = (scans_done / elapsed) if elapsed > 0 else 0.0
        rpm = scans_per_sec * 60.0

        print('\n=== Zusammenfassung ===')
        print(f'Umdrehungen (Scans): {scans_done}')
        print(f'Gesamtpunkte: {total_points}')
        print(f'Gültige Punkte (>0 mm): {valid_points}')
        print(f'Ungültige Punkte (0 mm): {zero_points}')
        if distances_mm:
            print(f'Distanz mm: min={min(distances_mm)}, max={max(distances_mm)}, mean={int(statistics.fmean(distances_mm))}')
        if qualities:
            print(f'Qualität: min={min(qualities)}, max={max(qualities)}, mean={int(statistics.fmean(qualities))}')
        print(f'Winkelabdeckung je Scan (global beobachtet): {angle_min:.2f}° .. {angle_max:.2f}°')
        print(f'Durchschnittliche Punkte/Scan: {int(statistics.fmean(per_scan_counts)) if per_scan_counts else 0}')
        print(f'Scanrate: {scans_per_sec:.2f} Hz  (≈ {rpm:.0f} RPM)')
        print(f'\nCSV geschrieben: {args.csv}')

    except RPLidarException as e:
        print(f'RPLidarException: {e}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Abgebrochen.')
    finally:
        try:
            if lidar:
                try:
                    lidar.stop()
                except Exception:
                    pass
                try:
                    lidar.stop_motor()
                except Exception:
                    pass
                lidar.disconnect()
        except Exception:
            pass
        try:
            if csv_file:
                csv_file.flush()
                csv_file.close()
        except Exception:
            pass

if __name__ == '__main__':
    sys.exit(main())
