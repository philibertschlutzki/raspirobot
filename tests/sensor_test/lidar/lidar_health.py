#!/usr/bin/env python3

"""
RPLIDAR C1 Robuster Test mit 2D-Visualisierung
Installation: pip install rplidar-roboticia matplotlib numpy
"""

import argparse
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rplidar import RPLidar, RPLidarException


def reset_lidar_connection(port: str, baud: int, timeout: float) -> RPLidar:
    """
    Hard-Reset der LIDAR-Verbindung mit vollständigem Cleanup.
    Behebt 'Descriptor length mismatch' durch Buffer-Clear.
    """
    lidar = None
    try:
        # Erste Verbindung zum Cleanup
        print(f"Initialisiere LIDAR auf {port} @ {baud} Baud...")
        lidar = RPLidar(port, baudrate=baud, timeout=timeout)
        
        # Explizit stoppen und Motor ausschalten
        try:
            lidar.stop()
            time.sleep(0.3)
        except Exception:
            pass
            
        try:
            lidar.stop_motor()
            time.sleep(0.3)
        except Exception:
            pass
            
        # Verbindung trennen für Buffer-Reset
        lidar.disconnect()
        lidar = None
        time.sleep(1.0)  # Hardware-Zeit für Reset
        
        # Neue saubere Verbindung
        print("Stelle neue Verbindung her...")
        lidar = RPLidar(port, baudrate=baud, timeout=timeout)
        
        # Geräteinfo abrufen (validiert Connection)
        info = lidar.get_info()
        print(f"Geräteinfo: {info}")
        
        health = lidar.get_health()
        print(f"Health: {health}")
        
        status = health[0] if isinstance(health, tuple) else health.get('status', 'Unknown')
        if status == 'Error':
            raise RuntimeError('Gerät meldet Fehlerzustand')
        
        return lidar
        
    except Exception as e:
        if lidar:
            try:
                lidar.disconnect()
            except Exception:
                pass
        raise e


def collect_scan_data(lidar: RPLidar, num_scans: int = 5, max_buf: int = 5000):
    """
    Sammelt mehrere Scans und gibt aggregierte Polar-Daten zurück.
    """
    print(f"\nStarte Motor und warte auf Stabilisierung...")
    
    try:
        lidar.start_motor()
        time.sleep(2.0)  # Längere Wartezeit für Motor-Stabilisierung
    except Exception as e:
        print(f"Warnung: start_motor() - {e}")
    
    all_angles = []
    all_distances = []
    
    print(f"Sammle {num_scans} Scans...")
    
    try:
        iterator = lidar.iter_scans(max_buf_meas=max_buf, min_len=10)
        
        for scan_idx in range(num_scans):
            try:
                scan = next(iterator)
                points = len(scan)
                valid = sum(1 for _, _, d in scan if d > 0)
                
                print(f"  Scan {scan_idx + 1}/{num_scans}: {points} Punkte ({valid} gültig)")
                
                for quality, angle, distance in scan:
                    if distance > 0:  # Nur gültige Messungen
                        all_angles.append(angle)
                        all_distances.append(distance)
                        
            except StopIteration:
                print("Iterator beendet vorzeitig")
                break
            except RPLidarException as e:
                print(f"Scan-Fehler ignoriert: {e}")
                continue
                
    except RPLidarException as e:
        print(f"Kritischer Scan-Fehler: {e}")
        raise
    finally:
        print("\nStoppe LIDAR...")
        try:
            lidar.stop()
        except Exception:
            pass
        try:
            lidar.stop_motor()
        except Exception:
            pass
    
    return np.array(all_angles), np.array(all_distances)


def create_2d_map(angles, distances, output_file='lidar_map_2d.png'):
    """
    Erstellt eine 2D-Polar-Plot-Karte aus LIDAR-Daten.
    """
    if len(angles) == 0:
        print("Keine Daten für Visualisierung vorhanden!")
        return
    
    print(f"\nErstelle 2D-Karte mit {len(angles)} Messpunkten...")
    
    # Konvertiere zu Radians für Polar-Plot
    angles_rad = np.deg2rad(angles)
    
    # Erstelle Figure mit zwei Subplots
    fig = plt.figure(figsize=(16, 7))
    
    # Subplot 1: Polar Plot (traditionelle LIDAR-Ansicht)
    ax1 = fig.add_subplot(121, projection='polar')
    ax1.scatter(angles_rad, distances, c=distances, cmap='jet', s=1, alpha=0.6)
    ax1.set_theta_zero_location('N')
    ax1.set_theta_direction(-1)
    ax1.set_ylim(0, max(distances) * 1.1)
    ax1.set_title('RPLIDAR C1 - Polar View', pad=20, fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    
    # Subplot 2: Kartesische 2D-Karte (Top-Down View)
    ax2 = fig.add_subplot(122)
    
    # Konvertiere Polar zu Kartesisch (LIDAR im Zentrum)
    x = distances * np.cos(angles_rad)
    y = distances * np.sin(angles_rad)
    
    scatter = ax2.scatter(x, y, c=distances, cmap='jet', s=1, alpha=0.6)
    ax2.plot(0, 0, 'r+', markersize=15, markeredgewidth=2)  # LIDAR-Position
    ax2.set_xlabel('X [mm]', fontsize=12)
    ax2.set_ylabel('Y [mm]', fontsize=12)
    ax2.set_title('RPLIDAR C1 - Top-Down Map', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    # Colorbar
    cbar = plt.colorbar(scatter, ax=ax2, label='Distanz [mm]')
    
    # Statistik-Text
    stats_text = (
        f"Messpunkte: {len(angles)}\n"
        f"Distanz: {distances.min():.0f} - {distances.max():.0f} mm\n"
        f"Mittelwert: {distances.mean():.0f} mm"
    )
    ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"2D-Karte gespeichert: {output_file}")
    
    # Optional: Anzeigen (nur wenn Display verfügbar)
    try:
        plt.show(block=False)
        plt.pause(3)
    except Exception:
        print("Kein Display verfügbar - nur Datei gespeichert")
    finally:
        plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='RPLIDAR C1 Robuster Test mit 2D-Visualisierung'
    )
    parser.add_argument('--port', default='/dev/ttyUSB0',
                        help='Serieller Port')
    parser.add_argument('--baud', type=int, default=460800,
                        help='Baudrate (C1: 460800)')
    parser.add_argument('--timeout', type=float, default=2.0,
                        help='Timeout in Sekunden')
    parser.add_argument('--scans', type=int, default=10,
                        help='Anzahl Scans für Karte')
    parser.add_argument('--output', default='lidar_map_2d.png',
                        help='Output-Datei für 2D-Karte')
    parser.add_argument('--max-buf', type=int, default=5000,
                        help='Buffer-Limit für iter_scans')
    
    args = parser.parse_args()
    
    lidar = None
    
    try:
        # Robuste Verbindung mit Reset
        lidar = reset_lidar_connection(args.port, args.baud, args.timeout)
        
        # Daten sammeln
        angles, distances = collect_scan_data(lidar, args.scans, args.max_buf)
        
        if len(angles) == 0:
            print("\nKeine gültigen Daten erfasst!")
            return 1
        
        # 2D-Karte erstellen
        create_2d_map(angles, distances, args.output)
        
        print("\n=== Erfolgreich abgeschlossen ===")
        print(f"Gesamte Messpunkte: {len(angles)}")
        print(f"Distanzbereich: {distances.min():.0f} - {distances.max():.0f} mm")
        print(f"Karte gespeichert: {args.output}")
        
        return 0
        
    except RPLidarException as e:
        print(f"\nRPLidarException: {e}", file=sys.stderr)
        print("\nFehlerbehebung:", file=sys.stderr)
        print("1. LIDAR physisch trennen und neu verbinden", file=sys.stderr)
        print("2. Anderen USB-Port versuchen", file=sys.stderr)
        print("3. Timeout erhöhen: --timeout 3.0", file=sys.stderr)
        return 1
        
    except KeyboardInterrupt:
        print("\nAbgebrochen durch Benutzer")
        return 130
        
    except Exception as e:
        print(f"\nFehler: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1
        
    finally:
        if lidar:
            try:
                lidar.stop()
            except Exception:
                pass
            try:
                lidar.stop_motor()
            except Exception:
                pass
            try:
                lidar.disconnect()
            except Exception:
                pass


if __name__ == '__main__':
    sys.exit(main())
