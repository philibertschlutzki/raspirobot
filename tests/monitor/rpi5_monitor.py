#!/usr/bin/env python3
"""
Raspberry Pi 5 System Monitor
Zeigt Temperatur, Spannungen, Throttling und Leistungsaufnahme
"""

import subprocess
import time
import re
from datetime import datetime
import monitor_utils

class RaspberryPiMonitor:
    """Klasse zur Überwachung von Raspberry Pi 5 Systemwerten"""

    def __init__(self):
        self.throttle_meanings = {
            0x1: "Unterspannung erkannt",
            0x2: "ARM-Frequenz begrenzt",
            0x4: "Aktuell gedrosselt",
            0x8: "Soft-Temperaturlimit erreicht",
            0x10000: "Unterspannung ist aufgetreten",
            0x20000: "Frequenzbegrenzung ist aufgetreten",
            0x40000: "Throttling ist aufgetreten",
            0x80000: "Soft-Temperaturlimit wurde erreicht"
        }

    def run_command(self, cmd):
        """Führt Shell-Befehl aus und gibt Ausgabe zurück"""
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            return result.stdout.strip()
        except Exception as e:
            return f"Fehler: {e}"

    def get_temperature(self):
        """Liest CPU-Temperatur aus"""
        temp_str = self.run_command("vcgencmd measure_temp")
        if "temp=" in temp_str:
            temp = temp_str.replace("temp=", "").replace("'C", "")
            return float(temp)
        return None

    def get_cpu_freq(self):
        """Liest ARM-CPU-Frequenz aus"""
        freq_str = self.run_command("vcgencmd measure_clock arm")
        if freq_str and "=" in freq_str:
            freq_hz = int(freq_str.split("=")[1])
            return freq_hz / 1_000_000  # Konvertierung zu MHz
        return None

    def get_voltages(self):
        """Liest Standard-Spannungen aus"""
        return monitor_utils.get_voltages(self.run_command)

    def get_pmic_data(self):
        """Liest detaillierte PMIC-Daten (Spannungen und Ströme)"""
        pmic_output = self.run_command("vcgencmd pmic_read_adc")

        currents = {}
        voltages = {}

        for line in pmic_output.split('\n'):
            line = line.strip()
            if not line:
                continue

            # Extrahiere Ströme
            if "current" in line:
                match = re.search(r'([\w_]+)\s+current\(\d+\)=([\d.]+)A', line)
                if match:
                    name, value = match.groups()
                    currents[name] = float(value)

            # Extrahiere Spannungen
            elif "volt" in line and "current" not in line:
                match = re.search(r'([\w_]+)\s+volt\(\d+\)=([\d.]+)V', line)
                if match:
                    name, value = match.groups()
                    voltages[name] = float(value)

        return currents, voltages

    def calculate_power(self, currents, voltages):
        """Berechnet Gesamtleistung aus PMIC-Daten"""
        total_power = 0.0
        power_details = {}

        for current_name, current_value in currents.items():
            # Finde entsprechende Spannung
            voltage_name = current_name.replace("_A", "_V")
            if voltage_name in voltages:
                power = current_value * voltages[voltage_name]
                power_details[current_name.replace("_A", "")] = power
                total_power += power

        # Lineare Korrektur basierend auf Kalibrierung
        # Quelle: https://github.com/jfikar/RPi5-power
        corrected_power = total_power * 1.1451 + 0.5879

        return total_power, corrected_power, power_details

    def get_throttled_status(self):
        """Liest Throttling-Status aus"""
        throttle_str = self.run_command("vcgencmd get_throttled")
        if "throttled=" in throttle_str:
            value_str = throttle_str.split("=")[1]
            throttle_value = int(value_str, 16)

            active_flags = []
            for flag, meaning in self.throttle_meanings.items():
                if throttle_value & flag:
                    active_flags.append(meaning)

            return throttle_value, active_flags
        return None, []

    def format_temperature_status(self, temp):
        """Formatiert Temperatur mit Status-Indikator"""
        if temp is None:
            return "N/A", ""

        if temp < 60:
            status = "✓ Normal"
        elif temp < 80:
            status = "⚠ Erhöht"
        elif temp < 85:
            status = "⚠ Throttling aktiv"
        else:
            status = "❌ Kritisch!"

        return f"{temp:.1f}°C", status

    def print_system_info(self):
        """Gibt komplette Systeminformationen formatiert aus"""
        print("\n" + "="*70)
        print(f"  RASPBERRY PI 5 SYSTEM-MONITOR")
        print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*70)

        # Temperatur
        temp = self.get_temperature()
        temp_str, temp_status = self.format_temperature_status(temp)
        print(f"\n📊 TEMPERATUR & TAKTFREQUENZ")
        print(f"  CPU-Temperatur:      {temp_str:>12}  {temp_status}")

        # CPU-Frequenz
        freq = self.get_cpu_freq()
        if freq:
            print(f"  ARM-Taktfrequenz:    {freq:>9.0f} MHz")

        # Throttling-Status
        throttle_value, throttle_flags = self.get_throttled_status()
        print(f"\n⚡ THROTTLING-STATUS")
        print(f"  Status-Code:         {hex(throttle_value) if throttle_value is not None else 'N/A'}")
        if throttle_flags:
            for flag in throttle_flags:
                print(f"    ⚠ {flag}")
        else:
            print(f"    ✓ Keine Probleme erkannt")

        # Standard-Spannungen
        print(f"\n🔋 SPANNUNGEN (Standard)")
        voltages = self.get_voltages()
        for desc, volt in voltages.items():
            print(f"  {desc:<15}  {volt:>6.3f} V")

        # PMIC-Daten
        currents, pmic_voltages = self.get_pmic_data()

        if currents and pmic_voltages:
            print(f"\n🔌 PMIC-DETAILS (Top Verbraucher)")

            # Berechne Leistung pro Zweig
            power_per_rail = {}
            for current_name, current_value in currents.items():
                voltage_name = current_name.replace("_A", "_V")
                if voltage_name in pmic_voltages:
                    power = current_value * pmic_voltages[voltage_name]
                    rail_name = current_name.replace("_A", "")
                    power_per_rail[rail_name] = {
                        'voltage': pmic_voltages[voltage_name],
                        'current': current_value,
                        'power': power
                    }

            # Sortiere nach Leistung
            sorted_rails = sorted(power_per_rail.items(), 
                                key=lambda x: x[1]['power'], 
                                reverse=True)[:8]

            print(f"  {'Zweig':<14} {'Spannung':>10} {'Strom':>10} {'Leistung':>10}")
            print(f"  {'-'*14} {'-'*10} {'-'*10} {'-'*10}")

            for rail, data in sorted_rails:
                print(f"  {rail:<14} {data['voltage']:>8.2f} V "
                      f"{data['current']:>8.4f} A {data['power']:>8.3f} W")

        # Leistungsaufnahme
        if currents and pmic_voltages:
            total_power, corrected_power, power_details = self.calculate_power(
                currents, pmic_voltages
            )

            print(f"\n⚡ LEISTUNGSAUFNAHME")
            print(f"  PMIC-Summe (gemessen):    {total_power:>6.2f} W")
            print(f"  Geschätzt (korrigiert):   {corrected_power:>6.2f} W")
            print(f"  Hinweis: Exklusive USB-Geräte und HATs")

        print("\n" + "="*70 + "\n")

def main():
    """Hauptfunktion"""
    import argparse

    parser = argparse.ArgumentParser(
        description="Raspberry Pi 5 System-Monitor"
    )
    parser.add_argument(
        '-c', '--continuous',
        action='store_true',
        help='Kontinuierliche Überwachung (Aktualisierung alle 2 Sekunden)'
    )
    parser.add_argument(
        '-i', '--interval',
        type=int,
        default=2,
        help='Aktualisierungsintervall in Sekunden (Standard: 2)'
    )

    args = parser.parse_args()

    monitor = RaspberryPiMonitor()

    if args.continuous:
        try:
            while True:
                # Bildschirm löschen
                print("\033[2J\033[H", end="")
                monitor.print_system_info()
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\n\nÜberwachung beendet.")
    else:
        monitor.print_system_info()

if __name__ == "__main__":
    main()
