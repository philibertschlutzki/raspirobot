#!/usr/bin/env python3
"""
Raspberry Pi 5 System Monitor - Erweiterte Version
Zeigt Temperatur, Spannungen, Taktfrequenzen, Systemauslastung und mehr
"""

import subprocess
import time
import re
import os
from datetime import datetime

class RaspberryPiMonitor:
    """Erweiterte Klasse zur Überwachung von Raspberry Pi 5 Systemwerten"""

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

    def read_file(self, filepath):
        """Liest Datei aus und gibt Inhalt zurück"""
        try:
            with open(filepath, 'r') as f:
                return f.read().strip()
        except Exception:
            return None

    def get_temperature(self):
        """Liest CPU-Temperatur aus"""
        temp_str = self.run_command("vcgencmd measure_temp")
        if "temp=" in temp_str:
            temp = temp_str.replace("temp=", "").replace("'C", "")
            return float(temp)
        return None

    def get_all_frequencies(self):
        """Liest alle verfügbaren Taktfrequenzen aus"""
        freq_types = {
            "arm": "ARM CPU",
            "core": "GPU Core",
            "v3d": "3D Block",
            "isp": "Image Sensor",
            "h264": "H.264 Codec",
            "hdmi": "HDMI",
            "uart": "UART",
            "emmc": "SD-Karte"
        }

        frequencies = {}
        for ftype, description in freq_types.items():
            freq_str = self.run_command(f"vcgencmd measure_clock {ftype}")
            if freq_str and "frequency" in freq_str:
                match = re.search(r'frequency\((\d+)\)=(\d+)', freq_str)
                if match:
                    freq_hz = int(match.group(2))
                    freq_mhz = freq_hz / 1_000_000
                    if freq_mhz > 0:  # Nur aktive Frequenzen anzeigen
                        frequencies[description] = freq_mhz

        return frequencies

    def get_memory_info(self):
        """Liest RAM-Auslastung aus"""
        mem_total_str = self.run_command("cat /proc/meminfo | grep MemTotal")
        mem_avail_str = self.run_command("cat /proc/meminfo | grep MemAvailable")

        if mem_total_str and mem_avail_str:
            total = int(re.search(r'(\d+)', mem_total_str).group(1)) / 1024  # MB
            avail = int(re.search(r'(\d+)', mem_avail_str).group(1)) / 1024  # MB
            used = total - avail
            usage_pct = (used / total) * 100

            return {
                'total': total,
                'used': used,
                'available': avail,
                'percentage': usage_pct
            }
        return None

    def get_cpu_usage(self):
        """Berechnet CPU-Auslastung"""
        # Erste Messung
        stat1 = self.run_command("cat /proc/stat | grep '^cpu '")
        time.sleep(0.1)
        stat2 = self.run_command("cat /proc/stat | grep '^cpu '")

        if stat1 and stat2:
            vals1 = [int(x) for x in stat1.split()[1:]]
            vals2 = [int(x) for x in stat2.split()[1:]]

            idle1 = vals1[3]
            idle2 = vals2[3]

            total1 = sum(vals1)
            total2 = sum(vals2)

            diff_idle = idle2 - idle1
            diff_total = total2 - total1

            if diff_total > 0:
                usage = 100.0 * (diff_total - diff_idle) / diff_total
                return usage

        return None

    def get_uptime(self):
        """Liest System-Uptime aus"""
        uptime_str = self.read_file('/proc/uptime')
        if uptime_str:
            uptime_seconds = float(uptime_str.split()[0])
            days = int(uptime_seconds // 86400)
            hours = int((uptime_seconds % 86400) // 3600)
            minutes = int((uptime_seconds % 3600) // 60)
            return days, hours, minutes
        return None, None, None

    def get_board_info(self):
        """Liest Board-Revision und Seriennummer aus"""
        revision = self.run_command("vcgencmd otp_dump | grep 30: | cut -d: -f2")
        serial = self.run_command("cat /proc/cpuinfo | grep Serial | cut -d' ' -f2")
        model = self.read_file("/proc/device-tree/model")

        return {
            'model': model if model else "N/A",
            'revision': revision.strip() if revision else "N/A",
            'serial': serial.strip() if serial else "N/A"
        }

    def get_gpu_memory(self):
        """Liest GPU-Speicherzuteilung aus"""
        gpu_mem = self.run_command("vcgencmd get_mem gpu")
        arm_mem = self.run_command("vcgencmd get_mem arm")

        gpu_mb = None
        arm_mb = None

        if "gpu=" in gpu_mem:
            gpu_mb = gpu_mem.replace("gpu=", "").replace("M", "")
        if "arm=" in arm_mem:
            arm_mb = arm_mem.replace("arm=", "").replace("M", "")

        return gpu_mb, arm_mb

    def get_thermal_zones(self):
        """Liest alle verfügbaren Thermal Zones aus sysfs"""
        thermal_zones = {}
        thermal_path = "/sys/class/thermal"

        if os.path.exists(thermal_path):
            for zone in os.listdir(thermal_path):
                if zone.startswith("thermal_zone"):
                    temp_file = os.path.join(thermal_path, zone, "temp")
                    type_file = os.path.join(thermal_path, zone, "type")

                    temp_val = self.read_file(temp_file)
                    type_val = self.read_file(type_file)

                    if temp_val:
                        temp_c = float(temp_val) / 1000.0
                        zone_type = type_val if type_val else zone
                        thermal_zones[zone_type] = temp_c

        return thermal_zones

    def get_cooling_device_state(self):
        """Liest Fan/Cooling Device Status aus (falls aktiv)"""
        cooling_path = "/sys/class/thermal/cooling_device0"

        if os.path.exists(cooling_path):
            cur_state = self.read_file(os.path.join(cooling_path, "cur_state"))
            max_state = self.read_file(os.path.join(cooling_path, "max_state"))
            cooling_type = self.read_file(os.path.join(cooling_path, "type"))

            if cur_state and max_state:
                return {
                    'type': cooling_type,
                    'current': int(cur_state),
                    'max': int(max_state)
                }

        return None

    def get_voltages(self):
        """Liest Standard-Spannungen aus"""
        voltages = {}
        voltage_types = {
            "core": "VideoCore",
            "sdram_c": "RAM Core",
            "sdram_i": "RAM I/O",
            "sdram_p": "RAM Phy"
        }

        for vtype, description in voltage_types.items():
            volt_str = self.run_command(f"vcgencmd measure_volts {vtype}")
            if "volt=" in volt_str:
                volt = volt_str.replace("volt=", "").replace("V", "")
                voltages[description] = float(volt)

        return voltages

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
            voltage_name = current_name.replace("_A", "_V")
            if voltage_name in voltages:
                power = current_value * voltages[voltage_name]
                power_details[current_name.replace("_A", "")] = power
                total_power += power

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
        print(f"  RASPBERRY PI 5 SYSTEM-MONITOR (ERWEITERT)")
        print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*70)

        # Board-Informationen
        board_info = self.get_board_info()
        print(f"\n🖥️  SYSTEM-INFORMATION")
        print(f"  Modell:              {board_info['model']}")

        # Uptime
        days, hours, minutes = self.get_uptime()
        if days is not None:
            uptime_str = f"{days}d {hours}h {minutes}m"
            print(f"  Laufzeit:            {uptime_str}")

        # Temperatur und Thermal Zones
        temp = self.get_temperature()
        temp_str, temp_status = self.format_temperature_status(temp)
        print(f"\n🌡️  TEMPERATUR")
        print(f"  CPU (vcgencmd):      {temp_str:>12}  {temp_status}")

        # Zusätzliche Thermal Zones aus sysfs
        thermal_zones = self.get_thermal_zones()
        if thermal_zones:
            for zone_name, zone_temp in thermal_zones.items():
                zone_str, zone_status = self.format_temperature_status(zone_temp)
                print(f"  {zone_name:<18}  {zone_str:>12}  {zone_status}")

        # Cooling Device
        cooling = self.get_cooling_device_state()
        if cooling:
            print(f"\n❄️  KÜHLUNG")
            print(f"  Typ:                 {cooling['type']}")
            print(f"  Aktueller Zustand:   {cooling['current']}/{cooling['max']}")

            # PWM-Tabelle
            pwm_levels = {0: "Aus (<50°C)", 1: "Niedrig (50-60°C)", 
                         2: "Mittel (60-67.5°C)", 3: "Hoch (67.5-75°C)", 
                         4: "Maximum (>75°C)"}
            if cooling['current'] in pwm_levels:
                print(f"  Status:              {pwm_levels[cooling['current']]}")

        # CPU & System-Auslastung
        print(f"\n⚙️  SYSTEM-AUSLASTUNG")
        cpu_usage = self.get_cpu_usage()
        if cpu_usage is not None:
            print(f"  CPU-Auslastung:      {cpu_usage:>6.1f} %")

        # Speicher-Auslastung
        mem_info = self.get_memory_info()
        if mem_info:
            print(f"  RAM-Auslastung:      {mem_info['percentage']:>6.1f} %")
            print(f"  RAM Verwendet:       {mem_info['used']:>7.0f} MB / {mem_info['total']:.0f} MB")
            print(f"  RAM Verfügbar:       {mem_info['available']:>7.0f} MB")

        # GPU-Speicher
        gpu_mem, arm_mem = self.get_gpu_memory()
        if gpu_mem and arm_mem:
            print(f"  GPU-Speicher:        {gpu_mem:>7} MB")
            print(f"  ARM-Speicher:        {arm_mem:>7} MB")

        # Taktfrequenzen
        print(f"\n⏱️  TAKTFREQUENZEN")
        frequencies = self.get_all_frequencies()
        for desc, freq in sorted(frequencies.items()):
            print(f"  {desc:<18}  {freq:>9.0f} MHz")

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
        print(f"\n🔋 SPANNUNGEN")
        voltages = self.get_voltages()
        for desc, volt in voltages.items():
            print(f"  {desc:<18}  {volt:>6.3f} V")

        # PMIC-Daten (nur wenn verfügbar)
        currents, pmic_voltages = self.get_pmic_data()

        if currents and pmic_voltages:
            print(f"\n🔌 PMIC-DETAILS (Top-5 Verbraucher)")

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

            sorted_rails = sorted(power_per_rail.items(), 
                                key=lambda x: x[1]['power'], 
                                reverse=True)[:5]

            print(f"  {'Zweig':<14} {'Spannung':>10} {'Strom':>10} {'Leistung':>10}")
            print(f"  {'-'*14} {'-'*10} {'-'*10} {'-'*10}")

            for rail, data in sorted_rails:
                print(f"  {rail:<14} {data['voltage']:>8.2f} V "
                      f"{data['current']:>8.4f} A {data['power']:>8.3f} W")

            # Leistungsaufnahme
            total_power, corrected_power, _ = self.calculate_power(currents, pmic_voltages)

            print(f"\n⚡ LEISTUNGSAUFNAHME")
            print(f"  PMIC-Summe:          {total_power:>6.2f} W")
            print(f"  Geschätzt (korr.):   {corrected_power:>6.2f} W")
            print(f"  Hinweis: Exklusive USB-Geräte und HATs")

        print("\n" + "="*70 + "\n")

def main():
    """Hauptfunktion"""
    import argparse

    parser = argparse.ArgumentParser(
        description="Raspberry Pi 5 System-Monitor (Erweitert)"
    )
    parser.add_argument(
        '-c', '--continuous',
        action='store_true',
        help='Kontinuierliche Überwachung (alle 2 Sekunden)'
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
                print("\033[2J\033[H", end="")
                monitor.print_system_info()
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\n\nÜberwachung beendet.")
    else:
        monitor.print_system_info()

if __name__ == "__main__":
    main()
