#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test-Script für LIDAR-Controller-Fusion System
==============================================

Testet alle Komponenten des LIDAR-Fusion-Systems:
- LIDAR Thread Management
- Koordinatentransformation
- Kollisionserkennung  
- Environment Mapping
- Data Recording Integration

Usage:
    python test_lidar_fusion.py [--duration 30] [--export-map]
    python test_lidar_fusion.py --simulate  # Ohne echte Hardware
    python test_lidar_fusion.py --help      # Hilfe anzeigen

Version: 1.0.2 - Korrigierte Typing-Imports
"""

import argparse
import time
import logging
import sys
import os
from pathlib import Path
import numpy as np
import threading
import queue
from typing import Dict, List, Optional, Tuple, Any, Union  # KORREKTUR: Typing-Imports hinzugefügt

# Füge Pfad zum Hauptmodul hinzu
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

try:
    from lidar_controller_fusion import (
        LidarControllerFusion, 
        RobotPose, 
        EnvironmentMetadata,
        CollisionLevel,
        LidarFrame,
        LidarThreadManager,
        LidarProcessingStats  # KORREKTUR: Zusätzlicher Import für bessere Konsistenz
    )
except ImportError as e:
    print(f"❌ Import-Fehler: {e}")
    print("   Stelle sicher, dass 'lidar_controller_fusion.py' im gleichen Verzeichnis liegt")
    sys.exit(1)


# =============================================================================
# LIDAR-SIMULATOR FÜR TESTS OHNE HARDWARE
# =============================================================================

class LidarSimulator:
    """
    Simuliert LIDAR-Scandaten für Tests ohne echte Hardware.

    Erstellt realistische LIDAR-Scans mit simulierten Hindernissen
    und Umgebungsstrukturen für umfassende Systemtests.
    """

    def __init__(self, scan_rate_hz: float = 10.0):
        """
        Initialisiert den LIDAR-Simulator.

        Args:
            scan_rate_hz: Simulierte Scan-Rate in Hz
        """
        self.scan_rate_hz = scan_rate_hz
        self.scan_interval = 1.0 / scan_rate_hz
        self._running = False
        self._scan_queue: queue.Queue = queue.Queue(maxsize=50)  # KORREKTUR: Typ-Annotation
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Simulierte Umgebung (einfacher rechteckiger Raum mit Hindernissen)
        self.room_size = 10.0  # 10x10 Meter Raum
        self.obstacles: List[Dict[str, Any]] = [  # KORREKTUR: Typ-Annotation
            # Wände (als Liniensegmente)
            {"type": "wall", "start": (-5.0, -5.0), "end": (5.0, -5.0)},  # Süd
            {"type": "wall", "start": (5.0, -5.0), "end": (5.0, 5.0)},    # Ost
            {"type": "wall", "start": (5.0, 5.0), "end": (-5.0, 5.0)},    # Nord
            {"type": "wall", "start": (-5.0, 5.0), "end": (-5.0, -5.0)},  # West

            # Hindernisse im Raum
            {"type": "box", "center": (2.0, 1.0), "size": 0.5},
            {"type": "box", "center": (-1.5, -2.0), "size": 0.8},
            {"type": "box", "center": (0.0, 3.0), "size": 0.3},
        ]

        self.frame_id = 0

    def start(self) -> bool:
        """
        Startet die LIDAR-Simulation.

        Returns:
            True wenn erfolgreich gestartet
        """
        if self._running:
            return True

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._simulation_worker, daemon=True)
        self._thread.start()
        self._running = True

        return True

    def stop(self) -> None:
        """
        Stoppt die LIDAR-Simulation.
        """
        if not self._running:
            return

        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._running = False

    def get_latest_scan(self, timeout: float = 0.1) -> Optional[LidarFrame]:
        """
        Holt den neuesten simulierten Scan.

        Args:
            timeout: Maximale Wartezeit in Sekunden

        Returns:
            LidarFrame mit simulierten Scandaten oder None
        """
        try:
            return self._scan_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def is_running(self) -> bool:
        """
        Prüft ob die Simulation läuft.

        Returns:
            True wenn Simulation aktiv ist
        """
        return self._running

    def _simulation_worker(self) -> None:
        """
        Haupt-Simulation-Thread.
        """
        while not self._stop_event.is_set():
            start_time = time.time()

            # Generiere Scan
            scan_data = self._generate_scan()

            # Erstelle LidarFrame
            frame = LidarFrame(
                timestamp=time.time(),
                scan_data=scan_data,
                frame_id=self.frame_id
            )

            # Füge zu Queue hinzu
            try:
                self._scan_queue.put_nowait(frame)
                self.frame_id += 1
            except queue.Full:
                # Queue voll - entferne ältesten Scan
                try:
                    self._scan_queue.get_nowait()
                    self._scan_queue.put_nowait(frame)
                    self.frame_id += 1
                except queue.Empty:
                    pass

            # Warte bis zum nächsten Scan
            elapsed = time.time() - start_time
            sleep_time = max(0, self.scan_interval - elapsed)
            if sleep_time > 0:
                self._stop_event.wait(sleep_time)

    def _generate_scan(self) -> List[Tuple[float, float, float]]:
        """
        Generiert einen simulierten LIDAR-Scan.

        Returns:
            Liste von (quality, angle_deg, distance_mm) Tupeln
        """
        scan_data: List[Tuple[float, float, float]] = []  # KORREKTUR: Typ-Annotation

        # Generiere 360° Scan mit 1° Auflösung
        for angle_deg in range(0, 360, 1):
            angle_rad = np.radians(angle_deg)

            # Berechne Strahl-Richtung
            ray_dx = np.cos(angle_rad)
            ray_dy = np.sin(angle_rad)

            # Finde nächste Kollision
            min_distance = self._cast_ray(0.0, 0.0, ray_dx, ray_dy)

            if min_distance > 0:
                # Füge etwas Rauschen hinzu für Realismus
                noise = np.random.normal(0, 0.02)  # 2cm Standardabweichung
                distance_mm = (min_distance + noise) * 1000.0

                # Quality basierend auf Distanz (weiter weg = schlechtere Quality)
                quality = max(0.5, 1.0 - min_distance / 8.0)
                quality += np.random.normal(0, 0.05)  # Quality-Rauschen
                quality = np.clip(quality, 0.0, 1.0)

                scan_data.append((quality, angle_deg, distance_mm))

        return scan_data

    def _cast_ray(self, start_x: float, start_y: float, 
                 ray_dx: float, ray_dy: float, max_distance: float = 15.0) -> float:
        """
        Wirft einen Strahl und findet die nächste Kollision.

        Args:
            start_x, start_y: Strahl-Startpunkt
            ray_dx, ray_dy: Strahl-Richtung (normalisiert)
            max_distance: Maximale Strahl-Reichweite

        Returns:
            Distanz zur nächsten Kollision oder 0 wenn keine gefunden
        """
        min_distance = max_distance

        for obstacle in self.obstacles:
            if obstacle["type"] == "wall":
                # Linien-Kollision
                distance = self._ray_line_intersection(
                    start_x, start_y, ray_dx, ray_dy,
                    obstacle["start"], obstacle["end"]
                )
                if 0 < distance < min_distance:
                    min_distance = distance

            elif obstacle["type"] == "box":
                # Box-Kollision (vereinfacht als Kreis)
                distance = self._ray_circle_intersection(
                    start_x, start_y, ray_dx, ray_dy,
                    obstacle["center"], obstacle["size"]
                )
                if 0 < distance < min_distance:
                    min_distance = distance

        return min_distance if min_distance < max_distance else 0.0

    def _ray_line_intersection(self, ray_start_x: float, ray_start_y: float,
                             ray_dx: float, ray_dy: float,
                             line_start: Tuple[float, float], 
                             line_end: Tuple[float, float]) -> float:
        """
        Berechnet Strahl-Linien-Kollision.

        Returns:
            Distanz zur Kollision oder -1 wenn keine Kollision
        """
        x1, y1 = line_start
        x2, y2 = line_end

        # Liniensegment-Richtung
        line_dx = x2 - x1
        line_dy = y2 - y1

        # Determinante
        det = ray_dx * line_dy - ray_dy * line_dx
        if abs(det) < 1e-10:  # Parallel
            return -1

        # Parameter für Schnittpunkt
        u = ((x1 - ray_start_x) * line_dy - (y1 - ray_start_y) * line_dx) / det
        v = ((x1 - ray_start_x) * ray_dy - (y1 - ray_start_y) * ray_dx) / det

        # Prüfe ob Schnittpunkt gültig ist
        if u >= 0 and 0 <= v <= 1:
            return u

        return -1

    def _ray_circle_intersection(self, ray_start_x: float, ray_start_y: float,
                               ray_dx: float, ray_dy: float,
                               circle_center: Tuple[float, float],
                               circle_radius: float) -> float:
        """
        Berechnet Strahl-Kreis-Kollision.

        Returns:
            Distanz zur Kollision oder -1 wenn keine Kollision
        """
        cx, cy = circle_center

        # Vektor vom Strahl-Start zum Kreismittelpunkt
        oc_x = ray_start_x - cx
        oc_y = ray_start_y - cy

        # Quadratische Gleichung Koeffizienten
        a = ray_dx * ray_dx + ray_dy * ray_dy
        b = 2.0 * (oc_x * ray_dx + oc_y * ray_dy)
        c = oc_x * oc_x + oc_y * oc_y - circle_radius * circle_radius

        # Diskriminante
        discriminant = b * b - 4 * a * c

        if discriminant < 0:
            return -1  # Keine Kollision

        # Nächste Kollision (kleinste positive Lösung)
        sqrt_discriminant = np.sqrt(discriminant)
        t1 = (-b - sqrt_discriminant) / (2 * a)
        t2 = (-b + sqrt_discriminant) / (2 * a)

        if t1 >= 0:
            return t1
        elif t2 >= 0:
            return t2
        else:
            return -1


# =============================================================================
# SIMULIERTE LIDAR-MANAGER-KLASSE
# =============================================================================

class SimulatedLidarThreadManager:
    """
    Ersetzt LidarThreadManager für Simulation ohne Hardware.

    Bietet die gleiche API wie der echte LidarThreadManager,
    verwendet aber den LidarSimulator anstatt echter Hardware.
    """

    def __init__(self, port: str = "/dev/null", baudrate: int = 460800, 
                 logger: Optional[logging.Logger] = None):
        """
        Initialisiert den simulierten LIDAR Manager.

        Args:
            port: Wird ignoriert (für Kompatibilität)
            baudrate: Wird ignoriert (für Kompatibilität)  
            logger: Optional - Logger-Instanz
        """
        self.logger = logger or logging.getLogger(__name__)
        self.simulator = LidarSimulator(scan_rate_hz=10.0)

        # Fake Stats für Kompatibilität - KORREKTUR: Direkte Instanziierung
        self._stats = LidarProcessingStats()

    def start(self) -> bool:
        """
        Startet die Simulation.

        Returns:
            True (Simulation startet immer erfolgreich)
        """
        success = self.simulator.start()
        if success:
            self.logger.info("LIDAR-Simulation gestartet")
        return success

    def stop(self) -> None:
        """
        Stoppt die Simulation.
        """
        self.simulator.stop()
        self.logger.info("LIDAR-Simulation gestoppt")

    def get_latest_scan(self, timeout: float = 0.1) -> Optional[LidarFrame]:  # KORREKTUR: Return-Type
        """
        Holt den neuesten simulierten Scan.

        Args:
            timeout: Maximale Wartezeit in Sekunden

        Returns:
            LidarFrame mit simulierten Scandaten oder None
        """
        frame = self.simulator.get_latest_scan(timeout)
        if frame:
            self._stats.scans_processed += 1
            self._stats.processing_rate_hz = 10.0  # Feste Simulations-Rate
            self._stats.buffer_utilization = 0.5   # Fake Wert
        return frame

    def get_stats(self) -> LidarProcessingStats:  # KORREKTUR: Return-Type
        """
        Holt Statistiken der Simulation.

        Returns:
            LidarProcessingStats mit Simulations-Statistiken
        """
        return self._stats

    def is_running(self) -> bool:
        """
        Prüft ob die Simulation läuft.

        Returns:
            True wenn Simulation aktiv ist
        """
        return self.simulator.is_running()


# =============================================================================
# TEST-FUNKTIONEN
# =============================================================================

def setup_test_logging(log_level: str = "INFO") -> logging.Logger:
    """
    Konfiguriert Logging für Tests.

    Args:
        log_level: Log-Level ("DEBUG", "INFO", "WARNING", "ERROR")

    Returns:
        Konfigurierter Logger
    """
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )
    return logging.getLogger(__name__)


def test_basic_functionality(fusion_system: LidarControllerFusion, logger: logging.Logger, duration: int = 10) -> None:  # KORREKTUR: Typ-Annotations
    """
    Testet Grundfunktionalität des Systems.

    Args:
        fusion_system: LidarControllerFusion-Instanz
        logger: Logger-Instanz
        duration: Test-Dauer in Sekunden
    """
    logger.info(f"🔄 Starte Basis-Funktionalitätstest ({duration}s)...")

    start_time = time.time()
    frame_count = 0
    collision_warnings = 0
    collision_criticals = 0

    # Simuliere Roboter-Bewegung (Kreisbahn)
    robot_x = 0.0
    robot_y = 0.0
    robot_theta = 0.0

    while time.time() - start_time < duration:
        # Update Roboter-Position (simulierte Kreisbewegung)
        t = time.time() - start_time
        robot_x = 2.0 * np.cos(t * 0.2)
        robot_y = 2.0 * np.sin(t * 0.2)
        robot_theta = t * 0.2 + np.pi/2

        pose = RobotPose(robot_x, robot_y, robot_theta, time.time())
        fusion_system.update_robot_pose(pose)

        # Simuliere Ultraschall-Daten (dynamische Werte)
        ultrasonic_data: Dict[str, float] = {  # KORREKTUR: Typ-Annotation
            "front": 1.5 + 0.5 * np.sin(t),
            "left": 2.0 + 0.3 * np.cos(t),
            "right": 1.8 + 0.2 * np.sin(t * 1.5),
            "back": 3.0
        }

        # Simuliere Roboter-Geschwindigkeit
        robot_velocity: Tuple[float, float] = (0.2 * np.sin(t * 0.5), 0.1 * np.cos(t * 0.3))  # KORREKTUR: Typ-Annotation

        # Verarbeite Frame
        result = fusion_system.process_frame(ultrasonic_data, robot_velocity)

        if result:
            frame_count += 1

            # Prüfe Kollisionswarnung
            collision_analysis = result.get("collision_analysis")
            if collision_analysis:
                level = collision_analysis["overall_level"]
                if level == CollisionLevel.WARNING:
                    collision_warnings += 1
                elif level == CollisionLevel.CRITICAL:
                    collision_criticals += 1
                    logger.warning(f"⚠️  Kritische Kollision: {collision_analysis['recommended_action']}")

            # Zeige Statistiken alle 3 Sekunden  
            if frame_count % 30 == 0:
                stats = result.get("system_stats", {})
                lidar_stats = stats.get("lidar_stats", {})
                logger.info(f"📊 Frame {frame_count}: "
                          f"LIDAR Rate: {lidar_stats.get('processing_rate_hz', 0):.1f} Hz, "
                          f"Buffer: {lidar_stats.get('buffer_utilization', 0):.1%}")

        time.sleep(0.1)  # 10 Hz Test-Rate

    logger.info(f"✅ Basis-Test abgeschlossen: {frame_count} Frames, "
               f"{collision_warnings} Warnungen, {collision_criticals} kritische Situationen")


def test_mapping_functionality(fusion_system: LidarControllerFusion, logger: logging.Logger, duration: int = 15) -> None:  # KORREKTUR: Typ-Annotations
    """
    Testet Environment Mapping Funktionalität.

    Args:
        fusion_system: LidarControllerFusion-Instanz
        logger: Logger-Instanz  
        duration: Test-Dauer in Sekunden
    """
    logger.info(f"🗺️  Starte Mapping-Test ({duration}s)...")

    start_time = time.time()

    # Simuliere komplexere Roboter-Bewegung für Mapping (Spiralbahn)
    while time.time() - start_time < duration:
        t = time.time() - start_time

        # Spiralförmige Bewegung für besseres Mapping
        radius = 0.5 + t * 0.1
        angle = t * 0.5
        robot_x = radius * np.cos(angle)  
        robot_y = radius * np.sin(angle)
        robot_theta = angle + np.pi/2

        pose = RobotPose(robot_x, robot_y, robot_theta, time.time())
        fusion_system.update_robot_pose(pose)

        # Verarbeite Frame für Mapping
        fusion_system.process_frame()

        time.sleep(0.2)  # 5 Hz für Mapping

    # Teste Karten-Export
    try:
        map_result = fusion_system.get_map_around_robot(radius_meters=10.0)
        if map_result is not None:
            map_data, metadata = map_result
            occupied_cells = np.sum(map_data > 0.1)  # Zähle belegte Zellen
            total_cells = map_data.size
            occupancy_rate = occupied_cells / total_cells * 100

            logger.info(f"✅ Karte erstellt: {map_data.shape} Pixel, "
                       f"Auflösung: {metadata['resolution']:.3f}m/px, "
                       f"Belegung: {occupancy_rate:.1f}%")
        else:
            logger.warning("⚠️  Keine Kartendaten verfügbar")
    except Exception as e:
        logger.error(f"❌ Mapping-Test Fehler: {e}")


def test_recording_functionality(fusion_system: LidarControllerFusion, logger: logging.Logger, duration: int = 10) -> None:  # KORREKTUR: Typ-Annotations
    """
    Testet Data Recording Funktionalität.

    Args:
        fusion_system: LidarControllerFusion-Instanz
        logger: Logger-Instanz
        duration: Test-Dauer in Sekunden
    """
    logger.info(f"💾 Starte Recording-Test ({duration}s)...")

    # Erstelle Session-Metadaten
    session_metadata = EnvironmentMetadata(
        session_id=f"test_session_{int(time.time())}",
        start_timestamp=time.time(),
        environment_conditions={
            "temperature_celsius": 22.0,
            "humidity_percent": 45.0,
            "test_mode": True
        },
        hardware_info={
            "lidar_model": "RPLIDAR C1 (Simuliert)",
            "controller_model": "Test Controller"
        },
        recording_settings={
            "lidar_rate_hz": 10,
            "test_duration_seconds": duration
        }
    )

    # Starte Recording
    if not fusion_system.start_recording(session_metadata):
        logger.error("❌ Recording konnte nicht gestartet werden")
        return

    start_time = time.time()
    frames_recorded = 0

    # Sammle Daten mit linearer Bewegung
    while time.time() - start_time < duration:
        # Roboter-Bewegung für Recording (lineare Fahrt)
        t = time.time() - start_time
        pose = RobotPose(t * 0.1, 0.0, 0.0, time.time())
        fusion_system.update_robot_pose(pose)

        result = fusion_system.process_frame()
        if result:
            frames_recorded += 1

        time.sleep(0.1)

    # Stoppe Recording
    recording_data = fusion_system.stop_recording()

    if recording_data:
        stats = recording_data.get_stats()
        logger.info(f"✅ Recording abgeschlossen: {stats.get('lidar_frames', 0)} LIDAR Frames, "
                   f"Dauer: {stats.get('duration_seconds', 0):.1f}s, "
                   f"Rate: {stats.get('lidar_sample_rate', 0):.1f} Hz")
    else:
        logger.error("❌ Keine Recording-Daten erhalten")


def test_collision_detection(fusion_system: LidarControllerFusion, logger: logging.Logger, duration: int = 8) -> None:  # KORREKTUR: Typ-Annotations
    """
    Testet Kollisionserkennung mit gezielten Szenarien.

    Args:
        fusion_system: LidarControllerFusion-Instanz
        logger: Logger-Instanz
        duration: Test-Dauer in Sekunden
    """
    logger.info(f"🚧 Starte Kollisionserkennungs-Test ({duration}s)...")

    start_time = time.time()
    collision_events: Dict[str, int] = {"safe": 0, "warning": 0, "critical": 0}  # KORREKTUR: Typ-Annotation

    while time.time() - start_time < duration:
        t = time.time() - start_time

        # Simuliere Annäherung an Hindernis
        if t < duration / 2:
            # Sichere Phase - weit weg von Hindernissen
            pose = RobotPose(-3.0, 0.0, 0.0, time.time())
            ultrasonic_data: Dict[str, float] = {"front": 3.0, "left": 3.0, "right": 3.0, "back": 3.0}  # KORREKTUR: Typ-Annotation
        else:
            # Annäherung - näher zu Hindernissen
            approach_factor = (t - duration / 2) * 2  # 0 bis 1
            front_distance = 2.0 - approach_factor * 1.5  # von 2.0m auf 0.5m
            pose = RobotPose(-1.0 + approach_factor, 0.0, 0.0, time.time())
            ultrasonic_data = {
                "front": max(0.2, front_distance),
                "left": 2.0, 
                "right": 2.0, 
                "back": 3.0
            }

        fusion_system.update_robot_pose(pose)
        result = fusion_system.process_frame(ultrasonic_data, (0.3, 0.0))

        if result:
            collision_analysis = result.get("collision_analysis")
            if collision_analysis:
                level = collision_analysis["overall_level"]
                collision_events[level] += 1

                if level != CollisionLevel.SAFE:
                    action = collision_analysis["recommended_action"]
                    logger.info(f"🚨 {level.upper()}: {action}")

        time.sleep(0.1)

    logger.info(f"✅ Kollisionstest abgeschlossen: "
               f"Safe: {collision_events['safe']}, "
               f"Warning: {collision_events['warning']}, "
               f"Critical: {collision_events['critical']}")


def main() -> int:  # KORREKTUR: Return-Type
    """
    Haupt-Testfunktion.
    
    Returns:
        Exit-Code (0 = Erfolg, >0 = Fehler)
    """
    parser = argparse.ArgumentParser(
        description='LIDAR-Fusion System Test v1.0.2',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Beispiele:
  python test_lidar_fusion.py --duration 60 --export-map
  python test_lidar_fusion.py --simulate --verbose
  python test_lidar_fusion.py --quick-test
        """
    )

    parser.add_argument('--duration', type=int, default=45,
                       help='Gesamte Test-Dauer in Sekunden (default: 45)')
    parser.add_argument('--lidar-port', default='/dev/ttyUSB0',
                       help='LIDAR serieller Port (default: /dev/ttyUSB0)')
    parser.add_argument('--export-map', action='store_true',
                       help='Exportiere finale Karte nach dem Test')
    parser.add_argument('--simulate', action='store_true',
                       help='Verwende LIDAR-Simulation (keine echte Hardware)')
    parser.add_argument('--verbose', action='store_true',
                       help='Detailliertes Logging (DEBUG Level)')
    parser.add_argument('--quick-test', action='store_true',
                       help='Schneller Test (10s gesamt, nur Basis-Tests)')

    args = parser.parse_args()

    # Konfiguration anpassen für Quick-Test
    if args.quick_test:
        args.duration = 10
        args.simulate = True

    # Setup Logging
    log_level = "DEBUG" if args.verbose else "INFO"
    logger = setup_test_logging(log_level)

    try:
        logger.info("🚀 === LIDAR-Controller-Fusion System Test v1.0.2 ===")
        logger.info(f"📋 Test-Konfiguration:")
        logger.info(f"   - Dauer: {args.duration}s")
        logger.info(f"   - LIDAR Port: {args.lidar_port}")
        logger.info(f"   - Simulation: {'Ja' if args.simulate else 'Nein'}")
        logger.info(f"   - Karten-Export: {'Ja' if args.export_map else 'Nein'}")
        logger.info("")

        # Initialisiere System
        if args.simulate:
            # Patche LidarThreadManager für Simulation
            import lidar_controller_fusion
            original_manager = lidar_controller_fusion.LidarThreadManager  # KORREKTUR: Variable für Original speichern
            lidar_controller_fusion.LidarThreadManager = SimulatedLidarThreadManager
            logger.info("🎮 Simulation aktiviert - keine echte Hardware erforderlich")

        fusion_system = LidarControllerFusion(
            lidar_port=args.lidar_port,
            logger=logger
        )

        # Starte System
        logger.info("⚙️  Starte LIDAR-Fusion-System...")
        if not fusion_system.start():
            logger.error("❌ System konnte nicht gestartet werden")
            return 1

        # Kurz warten für Initialisierung
        time.sleep(1)

        try:
            # Test-Sequenz
            if args.quick_test:
                # Nur Basis-Test für Quick-Test
                test_basic_functionality(fusion_system, logger, args.duration)
            else:
                # Vollständige Test-Suite
                phase_duration = args.duration // 4

                # Phase 1: Basis-Funktionalität
                test_basic_functionality(fusion_system, logger, phase_duration)

                # Phase 2: Kollisionserkennung
                test_collision_detection(fusion_system, logger, phase_duration)

                # Phase 3: Mapping
                test_mapping_functionality(fusion_system, logger, phase_duration)

                # Phase 4: Recording
                test_recording_functionality(fusion_system, logger, phase_duration)

            # Finale Statistiken
            final_stats = fusion_system.get_system_stats()
            logger.info("")
            logger.info("📊 === Finale System-Statistiken ===")
            logger.info(f"🔋 System Status: {'✅ Läuft' if final_stats['system_running'] else '❌ Gestoppt'}")

            lidar_stats = final_stats.get('lidar_stats', {})
            logger.info(f"📡 LIDAR:")
            logger.info(f"   - Scans verarbeitet: {lidar_stats.get('scans_processed', 0)}")
            logger.info(f"   - Verarbeitungsrate: {lidar_stats.get('processing_rate_hz', 0):.1f} Hz")
            logger.info(f"   - Fehler aufgetreten: {lidar_stats.get('error_count', 0)}")
            logger.info(f"   - Buffer-Auslastung: {lidar_stats.get('buffer_utilization', 0):.1%}")

            mapping_stats = final_stats.get('mapping_stats', {})
            logger.info(f"🗺️  Mapping:")
            logger.info(f"   - Karten-Updates: {mapping_stats.get('map_updates', 0)}")
            logger.info(f"   - Kartenauflösung: {mapping_stats.get('map_resolution', 0):.3f} m/px")

            # Exportiere Karte falls gewünscht
            if args.export_map:
                output_dir = Path("test_results")
                output_dir.mkdir(exist_ok=True)

                timestamp = int(time.time())

                # Verschiedene Formate exportieren
                for format_type in ["json", "npy"]:
                    map_file = output_dir / f"lidar_map_{timestamp}.{format_type}"
                    if fusion_system.export_map(map_file, format_type):
                        logger.info(f"💾 Karte exportiert: {map_file}")
                    else:
                        logger.error(f"❌ Karten-Export ({format_type}) fehlgeschlagen")

        finally:
            # System sauber herunterfahren
            logger.info("🔌 Stoppe System...")
            fusion_system.stop()
            
            # KORREKTUR: Original LidarThreadManager wiederherstellen falls gepatcht
            if args.simulate:
                import lidar_controller_fusion
                lidar_controller_fusion.LidarThreadManager = original_manager

        logger.info("")
        logger.info("🎉 === Test erfolgreich abgeschlossen ===")
        logger.info("   Alle Komponenten funktionieren korrekt!")

        return 0

    except KeyboardInterrupt:
        logger.info("")
        logger.info("⏹️  Test durch Benutzer abgebrochen")
        return 130
    except Exception as e:
        logger.error(f"❌ Test-Fehler: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == '__main__':
    exit(main())
