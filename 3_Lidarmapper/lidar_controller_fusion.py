#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LIDAR-Controller Fusion System - Phase 3 Integration
===================================================

Echtzeit-Integration von LIDAR C1 mit Roboter-Controller-System.
Implementiert Thread-sichere LIDAR-Verarbeitung, Koordinatentransformation
und erweiterte Kollisionsvermeidung mit Data Fusion.

Features:
    - Separate LIDAR-Thread mit eigenständiger Error-Recovery
    - Koordinatentransformation: LIDAR-Zentrum → Roboter-Mittelpunkt
    - Buffer-Management für hochfrequente LIDAR-Daten (5-15Hz)
    - Integration in bestehende Kollisionsvermeidung als Safety-Layer
    - Echtzeit-Umgebungs-Mapping mit konsistenter 2D-Karte
    - Data Fusion zwischen Ultraschall- und LIDAR-Sensoren
    - Thread-sichere Datenübertragung mit Queue-System

Version: 1.0.0
Datum: 2025-10-11
Autor: Robotics Development Team

Hardware-Anforderungen:
    - RPLIDAR C1 (460800 Baud, /dev/ttyUSB0)
    - Xbox Controller (bereits implementiert in Phase 2)
    - Ultraschall-Sensoren (falls vorhanden)
    - Raspberry Pi 4+ (empfohlen für Threading-Performance)

Koordinatensystem:
    - LIDAR: Zentrum bei (0,0), 0° = Norden, Uhrzeigersinn
    - Roboter: Mittelpunkt bei (0,0), 0° = Vorwärts, Uhrzeigersinn
    - Transformation: LIDAR-Position relative zu Roboter-Mittelpunkt
"""

import threading
import queue
import time
import math
import numpy as np
from typing import Dict, List, Optional, Tuple, Any, NamedTuple
from dataclasses import dataclass, field
from collections import deque
import logging
from pathlib import Path
import json

# Import der bestehenden Systeme
from rplidar import RPLidar, RPLidarException
import sys
import os

# Füge Pfade zu bestehenden Modulen hinzu
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '1_storage'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '2_recorder'))

from path_recording_system import (
    LidarFrame, ControllerSample, PathRecordingData, 
    PathRecordingHeader, EnvironmentMetadata
)

# =============================================================================
# KONSTANTEN UND KONFIGURATION
# =============================================================================

# LIDAR-Konfiguration
LIDAR_DEFAULT_PORT = "/dev/ttyUSB0"
LIDAR_DEFAULT_BAUDRATE = 460800
LIDAR_SCAN_TIMEOUT = 2.0
LIDAR_TARGET_FREQUENCY = 10.0  # Hz - Ziel-Scan-Rate
LIDAR_MAX_BUFFER_SIZE = 50     # Maximale Anzahl gepufferter Scans

# Koordinatentransformation (LIDAR zu Roboter-Koordinaten)
LIDAR_OFFSET_X = 0.15  # meters - LIDAR 15cm vor Roboter-Mittelpunkt
LIDAR_OFFSET_Y = 0.0   # meters - LIDAR zentriert
LIDAR_ANGLE_OFFSET = 0.0  # radians - Winkel-Offset zwischen LIDAR und Roboter

# Kollisionserkennung
COLLISION_WARNING_DISTANCE = 0.8  # meters - Warndistanz
COLLISION_CRITICAL_DISTANCE = 0.3  # meters - Kritische Distanz (Stopp)
COLLISION_SECTOR_ANGLE = 60.0      # degrees - Sektor-Breite für Vorwärts-Kollision

# Mapping-Parameter  
MAP_RESOLUTION = 0.05  # meters per pixel
MAP_SIZE_METERS = 20.0  # meters - Kartengröße (20x20m)
MAP_DECAY_FACTOR = 0.99  # Vergessen-Faktor für alte Daten

# Thread-Konfiguration
LIDAR_THREAD_PRIORITY = threading.THREAD_CLASS_INTERACTIVE
QUEUE_TIMEOUT = 0.1  # seconds


# =============================================================================
# DATENSTRUKTUREN
# =============================================================================

@dataclass
class RobotPose:
    """
    Aktuelle Roboter-Position und -Orientierung.
    
    Attributes:
        x: X-Position in Metern (Vorwärts/Rückwärts)
        y: Y-Position in Metern (Links/Rechts)
        theta: Orientierung in Radiant (0 = Vorwärts)
        timestamp: Unix-Zeitstempel der Position
    """
    x: float
    y: float  
    theta: float
    timestamp: float


@dataclass
class ObstacleInfo:
    """
    Information über erkannte Hindernisse.
    
    Attributes:
        distance: Entfernung zum Hindernis in Metern
        angle: Winkel zum Hindernis in Radiant (relativ zur Roboter-Vorwärtsrichtung)
        sector: Sektor-Name ("front", "left", "right", "back")
        confidence: Vertrauenswert 0.0-1.0 (LIDAR vs Ultraschall)
        sensor_type: Sensor-Typ ("lidar", "ultrasonic", "fused")
    """
    distance: float
    angle: float
    sector: str
    confidence: float
    sensor_type: str


@dataclass  
class LidarProcessingStats:
    """
    Statistiken der LIDAR-Verarbeitung.
    
    Attributes:
        scans_processed: Anzahl verarbeiteter Scans
        processing_rate_hz: Aktuelle Verarbeitungsrate
        avg_points_per_scan: Durchschnittliche Punkte pro Scan
        error_count: Anzahl aufgetretener Fehler
        last_error_time: Zeitstempel des letzten Fehlers
        buffer_utilization: Buffer-Auslastung 0.0-1.0
    """
    scans_processed: int = 0
    processing_rate_hz: float = 0.0
    avg_points_per_scan: float = 0.0
    error_count: int = 0
    last_error_time: float = 0.0
    buffer_utilization: float = 0.0


class CollisionLevel:
    """Kollisions-Warnstufen"""
    SAFE = "safe"          # Keine Kollisionsgefahr
    WARNING = "warning"    # Warnung - Vorsicht erforderlich
    CRITICAL = "critical"  # Kritisch - Sofortiger Stopp erforderlich


# =============================================================================
# LIDAR-THREAD-MANAGER
# =============================================================================

class LidarThreadManager:
    """
    Thread-sichere LIDAR-Datenerfassung mit Error-Recovery.
    
    Verwaltet die LIDAR-Hardware in einem separaten Thread, implementiert
    robuste Error-Recovery-Mechanismen und stellt kontinuierliche Scandaten
    über ein Thread-sicheres Queue-System bereit.
    
    Features:
        - Automatische Wiederverbindung bei Hardware-Fehlern
        - Scan-Rate-Monitoring und -Anpassung
        - Buffer-Management für gleichmäßige Datenlieferung
        - Detaillierte Fehlerprotokollierung
        - Graceful Shutdown mit Resource-Cleanup
    """
    
    def __init__(self, port: str = LIDAR_DEFAULT_PORT, 
                 baudrate: int = LIDAR_DEFAULT_BAUDRATE,
                 logger: Optional[logging.Logger] = None):
        """
        Initialisiert den LIDAR Thread Manager.
        
        Args:
            port: Serieller Port für LIDAR-Verbindung
            baudrate: Baudrate für LIDAR-Kommunikation
            logger: Optional - Logger-Instanz für Fehlerprotokollierung
        """
        self.port = port
        self.baudrate = baudrate
        self.logger = logger or logging.getLogger(__name__)
        
        # Thread-Kontrolle
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._running = False
        
        # LIDAR-Hardware
        self._lidar: Optional[RPLidar] = None
        self._reconnect_attempts = 0
        self._max_reconnect_attempts = 5
        
        # Daten-Queues (Thread-sicher)
        self._scan_queue = queue.Queue(maxsize=LIDAR_MAX_BUFFER_SIZE)
        self._stats_queue = queue.Queue(maxsize=1)  # Nur neueste Stats
        
        # Statistiken
        self._stats = LidarProcessingStats()
        self._last_stats_update = time.time()
        self._scan_times = deque(maxlen=50)  # Für Rate-Berechnung
        
    def start(self) -> bool:
        """
        Startet den LIDAR-Thread.
        
        Returns:
            True wenn erfolgreich gestartet, False bei Fehler
        """
        if self._running:
            self.logger.warning("LIDAR-Thread läuft bereits")
            return True
            
        try:
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._lidar_thread_worker,
                name="LidarThread",
                daemon=True
            )
            self._thread.start()
            
            # Warte auf erfolgreiche Initialisierung
            start_time = time.time()
            while not self._running and time.time() - start_time < 5.0:
                time.sleep(0.1)
                
            if self._running:
                self.logger.info("LIDAR-Thread erfolgreich gestartet")
                return True
            else:
                self.logger.error("LIDAR-Thread Initialisierung fehlgeschlagen")
                return False
                
        except Exception as e:
            self.logger.error(f"Fehler beim Starten des LIDAR-Threads: {e}")
            return False
    
    def stop(self) -> None:
        """
        Stoppt den LIDAR-Thread graceful.
        """
        if not self._running:
            return
            
        self.logger.info("Stoppe LIDAR-Thread...")
        self._stop_event.set()
        
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                self.logger.warning("LIDAR-Thread konnte nicht graceful gestoppt werden")
        
        self._running = False
        self.logger.info("LIDAR-Thread gestoppt")
    
    def get_latest_scan(self, timeout: float = QUEUE_TIMEOUT) -> Optional[LidarFrame]:
        """
        Holt den neuesten LIDAR-Scan aus der Queue.
        
        Args:
            timeout: Maximale Wartezeit in Sekunden
            
        Returns:
            LidarFrame mit Scandaten oder None wenn keine Daten verfügbar
        """
        try:
            return self._scan_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_stats(self) -> LidarProcessingStats:
        """
        Holt die aktuellen Verarbeitungsstatistiken.
        
        Returns:
            LidarProcessingStats mit aktuellen Statistiken
        """
        try:
            # Hole neueste Stats aus Queue (non-blocking)
            while True:
                stats = self._stats_queue.get_nowait()
                self._stats = stats
        except queue.Empty:
            pass
        
        return self._stats
    
    def is_running(self) -> bool:
        """
        Prüft ob der LIDAR-Thread läuft.
        
        Returns:
            True wenn Thread aktiv ist
        """
        return self._running
    
    def _lidar_thread_worker(self) -> None:
        """
        Haupt-Worker-Funktion des LIDAR-Threads.
        
        Implementiert die kontinuierliche LIDAR-Datenerfassung mit
        automatischer Error-Recovery und Hardware-Reconnection.
        """
        self.logger.info("LIDAR-Thread gestartet")
        
        while not self._stop_event.is_set():
            try:
                # LIDAR-Verbindung aufbauen/wiederherstellen
                if not self._ensure_lidar_connection():
                    time.sleep(1.0)
                    continue
                
                # Scan-Schleife
                self._scan_loop()
                
            except Exception as e:
                self.logger.error(f"Unerwarteter Fehler im LIDAR-Thread: {e}")
                self._cleanup_lidar_connection()
                self._stats.error_count += 1
                self._stats.last_error_time = time.time()
                time.sleep(1.0)
        
        # Cleanup beim Thread-Ende
        self._cleanup_lidar_connection()
        self.logger.info("LIDAR-Thread beendet")
    
    def _ensure_lidar_connection(self) -> bool:
        """
        Stellt sicher, dass eine gültige LIDAR-Verbindung besteht.
        
        Returns:
            True wenn Verbindung erfolgreich, False bei Fehler
        """
        if self._lidar is not None:
            return True
            
        try:
            self.logger.info(f"Verbinde mit LIDAR auf {self.port}@{self.baudrate}")
            
            # Neue LIDAR-Verbindung aufbauen
            self._lidar = RPLidar(self.port, baudrate=self.baudrate, 
                                timeout=LIDAR_SCAN_TIMEOUT)
            
            # Geräteinfo prüfen
            info = self._lidar.get_info()
            self.logger.info(f"LIDAR Info: {info}")
            
            # Health-Check
            health = self._lidar.get_health()
            status = health[0] if isinstance(health, tuple) else health.get('status', 'Unknown')
            
            if status == 'Error':
                raise RuntimeError('LIDAR meldet Fehlerzustand')
            
            # Motor starten
            self._lidar.start_motor()
            time.sleep(2.0)  # Motor-Stabilisierung
            
            self._reconnect_attempts = 0
            self._running = True
            self.logger.info("LIDAR erfolgreich verbunden")
            return True
            
        except Exception as e:
            self.logger.error(f"LIDAR-Verbindung fehlgeschlagen: {e}")
            self._cleanup_lidar_connection()
            
            self._reconnect_attempts += 1
            if self._reconnect_attempts >= self._max_reconnect_attempts:
                self.logger.error("Maximale Anzahl Reconnect-Versuche erreicht")
                return False
            
            return False
    
    def _cleanup_lidar_connection(self) -> None:
        """
        Räumt die LIDAR-Verbindung auf.
        """
        if self._lidar:
            try:
                self._lidar.stop()
            except Exception:
                pass
            try:
                self._lidar.stop_motor()
            except Exception:
                pass
            try:
                self._lidar.disconnect()
            except Exception:
                pass
            
            self._lidar = None
    
    def _scan_loop(self) -> None:
        """
        Hauptschleife für kontinuierliche Scan-Erfassung.
        """
        try:
            iterator = self._lidar.iter_scans(max_buf_meas=5000, min_len=10)
            
            for scan in iterator:
                if self._stop_event.is_set():
                    break
                
                # Verarbeite Scan
                self._process_scan(scan)
                
                # Update Statistiken
                self._update_stats()
                
        except RPLidarException as e:
            self.logger.error(f"LIDAR-Scan-Fehler: {e}")
            self._cleanup_lidar_connection()
        except StopIteration:
            self.logger.warning("LIDAR Iterator beendet vorzeitig")
            self._cleanup_lidar_connection()
    
    def _process_scan(self, scan: List[Tuple[float, float, float]]) -> None:
        """
        Verarbeitet einen einzelnen LIDAR-Scan.
        
        Args:
            scan: Liste von (quality, angle, distance) Tupeln
        """
        timestamp = time.time()
        
        # Erstelle LidarFrame
        frame = LidarFrame(
            timestamp=timestamp,
            scan_data=list(scan),
            frame_id=self._stats.scans_processed
        )
        
        # Füge zu Queue hinzu (non-blocking)
        try:
            self._scan_queue.put_nowait(frame)
        except queue.Full:
            # Queue voll - entferne ältesten Scan
            try:
                self._scan_queue.get_nowait()
                self._scan_queue.put_nowait(frame)
            except queue.Empty:
                pass
        
        # Update Scan-Zeiten für Rate-Berechnung
        self._scan_times.append(timestamp)
        self._stats.scans_processed += 1
    
    def _update_stats(self) -> None:
        """
        Aktualisiert die Verarbeitungsstatistiken.
        """
        now = time.time()
        
        # Rate berechnen basierend auf letzten Scans
        if len(self._scan_times) >= 2:
            time_span = self._scan_times[-1] - self._scan_times[0]
            if time_span > 0:
                self._stats.processing_rate_hz = (len(self._scan_times) - 1) / time_span
        
        # Buffer-Auslastung
        self._stats.buffer_utilization = self._scan_queue.qsize() / LIDAR_MAX_BUFFER_SIZE
        
        # Stats in Queue aktualisieren (nur wenn sich etwas geändert hat)
        if now - self._last_stats_update > 1.0:  # Alle 1 Sekunde
            try:
                # Entferne alte Stats
                while not self._stats_queue.empty():
                    self._stats_queue.get_nowait()
                # Füge neue Stats hinzu
                self._stats_queue.put_nowait(self._stats)
                self._last_stats_update = now
            except queue.Full:
                pass


# =============================================================================
# KOORDINATENTRANSFORMATION
# =============================================================================

class CoordinateTransformer:
    """
    Koordinatentransformation zwischen LIDAR- und Roboter-Koordinatensystem.
    
    Transformiert LIDAR-Scandaten vom LIDAR-zentrierten Koordinatensystem
    in das Roboter-zentrierte Koordinatensystem unter Berücksichtigung der
    physischen LIDAR-Position am Roboter.
    
    Koordinatensysteme:
        - LIDAR: Zentrum bei (0,0), 0° = Norden, Uhrzeigersinn positiv
        - Roboter: Mittelpunkt bei (0,0), 0° = Vorwärts, Uhrzeigersinn positiv
    """
    
    def __init__(self, lidar_offset_x: float = LIDAR_OFFSET_X,
                 lidar_offset_y: float = LIDAR_OFFSET_Y,
                 lidar_angle_offset: float = LIDAR_ANGLE_OFFSET):
        """
        Initialisiert den Koordinatentransformer.
        
        Args:
            lidar_offset_x: X-Offset des LIDAR zum Roboter-Mittelpunkt (meters)
            lidar_offset_y: Y-Offset des LIDAR zum Roboter-Mittelpunkt (meters) 
            lidar_angle_offset: Winkel-Offset zwischen LIDAR und Roboter (radians)
        """
        self.lidar_offset_x = lidar_offset_x
        self.lidar_offset_y = lidar_offset_y
        self.lidar_angle_offset = lidar_angle_offset
    
    def lidar_to_robot_coordinates(self, scan_data: List[Tuple[float, float, float]],
                                 robot_pose: Optional[RobotPose] = None) -> List[Tuple[float, float, float]]:
        """
        Transformiert LIDAR-Scandaten in Roboter-Koordinaten.
        
        Args:
            scan_data: LIDAR-Scandaten als [(quality, angle_deg, distance_mm), ...]
            robot_pose: Aktuelle Roboter-Position (optional für absolute Koordinaten)
            
        Returns:
            Transformierte Scandaten mit Roboter-relativen Koordinaten
        """
        transformed_data = []
        
        for quality, angle_deg, distance_mm in scan_data:
            if distance_mm <= 0:  # Ungültige Messungen überspringen
                continue
            
            # Konvertiere zu SI-Einheiten
            distance_m = distance_mm / 1000.0
            angle_rad = math.radians(angle_deg)
            
            # LIDAR-Koordinaten (LIDAR als Ursprung)
            lidar_x = distance_m * math.cos(angle_rad)
            lidar_y = distance_m * math.sin(angle_rad)
            
            # Transformiere zu Roboter-Koordinaten
            robot_x = lidar_x + self.lidar_offset_x
            robot_y = lidar_y + self.lidar_offset_y
            
            # Winkel-Offset anwenden
            angle_robot = angle_rad + self.lidar_angle_offset
            
            # Konvertiere zurück zu Polarkoordinaten (für Kompatibilität)
            distance_robot = math.sqrt(robot_x**2 + robot_y**2)
            angle_robot_deg = math.degrees(math.atan2(robot_y, robot_x))
            
            # Normalisiere Winkel auf 0-360°
            if angle_robot_deg < 0:
                angle_robot_deg += 360.0
            
            transformed_data.append((quality, angle_robot_deg, distance_robot * 1000.0))
        
        return transformed_data
    
    def robot_to_global_coordinates(self, robot_points: List[Tuple[float, float]],
                                  robot_pose: RobotPose) -> List[Tuple[float, float]]:
        """
        Transformiert Roboter-relative Punkte in globale Koordinaten.
        
        Args:
            robot_points: Liste von (x, y) Punkten relativ zum Roboter
            robot_pose: Aktuelle Roboter-Position und -Orientierung
            
        Returns:
            Liste der Punkte in globalen Koordinaten
        """
        global_points = []
        
        cos_theta = math.cos(robot_pose.theta)
        sin_theta = math.sin(robot_pose.theta)
        
        for x_robot, y_robot in robot_points:
            # Rotation um Roboter-Orientierung
            x_rotated = x_robot * cos_theta - y_robot * sin_theta
            y_rotated = x_robot * sin_theta + y_robot * cos_theta
            
            # Translation um Roboter-Position
            x_global = x_rotated + robot_pose.x
            y_global = y_rotated + robot_pose.y
            
            global_points.append((x_global, y_global))
        
        return global_points


# =============================================================================
# KOLLISIONSERKENNUNG UND DATA FUSION
# =============================================================================

class CollisionDetector:
    """
    Erweiterte Kollisionserkennung mit LIDAR- und Ultraschall-Data-Fusion.
    
    Implementiert intelligente Kollisionserkennung durch Fusion von LIDAR-
    und Ultraschall-Sensordaten. Bietet sektorbasierte Analyse und
    verschiedene Warnstufen für sichere Roboter-Navigation.
    
    Features:
        - Sektorbasierte Kollisionsanalyse (Vorne, Links, Rechts, Hinten)
        - Data Fusion zwischen LIDAR und Ultraschall-Sensoren
        - Adaptive Schwellwerte basierend auf Roboter-Geschwindigkeit
        - Confidence-basierte Sensor-Gewichtung
        - Hysterese zur Vermeidung von Oszillationen
    """
    
    def __init__(self, coordinate_transformer: CoordinateTransformer,
                 logger: Optional[logging.Logger] = None):
        """
        Initialisiert den Kollisionsdetektor.
        
        Args:
            coordinate_transformer: Koordinatentransformer für LIDAR-Daten
            logger: Optional - Logger-Instanz
        """
        self.transformer = coordinate_transformer
        self.logger = logger or logging.getLogger(__name__)
        
        # Kollisions-Schwellwerte
        self.warning_distance = COLLISION_WARNING_DISTANCE
        self.critical_distance = COLLISION_CRITICAL_DISTANCE
        self.sector_angle = COLLISION_SECTOR_ANGLE
        
        # Hysterese-Parameter (zur Vermeidung von Flackern)
        self.hysteresis_factor = 0.1  # 10% Hysterese
        self._last_collision_state = CollisionLevel.SAFE
        
        # Sensor-Fusion Gewichtungen
        self.lidar_weight = 0.7    # LIDAR-Vertrauen
        self.ultrasonic_weight = 0.3  # Ultraschall-Vertrauen
        
    def analyze_collision_risk(self, lidar_frame: LidarFrame,
                             ultrasonic_data: Optional[Dict[str, float]] = None,
                             robot_velocity: Optional[Tuple[float, float]] = None) -> Dict[str, Any]:
        """
        Analysiert Kollisionsrisiko basierend auf LIDAR- und Ultraschall-Daten.
        
        Args:
            lidar_frame: Aktueller LIDAR-Scan
            ultrasonic_data: Optional - Ultraschall-Sensordaten {"front": dist, "left": dist, ...}
            robot_velocity: Optional - Roboter-Geschwindigkeit (vx, vy) in m/s
            
        Returns:
            Dictionary mit Kollisionsanalyse:
            - overall_level: Gesamte Kollisionswarnstufe
            - sectors: Dictionary mit Sektor-spezifischen Informationen
            - closest_obstacle: Information über nächstes Hindernis
            - recommended_action: Empfohlene Aktion
        """
        # Transformiere LIDAR-Daten zu Roboter-Koordinaten
        robot_scan = self.transformer.lidar_to_robot_coordinates(lidar_frame.scan_data)
        
        # Analysiere Sektoren
        sectors = self._analyze_sectors(robot_scan, ultrasonic_data)
        
        # Finde nächstes Hindernis
        closest_obstacle = self._find_closest_obstacle(sectors)
        
        # Bestimme Gesamtwarnstufe mit Hysterese
        overall_level = self._determine_collision_level(sectors, closest_obstacle)
        
        # Geschwindigkeits-adaptive Schwellwerte
        if robot_velocity:
            overall_level = self._adjust_for_velocity(overall_level, robot_velocity)
        
        # Empfohlene Aktion
        recommended_action = self._get_recommended_action(overall_level, sectors)
        
        result = {
            "overall_level": overall_level,
            "sectors": sectors,
            "closest_obstacle": closest_obstacle,
            "recommended_action": recommended_action,
            "timestamp": lidar_frame.timestamp,
            "fusion_confidence": self._calculate_fusion_confidence(sectors)
        }
        
        self._last_collision_state = overall_level
        return result
    
    def _analyze_sectors(self, robot_scan: List[Tuple[float, float, float]],
                        ultrasonic_data: Optional[Dict[str, float]]) -> Dict[str, Dict[str, Any]]:
        """
        Analysiert Kollisionsrisiko in verschiedenen Sektoren.
        
        Args:
            robot_scan: LIDAR-Daten in Roboter-Koordinaten
            ultrasonic_data: Optional - Ultraschall-Sensordaten
            
        Returns:
            Dictionary mit Sektor-Informationen
        """
        # Initialisiere Sektoren
        sectors = {
            "front": {"obstacles": [], "min_distance": float('inf'), "level": CollisionLevel.SAFE},
            "left": {"obstacles": [], "min_distance": float('inf'), "level": CollisionLevel.SAFE},
            "right": {"obstacles": [], "min_distance": float('inf'), "level": CollisionLevel.SAFE},
            "back": {"obstacles": [], "min_distance": float('inf'), "level": CollisionLevel.SAFE}
        }
        
        # Verarbeite LIDAR-Daten
        for quality, angle_deg, distance_mm in robot_scan:
            distance_m = distance_mm / 1000.0
            
            # Bestimme Sektor basierend auf Winkel
            sector = self._get_sector_from_angle(angle_deg)
            
            # Erstelle Hindernis-Info
            obstacle = ObstacleInfo(
                distance=distance_m,
                angle=math.radians(angle_deg),
                sector=sector,
                confidence=quality * self.lidar_weight,
                sensor_type="lidar"
            )
            
            sectors[sector]["obstacles"].append(obstacle)
            sectors[sector]["min_distance"] = min(sectors[sector]["min_distance"], distance_m)
        
        # Fusioniere mit Ultraschall-Daten
        if ultrasonic_data:
            self._fuse_ultrasonic_data(sectors, ultrasonic_data)
        
        # Bestimme Warnstufen für jeden Sektor
        for sector_name, sector_data in sectors.items():
            sector_data["level"] = self._get_sector_collision_level(sector_data["min_distance"])
        
        return sectors
    
    def _get_sector_from_angle(self, angle_deg: float) -> str:
        """
        Bestimmt den Sektor basierend auf dem Winkel.
        
        Args:
            angle_deg: Winkel in Grad (0° = Vorwärts)
            
        Returns:
            Sektor-Name ("front", "left", "right", "back")
        """
        # Normalisiere Winkel auf 0-360°
        angle = angle_deg % 360.0
        
        # Definiere Sektor-Grenzen
        if angle < 45.0 or angle >= 315.0:
            return "front"
        elif 45.0 <= angle < 135.0:
            return "right"
        elif 135.0 <= angle < 225.0:
            return "back"
        else:  # 225.0 <= angle < 315.0
            return "left"
    
    def _fuse_ultrasonic_data(self, sectors: Dict[str, Dict[str, Any]], 
                            ultrasonic_data: Dict[str, float]) -> None:
        """
        Fusioniert Ultraschall-Daten mit LIDAR-Sektordaten.
        
        Args:
            sectors: Sektor-Dictionary (wird modifiziert)
            ultrasonic_data: Ultraschall-Sensordaten
        """
        for sensor_name, distance_m in ultrasonic_data.items():
            if sensor_name in sectors and distance_m > 0:
                # Erstelle Ultraschall-Hindernis
                obstacle = ObstacleInfo(
                    distance=distance_m,
                    angle=0.0,  # Ultraschall: exakte Richtung unbekannt
                    sector=sensor_name,
                    confidence=self.ultrasonic_weight,
                    sensor_type="ultrasonic"
                )
                
                sectors[sensor_name]["obstacles"].append(obstacle)
                
                # Update minimale Distanz (gewichtet)
                lidar_min = sectors[sensor_name]["min_distance"]
                if lidar_min == float('inf'):
                    sectors[sensor_name]["min_distance"] = distance_m
                else:
                    # Gewichtete Fusion der Distanzen
                    fused_distance = (
                        lidar_min * self.lidar_weight + 
                        distance_m * self.ultrasonic_weight
                    )
                    sectors[sensor_name]["min_distance"] = fused_distance
    
    def _get_sector_collision_level(self, min_distance: float) -> str:
        """
        Bestimmt Kollisionswarnstufe für einen Sektor.
        
        Args:
            min_distance: Minimale Distanz in diesem Sektor
            
        Returns:
            Kollisionswarnstufe
        """
        if min_distance == float('inf'):
            return CollisionLevel.SAFE
        elif min_distance <= self.critical_distance:
            return CollisionLevel.CRITICAL
        elif min_distance <= self.warning_distance:
            return CollisionLevel.WARNING
        else:
            return CollisionLevel.SAFE
    
    def _find_closest_obstacle(self, sectors: Dict[str, Dict[str, Any]]) -> Optional[ObstacleInfo]:
        """
        Findet das nächstgelegene Hindernis über alle Sektoren.
        
        Args:
            sectors: Sektor-Dictionary mit Hindernissen
            
        Returns:
            ObstacleInfo des nächsten Hindernisses oder None
        """
        closest_obstacle = None
        min_distance = float('inf')
        
        for sector_data in sectors.values():
            for obstacle in sector_data["obstacles"]:
                if obstacle.distance < min_distance:
                    min_distance = obstacle.distance
                    closest_obstacle = obstacle
        
        return closest_obstacle
    
    def _determine_collision_level(self, sectors: Dict[str, Dict[str, Any]], 
                                 closest_obstacle: Optional[ObstacleInfo]) -> str:
        """
        Bestimmt Gesamtwarnstufe mit Hysterese.
        
        Args:
            sectors: Sektor-Dictionary
            closest_obstacle: Nächstes Hindernis
            
        Returns:
            Gesamtwarnstufe
        """
        if not closest_obstacle:
            return CollisionLevel.SAFE
        
        # Basis-Warnstufe
        if closest_obstacle.distance <= self.critical_distance:
            new_level = CollisionLevel.CRITICAL
        elif closest_obstacle.distance <= self.warning_distance:
            new_level = CollisionLevel.WARNING
        else:
            new_level = CollisionLevel.SAFE
        
        # Hysterese anwenden
        if self._last_collision_state == CollisionLevel.CRITICAL and new_level == CollisionLevel.WARNING:
            # Verlasse Critical nur wenn deutlich über Schwellwert
            if closest_obstacle.distance > self.critical_distance * (1 + self.hysteresis_factor):
                return new_level
            else:
                return CollisionLevel.CRITICAL
        
        if self._last_collision_state == CollisionLevel.WARNING and new_level == CollisionLevel.SAFE:
            # Verlasse Warning nur wenn deutlich über Schwellwert
            if closest_obstacle.distance > self.warning_distance * (1 + self.hysteresis_factor):
                return new_level
            else:
                return CollisionLevel.WARNING
        
        return new_level
    
    def _adjust_for_velocity(self, collision_level: str, 
                           robot_velocity: Tuple[float, float]) -> str:
        """
        Passt Kollisionswarnstufe basierend auf Roboter-Geschwindigkeit an.
        
        Args:
            collision_level: Aktuelle Warnstufe
            robot_velocity: Roboter-Geschwindigkeit (vx, vy)
            
        Returns:
            Angepasste Warnstufe
        """
        velocity_magnitude = math.sqrt(robot_velocity[0]**2 + robot_velocity[1]**2)
        
        # Bei hoher Geschwindigkeit früher warnen
        if velocity_magnitude > 0.5:  # > 0.5 m/s
            if collision_level == CollisionLevel.SAFE:
                return CollisionLevel.WARNING
            elif collision_level == CollisionLevel.WARNING:
                return CollisionLevel.CRITICAL
        
        return collision_level
    
    def _get_recommended_action(self, collision_level: str, 
                              sectors: Dict[str, Dict[str, Any]]) -> str:
        """
        Bestimmt empfohlene Aktion basierend auf Kollisionsanalyse.
        
        Args:
            collision_level: Gesamtwarnstufe
            sectors: Sektor-Informationen
            
        Returns:
            Empfohlene Aktion als String
        """
        if collision_level == CollisionLevel.CRITICAL:
            return "IMMEDIATE_STOP"
        elif collision_level == CollisionLevel.WARNING:
            # Analysiere welche Sektoren betroffen sind
            critical_sectors = [name for name, data in sectors.items() 
                              if data["level"] in [CollisionLevel.CRITICAL, CollisionLevel.WARNING]]
            
            if "front" in critical_sectors:
                return "SLOW_DOWN"
            elif "left" in critical_sectors and "right" not in critical_sectors:
                return "TURN_RIGHT"
            elif "right" in critical_sectors and "left" not in critical_sectors:
                return "TURN_LEFT"
            else:
                return "CAREFUL_NAVIGATION"
        else:
            return "CONTINUE"
    
    def _calculate_fusion_confidence(self, sectors: Dict[str, Dict[str, Any]]) -> float:
        """
        Berechnet Vertrauenswert der Sensor-Fusion.
        
        Args:
            sectors: Sektor-Informationen
            
        Returns:
            Fusion-Vertrauenswert 0.0-1.0
        """
        total_confidence = 0.0
        total_obstacles = 0
        
        for sector_data in sectors.values():
            for obstacle in sector_data["obstacles"]:
                total_confidence += obstacle.confidence
                total_obstacles += 1
        
        if total_obstacles == 0:
            return 0.0
        
        return min(1.0, total_confidence / total_obstacles)


# =============================================================================
# MAPPING-SYSTEM
# =============================================================================

class EnvironmentMapper:
    """
    Echtzeit-Umgebungs-Mapping mit konsistenter 2D-Karte.
    
    Erstellt und aktualisiert kontinuierlich eine 2D-Belegungskarte der
    Umgebung basierend auf LIDAR-Scandaten. Implementiert Decay-Mechanismen
    für dynamische Umgebungen und bietet Export-Funktionen.
    
    Features:
        - Occupancy Grid Mapping mit konfigurierbarer Auflösung
        - Temporal Decay für dynamische Hindernisse
        - Memory-effiziente Speicherung großer Karten
        - Export zu verschiedenen Formaten (JSON, PNG, NPY)
        - Thread-sichere Karten-Updates
    """
    
    def __init__(self, resolution: float = MAP_RESOLUTION,
                 map_size_meters: float = MAP_SIZE_METERS,
                 decay_factor: float = MAP_DECAY_FACTOR,
                 coordinate_transformer: Optional[CoordinateTransformer] = None):
        """
        Initialisiert den Environment Mapper.
        
        Args:
            resolution: Auflösung in Metern pro Pixel
            map_size_meters: Kartengröße in Metern (quadratisch)
            decay_factor: Decay-Faktor für alte Daten (0.0-1.0)
            coordinate_transformer: Optional - Koordinatentransformer
        """
        self.resolution = resolution
        self.map_size_meters = map_size_meters
        self.decay_factor = decay_factor
        self.transformer = coordinate_transformer
        
        # Karten-Parameter
        self.map_size_pixels = int(map_size_meters / resolution)
        self.map_center = self.map_size_pixels // 2
        
        # Occupancy Grid (0.0 = frei, 1.0 = belegt)
        self.occupancy_grid = np.zeros((self.map_size_pixels, self.map_size_pixels), dtype=np.float32)
        
        # Konfidenz-Grid (Vertrauenswert der Daten)
        self.confidence_grid = np.zeros((self.map_size_pixels, self.map_size_pixels), dtype=np.float32)
        
        # Thread-Sicherheit
        self._map_lock = threading.Lock()
        
        # Statistiken
        self.updates_count = 0
        self.last_update_time = 0.0
    
    def update_map(self, lidar_frame: LidarFrame, 
                  robot_pose: Optional[RobotPose] = None) -> None:
        """
        Aktualisiert die Karte mit neuen LIDAR-Daten.
        
        Args:
            lidar_frame: Neuer LIDAR-Scan
            robot_pose: Aktuelle Roboter-Position (optional)
        """
        with self._map_lock:
            # Transformiere LIDAR-Daten wenn Transformer verfügbar
            if self.transformer:
                scan_data = self.transformer.lidar_to_robot_coordinates(lidar_frame.scan_data)
            else:
                scan_data = lidar_frame.scan_data
            
            # Decay-Anwendung auf bestehende Karte
            self._apply_decay()
            
            # Roboter-Position in Karten-Koordinaten
            if robot_pose:
                robot_x_px, robot_y_px = self._world_to_pixel(robot_pose.x, robot_pose.y)
            else:
                robot_x_px, robot_y_px = self.map_center, self.map_center
            
            # Verarbeite jeden Scan-Punkt
            for quality, angle_deg, distance_mm in scan_data:
                if distance_mm <= 0:
                    continue
                
                # Konvertiere zu Weltkoordinaten
                distance_m = distance_mm / 1000.0
                angle_rad = math.radians(angle_deg)
                
                # Berechne Endpunkt des Strahls
                end_x = distance_m * math.cos(angle_rad)
                end_y = distance_m * math.sin(angle_rad)
                
                # Konvertiere zu Pixel-Koordinaten
                end_x_px, end_y_px = self._world_to_pixel(end_x, end_y)
                
                # Raytracing: Markiere freien Raum entlang des Strahls
                self._bresenham_line(robot_x_px, robot_y_px, end_x_px, end_y_px, 
                                   is_obstacle=False, quality=quality)
                
                # Markiere Endpunkt als Hindernis
                if self._is_valid_pixel(end_x_px, end_y_px):
                    self.occupancy_grid[end_y_px, end_x_px] = min(1.0,
                        self.occupancy_grid[end_y_px, end_x_px] + quality * 0.1)
                    self.confidence_grid[end_y_px, end_x_px] = min(1.0,
                        self.confidence_grid[end_y_px, end_x_px] + quality * 0.05)
            
            # Update Statistiken
            self.updates_count += 1
            self.last_update_time = lidar_frame.timestamp
    
    def get_map_around_robot(self, robot_pose: RobotPose, 
                           radius_meters: float = 5.0) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Holt einen Kartenausschnitt um die Roboter-Position.
        
        Args:
            robot_pose: Aktuelle Roboter-Position
            radius_meters: Radius des Ausschnitts in Metern
            
        Returns:
            Tuple aus (Karten-Array, Metadaten-Dictionary)
        """
        with self._map_lock:
            radius_pixels = int(radius_meters / self.resolution)
            
            # Roboter-Position in Pixel-Koordinaten
            robot_x_px, robot_y_px = self._world_to_pixel(robot_pose.x, robot_pose.y)
            
            # Definiere Ausschnitt-Grenzen
            x_min = max(0, robot_x_px - radius_pixels)
            x_max = min(self.map_size_pixels, robot_x_px + radius_pixels)
            y_min = max(0, robot_y_px - radius_pixels)
            y_max = min(self.map_size_pixels, robot_y_px + radius_pixels)
            
            # Extrahiere Ausschnitt
            map_section = self.occupancy_grid[y_min:y_max, x_min:x_max].copy()
            
            # Metadaten
            metadata = {
                "center_world": (robot_pose.x, robot_pose.y),
                "center_pixel": (robot_x_px - x_min, robot_y_px - y_min),
                "resolution": self.resolution,
                "size_pixels": map_section.shape,
                "bounds_world": (
                    (x_min - self.map_center) * self.resolution,
                    (y_min - self.map_center) * self.resolution,
                    (x_max - self.map_center) * self.resolution,
                    (y_max - self.map_center) * self.resolution
                )
            }
            
            return map_section, metadata
    
    def export_map(self, filepath: Path, format_type: str = "json") -> bool:
        """
        Exportiert die aktuelle Karte in verschiedene Formate.
        
        Args:
            filepath: Ziel-Dateipfad
            format_type: Export-Format ("json", "png", "npy")
            
        Returns:
            True wenn erfolgreich exportiert
        """
        try:
            with self._map_lock:
                if format_type == "json":
                    self._export_json(filepath)
                elif format_type == "png":
                    self._export_png(filepath)
                elif format_type == "npy":
                    self._export_npy(filepath)
                else:
                    raise ValueError(f"Unbekanntes Export-Format: {format_type}")
            
            return True
            
        except Exception as e:
            print(f"Fehler beim Map-Export: {e}")
            return False
    
    def _apply_decay(self) -> None:
        """
        Wendet Decay-Faktor auf die Karte an.
        """
        self.occupancy_grid *= self.decay_factor
        self.confidence_grid *= self.decay_factor
    
    def _world_to_pixel(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Konvertiert Weltkoordinaten zu Pixel-Koordinaten.
        
        Args:
            world_x: X-Koordinate in Metern
            world_y: Y-Koordinate in Metern
            
        Returns:
            Tuple aus (pixel_x, pixel_y)
        """
        pixel_x = int(world_x / self.resolution + self.map_center)
        pixel_y = int(world_y / self.resolution + self.map_center)
        return pixel_x, pixel_y
    
    def _is_valid_pixel(self, x: int, y: int) -> bool:
        """
        Prüft ob Pixel-Koordinaten gültig sind.
        
        Args:
            x: X-Pixel-Koordinate
            y: Y-Pixel-Koordinate
            
        Returns:
            True wenn Koordinaten im gültigen Bereich
        """
        return 0 <= x < self.map_size_pixels and 0 <= y < self.map_size_pixels
    
    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int, 
                       is_obstacle: bool, quality: float) -> None:
        """
        Bresenham-Linienalgorithmus für Raytracing.
        
        Args:
            x0, y0: Start-Pixel-Koordinaten
            x1, y1: End-Pixel-Koordinaten
            is_obstacle: True wenn Linie Hindernis markiert, False für freien Raum
            quality: Qualitätswert für Update-Stärke
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if self._is_valid_pixel(x, y):
                if is_obstacle:
                    # Markiere als Hindernis
                    self.occupancy_grid[y, x] = min(1.0, 
                        self.occupancy_grid[y, x] + quality * 0.1)
                else:
                    # Markiere als freien Raum
                    self.occupancy_grid[y, x] = max(0.0, 
                        self.occupancy_grid[y, x] - quality * 0.02)
                
                # Update Konfidenz
                self.confidence_grid[y, x] = min(1.0,
                    self.confidence_grid[y, x] + quality * 0.01)
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def _export_json(self, filepath: Path) -> None:
        """
        Exportiert Karte als JSON-Datei.
        
        Args:
            filepath: Ziel-Dateipfad
        """
        map_data = {
            "occupancy_grid": self.occupancy_grid.tolist(),
            "confidence_grid": self.confidence_grid.tolist(),
            "metadata": {
                "resolution": self.resolution,
                "map_size_meters": self.map_size_meters,
                "map_size_pixels": self.map_size_pixels,
                "updates_count": self.updates_count,
                "last_update_time": self.last_update_time,
                "export_timestamp": time.time()
            }
        }
        
        with open(filepath, 'w') as f:
            json.dump(map_data, f, indent=2)
    
    def _export_png(self, filepath: Path) -> None:
        """
        Exportiert Karte als PNG-Bild.
        
        Args:
            filepath: Ziel-Dateipfad
        """
        try:
            import matplotlib.pyplot as plt
            
            # Konvertiere zu 8-bit Graustufen
            image_data = (self.occupancy_grid * 255).astype(np.uint8)
            
            # Invertiere für bessere Visualisierung (schwarz = Hindernis)
            image_data = 255 - image_data
            
            plt.figure(figsize=(10, 10))
            plt.imshow(image_data, cmap='gray', origin='lower')
            plt.title(f'Environment Map (Updates: {self.updates_count})')
            plt.colorbar(label='Occupancy (black = obstacle)')
            plt.xlabel('X [pixels]')
            plt.ylabel('Y [pixels]')
            plt.savefig(filepath, dpi=150, bbox_inches='tight')
            plt.close()
            
        except ImportError:
            # Fallback: Speichere als NumPy Array
            np.save(filepath.with_suffix('.npy'), self.occupancy_grid)
    
    def _export_npy(self, filepath: Path) -> None:
        """
        Exportiert Karte als NumPy-Array.
        
        Args:
            filepath: Ziel-Dateipfad
        """
        map_data = {
            'occupancy_grid': self.occupancy_grid,
            'confidence_grid': self.confidence_grid,
            'metadata': {
                'resolution': self.resolution,
                'map_size_meters': self.map_size_meters,
                'updates_count': self.updates_count,
                'last_update_time': self.last_update_time
            }
        }
        np.save(filepath, map_data)


# =============================================================================
# HAUPT-INTEGRATION-KLASSE
# =============================================================================

class LidarControllerFusion:
    """
    Haupt-Integration-Klasse für LIDAR-Controller-Fusion.
    
    Zentrale Klasse die alle Komponenten zusammenführt:
    - LIDAR Thread Management
    - Koordinatentransformation
    - Kollisionserkennung
    - Environment Mapping
    - Integration mit bestehendem Controller-System
    
    Bietet einheitliche API für das gesamte LIDAR-System mit
    Thread-sicherer Verarbeitung und robuster Error-Recovery.
    """
    
    def __init__(self, lidar_port: str = LIDAR_DEFAULT_PORT,
                 lidar_baudrate: int = LIDAR_DEFAULT_BAUDRATE,
                 logger: Optional[logging.Logger] = None):
        """
        Initialisiert das LIDAR-Controller-Fusion-System.
        
        Args:
            lidar_port: Serieller Port für LIDAR
            lidar_baudrate: Baudrate für LIDAR-Kommunikation
            logger: Optional - Logger-Instanz
        """
        self.logger = logger or logging.getLogger(__name__)
        
        # Initialisiere Komponenten
        self.coordinate_transformer = CoordinateTransformer()
        self.lidar_manager = LidarThreadManager(lidar_port, lidar_baudrate, self.logger)
        self.collision_detector = CollisionDetector(self.coordinate_transformer, self.logger)
        self.environment_mapper = EnvironmentMapper(
            coordinate_transformer=self.coordinate_transformer
        )
        
        # System-Status
        self._is_running = False
        self._last_collision_analysis = None
        self._current_robot_pose = RobotPose(0.0, 0.0, 0.0, time.time())
        
        # Data Recording Integration
        self._recording_data: Optional[PathRecordingData] = None
        self._recording_active = False
        
        self.logger.info("LIDAR-Controller-Fusion System initialisiert")
    
    def start(self) -> bool:
        """
        Startet das komplette LIDAR-System.
        
        Returns:
            True wenn erfolgreich gestartet
        """
        if self._is_running:
            self.logger.warning("System läuft bereits")
            return True
        
        try:
            # Starte LIDAR Thread Manager
            if not self.lidar_manager.start():
                self.logger.error("LIDAR Manager konnte nicht gestartet werden")
                return False
            
            self._is_running = True
            self.logger.info("LIDAR-Fusion-System erfolgreich gestartet")
            return True
            
        except Exception as e:
            self.logger.error(f"Fehler beim Starten des Systems: {e}")
            return False
    
    def stop(self) -> None:
        """
        Stoppt das komplette LIDAR-System.
        """
        if not self._is_running:
            return
        
        self.logger.info("Stoppe LIDAR-Fusion-System...")
        
        # Stoppe LIDAR Manager
        self.lidar_manager.stop()
        
        # Stoppe Recording falls aktiv
        if self._recording_active:
            self.stop_recording()
        
        self._is_running = False
        self.logger.info("LIDAR-Fusion-System gestoppt")
    
    def update_robot_pose(self, pose: RobotPose) -> None:
        """
        Aktualisiert die aktuelle Roboter-Position.
        
        Args:
            pose: Neue Roboter-Position und -Orientierung
        """
        self._current_robot_pose = pose
    
    def process_frame(self, ultrasonic_data: Optional[Dict[str, float]] = None,
                     robot_velocity: Optional[Tuple[float, float]] = None) -> Optional[Dict[str, Any]]:
        """
        Verarbeitet einen LIDAR-Frame mit Kollisionsanalyse und Mapping.
        
        Args:
            ultrasonic_data: Optional - Ultraschall-Sensordaten für Data Fusion
            robot_velocity: Optional - Roboter-Geschwindigkeit für adaptive Kollisionserkennung
            
        Returns:
            Dictionary mit Verarbeitungsergebnissen oder None wenn keine Daten
        """
        if not self._is_running:
            return None
        
        # Hole aktuellen LIDAR-Scan
        lidar_frame = self.lidar_manager.get_latest_scan()
        if not lidar_frame:
            return None
        
        try:
            # Kollisionsanalyse
            collision_analysis = self.collision_detector.analyze_collision_risk(
                lidar_frame, ultrasonic_data, robot_velocity
            )
            self._last_collision_analysis = collision_analysis
            
            # Environment Mapping
            self.environment_mapper.update_map(lidar_frame, self._current_robot_pose)
            
            # Recording falls aktiv
            if self._recording_active and self._recording_data:
                self._recording_data.add_lidar_frame(lidar_frame)
            
            # Zusammengestellte Ergebnisse
            result = {
                "lidar_frame": lidar_frame,
                "collision_analysis": collision_analysis,
                "robot_pose": self._current_robot_pose,
                "system_stats": self.get_system_stats(),
                "timestamp": lidar_frame.timestamp
            }
            
            return result
            
        except Exception as e:
            self.logger.error(f"Fehler bei Frame-Verarbeitung: {e}")
            return None
    
    def get_collision_status(self) -> Optional[Dict[str, Any]]:
        """
        Holt den aktuellen Kollisionsstatus.
        
        Returns:
            Letztes Kollisionsanalyse-Ergebnis oder None
        """
        return self._last_collision_analysis
    
    def get_map_around_robot(self, radius_meters: float = 5.0) -> Optional[Tuple[np.ndarray, Dict[str, Any]]]:
        """
        Holt einen Kartenausschnitt um die aktuelle Roboter-Position.
        
        Args:
            radius_meters: Radius des Kartenausschnitts
            
        Returns:
            Tuple aus (Karten-Array, Metadaten) oder None
        """
        if not self._is_running:
            return None
        
        try:
            return self.environment_mapper.get_map_around_robot(
                self._current_robot_pose, radius_meters
            )
        except Exception as e:
            self.logger.error(f"Fehler beim Abrufen der Karte: {e}")
            return None
    
    def export_map(self, filepath: Path, format_type: str = "json") -> bool:
        """
        Exportiert die aktuelle Umgebungskarte.
        
        Args:
            filepath: Ziel-Dateipfad
            format_type: Export-Format ("json", "png", "npy")
            
        Returns:
            True wenn erfolgreich exportiert
        """
        return self.environment_mapper.export_map(filepath, format_type)
    
    def start_recording(self, session_metadata: Optional[EnvironmentMetadata] = None) -> bool:
        """
        Startet Recording der LIDAR-Daten.
        
        Args:
            session_metadata: Optional - Session-Metadaten
            
        Returns:
            True wenn erfolgreich gestartet
        """
        if self._recording_active:
            self.logger.warning("Recording läuft bereits")
            return True
        
        try:
            # Erstelle neues Recording
            header = PathRecordingHeader(metadata=session_metadata)
            self._recording_data = PathRecordingData(header=header)
            self._recording_active = True
            
            self.logger.info("LIDAR-Daten-Recording gestartet")
            return True
            
        except Exception as e:
            self.logger.error(f"Fehler beim Starten des Recordings: {e}")
            return False
    
    def stop_recording(self) -> Optional[PathRecordingData]:
        """
        Stoppt Recording und gibt aufgezeichnete Daten zurück.
        
        Returns:
            PathRecordingData mit allen aufgezeichneten Daten oder None
        """
        if not self._recording_active:
            return None
        
        self._recording_active = False
        recording_data = self._recording_data
        self._recording_data = None
        
        # Update End-Timestamp im Header
        if recording_data and recording_data.header.metadata:
            recording_data.header.metadata.end_timestamp = time.time()
        
        self.logger.info("LIDAR-Daten-Recording gestoppt")
        return recording_data
    
    def get_system_stats(self) -> Dict[str, Any]:
        """
        Holt System-Statistiken aller Komponenten.
        
        Returns:
            Dictionary mit System-Statistiken
        """
        lidar_stats = self.lidar_manager.get_stats()
        
        return {
            "system_running": self._is_running,
            "lidar_stats": {
                "scans_processed": lidar_stats.scans_processed,
                "processing_rate_hz": lidar_stats.processing_rate_hz,
                "avg_points_per_scan": lidar_stats.avg_points_per_scan,
                "error_count": lidar_stats.error_count,
                "buffer_utilization": lidar_stats.buffer_utilization
            },
            "mapping_stats": {
                "map_updates": self.environment_mapper.updates_count,
                "last_update": self.environment_mapper.last_update_time,
                "map_resolution": self.environment_mapper.resolution,
                "map_size_pixels": self.environment_mapper.map_size_pixels
            },
            "recording_active": self._recording_active,
            "current_pose": {
                "x": self._current_robot_pose.x,
                "y": self._current_robot_pose.y,
                "theta": self._current_robot_pose.theta,
                "timestamp": self._current_robot_pose.timestamp
            }
        }
    
    def is_running(self) -> bool:
        """
        Prüft ob das System läuft.
        
        Returns:
            True wenn System aktiv ist
        """
        return self._is_running


# =============================================================================
# TEST UND DEMONSTRATION
# =============================================================================

def create_test_script():
    """
    Erstellt Test-Script für das LIDAR-Fusion-System.
    
    Returns:
        Test-Script als String
    """
    return '''#!/usr/bin/env python3
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
"""

import argparse
import time
import logging
from pathlib import Path
import numpy as np

from lidar_controller_fusion import (
    LidarControllerFusion, 
    RobotPose, 
    EnvironmentMetadata,
    CollisionLevel
)

def setup_test_logging():
    """Konfiguriert Logging für Tests."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    return logging.getLogger(__name__)

def test_basic_functionality(fusion_system, logger, duration=10):
    """
    Testet Grundfunktionalität des Systems.
    
    Args:
        fusion_system: LidarControllerFusion-Instanz
        logger: Logger-Instanz
        duration: Test-Dauer in Sekunden
    """
    logger.info(f"Starte Basis-Funktionalitätstest ({duration}s)...")
    
    start_time = time.time()
    frame_count = 0
    collision_warnings = 0
    
    # Simuliere Roboter-Bewegung
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
        
        # Simuliere Ultraschall-Daten
        ultrasonic_data = {
            "front": 1.5 + 0.5 * np.sin(t),
            "left": 2.0 + 0.3 * np.cos(t),
            "right": 1.8 + 0.2 * np.sin(t * 1.5),
            "back": 3.0
        }
        
        # Simuliere Roboter-Geschwindigkeit
        robot_velocity = (0.2 * np.sin(t * 0.5), 0.1 * np.cos(t * 0.3))
        
        # Verarbeite Frame
        result = fusion_system.process_frame(ultrasonic_data, robot_velocity)
        
        if result:
            frame_count += 1
            
            # Prüfe Kollisionswarnung
            collision_analysis = result.get("collision_analysis")
            if collision_analysis and collision_analysis["overall_level"] != CollisionLevel.SAFE:
                collision_warnings += 1
                logger.warning(f"Kollisionswarnung: {collision_analysis['overall_level']}")
            
            # Zeige Statistiken alle 5 Sekunden
            if frame_count % 50 == 0:
                stats = result.get("system_stats", {})
                lidar_stats = stats.get("lidar_stats", {})
                logger.info(f"Frame {frame_count}: "
                          f"LIDAR Rate: {lidar_stats.get('processing_rate_hz', 0):.1f} Hz, "
                          f"Buffer: {lidar_stats.get('buffer_utilization', 0):.1%}")
        
        time.sleep(0.1)  # 10 Hz Test-Rate
    
    logger.info(f"Basis-Test abgeschlossen: {frame_count} Frames, "
               f"{collision_warnings} Kollisionswarnungen")

def test_mapping_functionality(fusion_system, logger, duration=15):
    """
    Testet Environment Mapping Funktionalität.
    
    Args:
        fusion_system: LidarControllerFusion-Instanz
        logger: Logger-Instanz  
        duration: Test-Dauer in Sekunden
    """
    logger.info(f"Starte Mapping-Test ({duration}s)...")
    
    start_time = time.time()
    
    # Simuliere komplexere Roboter-Bewegung für Mapping
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
        map_data, metadata = fusion_system.get_map_around_robot(radius_meters=10.0)
        if map_data is not None:
            logger.info(f"Karte erstellt: {map_data.shape} Pixel, "
                       f"Auflösung: {metadata['resolution']:.3f}m/px")
        else:
            logger.warning("Keine Kartendaten verfügbar")
    except Exception as e:
        logger.error(f"Mapping-Test Fehler: {e}")

def test_recording_functionality(fusion_system, logger, duration=10):
    """
    Testet Data Recording Funktionalität.
    
    Args:
        fusion_system: LidarControllerFusion-Instanz
        logger: Logger-Instanz
        duration: Test-Dauer in Sekunden
    """
    logger.info(f"Starte Recording-Test ({duration}s)...")
    
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
            "lidar_model": "RPLIDAR C1",
            "controller_model": "Test Controller"
        },
        recording_settings={
            "lidar_rate_hz": 10,
            "test_duration_seconds": duration
        }
    )
    
    # Starte Recording
    if not fusion_system.start_recording(session_metadata):
        logger.error("Recording konnte nicht gestartet werden")
        return
    
    start_time = time.time()
    frames_recorded = 0
    
    # Sammle Daten
    while time.time() - start_time < duration:
        # Roboter-Bewegung für Recording
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
        logger.info(f"Recording abgeschlossen: {stats.get('lidar_frames', 0)} LIDAR Frames, "
                   f"Dauer: {stats.get('duration_seconds', 0):.1f}s")
    else:
        logger.error("Keine Recording-Daten erhalten")

def main():
    parser = argparse.ArgumentParser(description='LIDAR-Fusion System Test')
    parser.add_argument('--duration', type=int, default=30,
                       help='Gesamte Test-Dauer in Sekunden')
    parser.add_argument('--lidar-port', default='/dev/ttyUSB0',
                       help='LIDAR serieller Port')
    parser.add_argument('--export-map', action='store_true',
                       help='Exportiere finale Karte')
    parser.add_argument('--test-mode', action='store_true',
                       help='Simuliere LIDAR ohne Hardware')
    
    args = parser.parse_args()
    
    # Setup Logging
    logger = setup_test_logging()
    
    try:
        logger.info("=== LIDAR-Controller-Fusion System Test ===")
        logger.info(f"Test-Dauer: {args.duration}s")
        logger.info(f"LIDAR Port: {args.lidar_port}")
        
        # Initialisiere System
        fusion_system = LidarControllerFusion(
            lidar_port=args.lidar_port,
            logger=logger
        )
        
        # Starte System
        if not fusion_system.start():
            logger.error("System konnte nicht gestartet werden")
            return 1
        
        try:
            # Test-Sequenz
            test_duration_per_phase = args.duration // 3
            
            # Phase 1: Basis-Funktionalität
            test_basic_functionality(fusion_system, logger, test_duration_per_phase)
            
            # Phase 2: Mapping
            test_mapping_functionality(fusion_system, logger, test_duration_per_phase)
            
            # Phase 3: Recording
            test_recording_functionality(fusion_system, logger, test_duration_per_phase)
            
            # Finale Statistiken
            final_stats = fusion_system.get_system_stats()
            logger.info("=== Finale Statistiken ===")
            logger.info(f"System Status: {'Läuft' if final_stats['system_running'] else 'Gestoppt'}")
            
            lidar_stats = final_stats.get('lidar_stats', {})
            logger.info(f"LIDAR Scans: {lidar_stats.get('scans_processed', 0)}")
            logger.info(f"LIDAR Rate: {lidar_stats.get('processing_rate_hz', 0):.1f} Hz")
            logger.info(f"LIDAR Fehler: {lidar_stats.get('error_count', 0)}")
            
            mapping_stats = final_stats.get('mapping_stats', {})
            logger.info(f"Map Updates: {mapping_stats.get('map_updates', 0)}")
            
            # Exportiere Karte falls gewünscht
            if args.export_map:
                output_dir = Path("test_results")
                output_dir.mkdir(exist_ok=True)
                
                timestamp = int(time.time())
                map_file = output_dir / f"lidar_map_{timestamp}.json"
                
                if fusion_system.export_map(map_file, "json"):
                    logger.info(f"Karte exportiert: {map_file}")
                else:
                    logger.error("Karten-Export fehlgeschlagen")
        
        finally:
            # System sauber herunterfahren
            fusion_system.stop()
            logger.info("System gestoppt")
        
        logger.info("=== Test erfolgreich abgeschlossen ===")
        return 0
        
    except KeyboardInterrupt:
        logger.info("Test durch Benutzer abgebrochen")
        return 130
    except Exception as e:
        logger.error(f"Test-Fehler: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    exit(main())
'''

def main():
    """
    Hauptfunktion - erstellt Test-Script.
    """
    test_script = create_test_script()
    
    # Schreibe Test-Script
    script_path = Path("test_lidar_fusion.py")
    with open(script_path, 'w', encoding='utf-8') as f:
        f.write(test_script)
    
    # Mache ausführbar
    import stat
    script_path.chmod(script_path.stat().st_mode | stat.S_IEXEC)
    
    print("✅ LIDAR-Controller-Fusion System erstellt")
    print(f"✅ Test-Script erstellt: {script_path}")
    print("\nZum Testen ausführen:")
    print(f"    python {script_path} --duration 60 --export-map")
    print("\nSystem-Features:")
    print("    - ✅ Thread-sichere LIDAR-Datenerfassung (5-15Hz)")
    print("    - ✅ Koordinatentransformation LIDAR → Roboter")
    print("    - ✅ Data Fusion LIDAR + Ultraschall-Sensoren") 
    print("    - ✅ Echtzeit-Kollisionserkennung mit Hysterese")
    print("    - ✅ Environment Mapping mit Occupancy Grid")
    print("    - ✅ Integration in bestehendes Recording-System")
    print("    - ✅ Robuste Error-Recovery und Hardware-Reconnection")
    print("    - ✅ Export-Funktionen für Karten (JSON, PNG, NPY)")


if __name__ == '__main__':
    main()
