"""Alle Konstanten der Fahrsteuerung.

Die Werte sind unverändert aus der Referenz
``xbox_controller_with_recording_v3.0.1.py`` übernommen. Pin-Nummern sind
BCM-Nummern (``GPIO.setmode(GPIO.BCM)``).
"""

# Ultraschall (HC-SR04)
TRIG_LEFT = 12
ECHO_LEFT = 13
TRIG_RIGHT = 23
ECHO_RIGHT = 24
SOUND_SPEED = 34300  # cm/s

# Motor-Richtungspins. Bei DIR_HIGH_IS_BACKWARD = False bedeutet HIGH vorwärts.
DIR_LEFT_PIN = 20
DIR_RIGHT_PIN = 21
DIR_HIGH_IS_BACKWARD = False

# Hardware-PWM über rpi_hardware_pwm (/sys/class/pwm/pwmchip<PWM_CHIP>/pwm<Kanal>).
# Duty Cycle in Prozent (0-100).
PWM_FREQUENCY = 1000
PWM_CHIP = 0
PWM_CHANNEL_LEFT = 0
PWM_CHANNEL_RIGHT = 1
MAX_DUTY_CYCLE = 12.0
MIN_DUTY_CYCLE = 0.0

# Status-LED und Buzzer
STATUS_LED_PIN = 22
BUZZER_PIN = 25
LED_ENABLED = True
BUZZER_ENABLED = False

# RPLIDAR
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800
LIDAR_SCAN_TIMEOUT = 2.0

# LIDAR-Einbauposition relativ zum Roboter-Mittelpunkt (x vorn, y links, in m)
# und Drehung der LIDAR-Nullrichtung gegen den Uhrzeigersinn (Grad). Die Werte
# sind nicht vermessen. Sie gehen in die Recording-Metadaten und in die
# Positionsbestimmung (``localization``) ein.
LIDAR_OFFSET_X = 0.15
LIDAR_OFFSET_Y = 0.0
LIDAR_ANGLE_OFFSET = 0.0
# Der RPLIDAR zählt Winkel im Uhrzeigersinn, der Roboter gegen den Uhrzeigersinn.
# Mit ``python -m raspirobot.lidar_check`` prüfen (Hand links vor den Sensor halten).
LIDAR_ANGLE_SIGN = -1
LIDAR_MIN_QUALITY = 3
LIDAR_MIN_RANGE_M = 0.10
LIDAR_MAX_RANGE_M = 10.0
# Wie oft der LIDAR-Thread einen abgebrochenen Scan neu startet, bevor er aufgibt.
LIDAR_MAX_RESTARTS = 5

# Kollisionsschwellen
COLLISION_DISTANCE_CM = 30.0       # Ultraschall
LIDAR_COLLISION_DISTANCE_M = 0.3
LIDAR_WARNING_DISTANCE_M = 0.8     # nur für die Zählung lidar_obstacles_front
# Ausgewerteter LIDAR-Winkelbereich in Grad, so wie die Referenz ihn prüft.
# Der RPLIDAR liefert Winkel von 0 bis 360 Grad, daher greift effektiv nur 0..45.
LIDAR_FRONT_SECTOR_MIN_DEG = -45
LIDAR_FRONT_SECTOR_MAX_DEG = 45

# Xbox-Controller (pygame-Indizes)
CONTROLLER_ID = 0
AXIS_LEFT_TRIGGER = 2
AXIS_RIGHT_TRIGGER = 5
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_START = 7

# Schleifenfrequenzen
SENSOR_UPDATE_HZ = 10
CONTROL_LOOP_HZ = 50
LIDAR_UPDATE_HZ = 10        # nur Anzeige/Metadaten; der LIDAR-Thread läuft im Takt des Sensors
MAX_POSE_UPDATE_HZ = 50

# Recording
RECORDING_SAMPLE_RATE_HZ = 50
MIN_RECORDING_DURATION_SEC = 5.0
RECORDING_BASE_DIR = "path_recordings"  # relativ zum aktuellen Arbeitsverzeichnis
LIDAR_FRAME_BUFFER_SIZE = 100

# Odometrie aus PWM-Werten (keine Rad-Encoder).
WHEEL_BASE_M = 0.20
# Annahme der Referenz: 100 % Duty Cycle entspricht 0,5 m/s. Nicht gemessen.
ODOMETRY_MAX_SPEED_M_S = 0.5

# Shutdown
THREAD_SHUTDOWN_TIMEOUT_SEC = 3.0
LIDAR_STOP_DELAY_SEC = 0.5
LIDAR_MOTOR_STOP_DELAY_SEC = 0.3

# Positionsbestimmung (Scan-Matching gegen eine selbst aufgebaute Karte)
SLAM_ENABLED = True
SLAM_MAP_SIZE_M = 40.0             # Kantenlänge der quadratischen Karte, Start in der Mitte
SLAM_RESOLUTION_M = 0.025
SLAM_LIKELIHOOD_SIGMA_M = 0.06     # Breite der Wahrscheinlichkeitsfunktion um belegte Zellen
SLAM_COARSE_STEP_M = 0.05          # Rasterschritt der groben Suche; drei Verfeinerungen halbieren ihn je Stufe
SLAM_RELOCALIZE_STEP_M = 0.10
SLAM_MIN_POINTS = 60               # weniger gültige Punkte: Scan wird nicht ausgewertet
SLAM_ACCEPT_SCORE = 0.60           # mittlere Trefferwertung (0..1), ab der ein Match gilt
SLAM_RELOCALIZE_SCORE = 0.75       # strengere Schwelle für das Wiederfinden nach Verlust
SLAM_NARROW_XY_M = 0.10            # Suchfenster um die Odometrie-Vorhersage
SLAM_NARROW_THETA_RAD = 0.08
SLAM_WIDE_XY_M = 0.35              # zweiter Versuch, wenn das enge Fenster nicht reicht
SLAM_WIDE_THETA_RAD = 0.35
SLAM_RELOCALIZE_XY_M = 0.8
SLAM_PRIOR_PENALTY_PER_M = 0.05    # bevorzugt bei Gleichstand die Vorhersage
SLAM_PRIOR_PENALTY_PER_RAD = 0.05
SLAM_AMBIGUITY_MARGIN = 0.10       # ein ≥ 2 Schritte entfernter Treffer, der höchstens so viel schlechter ist, macht den Match mehrdeutig
SLAM_LOST_AFTER = 5                # aufeinanderfolgende Fehlmatches bis Status LOST
SLAM_MAP_REBUILD_EVERY = 25        # Scans zwischen vollständigen Neuberechnungen des Likelihood-Felds
