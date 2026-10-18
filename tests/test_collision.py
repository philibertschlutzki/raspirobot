from raspirobot.collision import analyze_collision


def test_no_points_reports_100m():
    status = analyze_collision(100.0, 100.0, [])
    assert status.lidar_min_distance == 100.0
    assert not status.collision_detected


def test_lidar_thresholds():
    assert analyze_collision(100, 100, [(10, 10.0, 250.0)]).collision_detected
    assert not analyze_collision(100, 100, [(10, 10.0, 250.0)]).collision_critical
    assert analyze_collision(100, 100, [(10, 10.0, 140.0)]).collision_critical


def test_obstacle_count_uses_warning_distance():
    scan = [(10, 5.0, 500.0), (10, 6.0, 900.0), (10, 7.0, 790.0)]
    assert analyze_collision(100, 100, scan).lidar_obstacles_front == 2


def test_sector_only_covers_0_to_45_degrees():
    # Dokumentiert das Verhalten der Referenz: Punkte bei 315-360 Grad
    # (also links vorne bzw. rechts vorne, je nach Einbau) werden ignoriert.
    assert analyze_collision(100, 100, [(10, 20.0, 100.0)]).collision_critical
    assert not analyze_collision(100, 100, [(10, 340.0, 100.0)]).collision_detected
