from rmua.rules.model import load_rules


def test_rules_schema_and_official_numbers():
    rules = load_rules()
    assert rules.version
    assert len(rules.segments) == 6
    assert [s.name for s in rules.segments] == ["1-3", "3-5", "5-8", "8-12", "12-10", "10-7"]
    assert [s.time_limit_s for s in rules.segments] == [180, 180, 180, 200, 220, 200]
    assert [s.wind for s in rules.segments] == [False, False, False, True, True, True]
    assert [s.factory_reports for s in rules.segments] == [0, 0, 0, 1, 2, 2]
    assert sum(len(s.gates) for s in rules.segments) == 12
    assert rules.course["start_timeout_s"] == 30.0
    assert rules.course["force_field_limit_s"] == 2.0
    assert rules.course["factory_devices"] == 20
    assert tuple(rules.course["factory_meter_range"]) == (0, 50)
    assert rules.control["max_hz"] == 100
    assert rules.sensors["imu_hz"] == 100
    assert rules.sensors["lidar_points_per_frame"] == 20000
    assert rules.sensors["camera_hz"] == 20
    assert rules.deployment["docker_max_gb"] == 15
    assert rules.deployment["submissions_per_day"] == 10


def test_wind_only_after_marker_8():
    rules = load_rules()
    for segment in rules.segments:
        assert segment.wind == (segment.start_marker >= 8)


def test_factory_prerequisite_schedule():
    rules = load_rules()
    assert rules.segment_by_id[4].factory_reports == 1
    assert rules.segment_by_id[5].factory_reports == 2
    assert rules.segment_by_id[6].factory_reports == 2
