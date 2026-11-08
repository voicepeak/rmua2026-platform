import pytest

from foundation.analyze_ros_clocks import summarize


def test_clock_summary_distinguishes_missing_and_running_clock():
    rows = [{'topic': 'clock', 'arrival_monotonic_ns': t, 'arrival_wall_ns': t + 1000,
             'stamp_ns': t + 500} for t in (300_000_000, 400_000_000, 500_000_000)]
    result = summarize(rows, 'clock', 0)
    assert result['sample_count'] == 3
    assert result['source_over_host_elapsed_ratio_window'] == pytest.approx(1.)
    assert result['arrival_wall_minus_source_ms_median'] == pytest.approx(.0005)
    missing = summarize(rows, 'gps', 0)
    assert missing['sample_count'] == 0
    assert missing['source_over_host_elapsed_ratio_window'] is None
