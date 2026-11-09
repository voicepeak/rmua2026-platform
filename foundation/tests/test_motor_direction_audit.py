from foundation.audit_motor_directions import RUNS, analyze, candidate_signs


def test_differential_runs_match_candidate_xy_signs():
    expected = candidate_signs()
    for run in RUNS:
        result = analyze(run)
        assert len(result['observed_signs']) == 4
        assert all(obs[:2] == exp[:2] for obs, exp in zip(result['observed_signs'], expected))
        assert result['observed_signs'] == analyze(RUNS[0])['observed_signs']
