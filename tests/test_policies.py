import numpy as np

from rmua.policies.action import DEFAULT_PWM_ACTION, DEFAULT_VEL_ACTION
from rmua.policies.obs import DEFAULT_OBS_SPEC
from rmua.policies.safety import FirstSignalLock, SafetyLayer
from rmua.eval.metrics import summarize


def test_action_clip():
    action = np.array([[99.0, -99.0, 0.0, 99.0]])
    clipped = DEFAULT_VEL_ACTION.clip(action)
    assert clipped[0, 0] == 12.0
    assert clipped[0, 1] == -12.0
    assert clipped[0, 3] == 2.5
    pwm = DEFAULT_PWM_ACTION.clip(np.array([[1.5, -0.2, 0.5, 0.5]]))
    assert np.allclose(pwm, [[1.0, 0.0, 0.5, 0.5]])


def test_obs_spec():
    obs = np.zeros((4, DEFAULT_OBS_SPEC.dim), dtype=np.float32)
    assert DEFAULT_OBS_SPEC.validate(obs).shape == obs.shape


def test_first_signal_lock():
    lock = FirstSignalLock()
    assert lock.accept("pwm")
    assert not lock.accept("vel")
    lock.reset()
    assert lock.accept("vel")


def test_safety_layer():
    safety = SafetyLayer()
    decision = safety.filter(np.array([1.0, 0.0, 0.0, 0.0]), {"force_time": 3.0})
    assert not decision.allow
    decision = safety.filter(np.array([1.0, 0.0, 0.0, 0.0]), {})
    assert decision.allow


def test_metrics_summary():
    info = {
        "success": np.array([True, False, False]),
        "gates_passed": np.array([12, 5, 0]),
        "segment_time": np.array([100.0, 180.0, 30.0]),
        "failure_reason": ["success", "collision", "timeout"],
    }
    summary = summarize(info)
    assert summary["episodes"] == 3
    assert abs(summary["success_rate"] - 1 / 3) < 1e-9
    assert summary["failure_counts"]["collision"] == 1
