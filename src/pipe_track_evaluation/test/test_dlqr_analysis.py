"""Numerical regression tests using analytical responses and independent DARE gains."""

import numpy as np
from pipe_track_evaluation.dlqr_analysis import (
    analyze_model, frequency_metrics, matrix_metrics, step_metrics, write_outputs,
)
import pytest
from scipy.linalg import expm, solve_discrete_are


def nominal_model():
    """Discretize an independent continuous plant with a matrix exponential."""
    dt, mass, damping = 1 / 30, 31.02, 3.0
    continuous = np.zeros((9, 9))
    continuous[:3, :3] = -damping / mass * np.eye(3)
    continuous[3:6, :3] = np.eye(3)
    continuous[:3, 6:] = np.eye(3) / mass
    discrete = expm(continuous * dt)
    a, b = discrete[:6, :6], discrete[:6, 6:]
    r = 0.1 * np.eye(3)
    p = solve_discrete_are(a, b, np.diag([80, 80, 80, 16, 16, 16]), r)
    ks = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
    av, bv = a[:3, :3], b[:3]
    pv = solve_discrete_are(av, bv, 10 * np.eye(3), r)
    kv = np.linalg.solve(r + bv.T @ pv @ bv, bv.T @ pv @ av)
    return {'schema_version': 1, 'dt': dt, 'mass': mass, 'damping': damping,
            'control_mode': 'state', 'waypoint_cruise_speed': 1.0,
            'A': a.tolist(), 'B': b.tolist(),
            'K_state': ks.tolist(), 'K_velocity': kv.tolist()}


def test_first_order_step_metrics():
    t = np.arange(0, 20, 0.001)
    metrics = step_metrics(t, 1 - np.exp(-t / 2))
    assert metrics['t63'] == pytest.approx(-2 * np.log(0.37), abs=0.001)
    assert metrics['tr'] == pytest.approx(2 * np.log(9), abs=0.001)
    assert metrics['ts'] == pytest.approx(-2 * np.log(0.02), abs=0.001)
    assert metrics['peak'] == pytest.approx(1, abs=0.0001)


def test_unsettled_and_recrossing():
    metrics = step_metrics(np.arange(5), np.array([0, 0.5, 1.0, 1.04, 1.01]))
    assert metrics['ts'] == 4
    assert metrics['peak'] == 1.04
    short = step_metrics(np.arange(3), np.array([0.0, 0.1, 0.2]))
    assert short['ts'] is None
    assert short['tr'] is None
    assert short['t63'] is None


def test_integrator_margins_include_nyquist():
    # L(z)=0.5/(z-1); |L|=1 at cos(theta)=7/8, L(-1)=-1/4.
    result = frequency_metrics(np.array([[1.0]]), np.array([[1.0]]),
                               np.array([[0.5]]), 0.1)
    theta = np.arccos(7 / 8)
    assert result['pm'][0] == pytest.approx(90 - np.degrees(theta) / 2)
    assert result['pm'][1] == pytest.approx(theta / 0.1)
    assert result['gm'][0] == pytest.approx(4.0)
    assert result['gm'][1] == pytest.approx(np.pi / 0.1)
    assert result['ms'] == pytest.approx(4 / 3)


def test_no_unity_gain_crossover():
    result = frequency_metrics(np.array([[0.5]]), np.array([[0.1]]),
                               np.array([[1.0]]), 1.0)
    assert result['pm'] is None
    assert result['gm'][0] == pytest.approx(15)


def test_unobservable_uncontrollable_states():
    result = matrix_metrics(np.diag([0.5, 0.8]), np.array([[1.0], [0.0]]),
                            np.array([[1.0, 0.0]]))
    assert result['controllability']['rank'] == 1
    assert result['observability']['rank'] == 1


def test_nominal_modes_and_feedforward():
    model = nominal_model()
    result = analyze_model(model)
    assert result['structure']['controllability']['rank'] == 6
    assert result['structure']['observability']['rank'] == 6
    for channel in result['channels']:
        assert channel['stable']
        assert channel['step']['ts'] is not None
        assert channel['step']['peak'] == pytest.approx(1.0, abs=0.01)
    velocity = result['channels'][3]
    pole = velocity['poles'][0].real
    tau = -model['dt'] / np.log(pole)
    assert velocity['step']['t63'] == pytest.approx(-tau * np.log(0.37), abs=model['dt'])
    assert velocity['step']['tr'] == pytest.approx(tau * np.log(9), abs=model['dt'])


@pytest.mark.parametrize('fault', ['coupling', 'shape', 'nan', 'dt'])
def test_reject_invalid_models(fault):
    model = nominal_model()
    if fault == 'coupling':
        model['A'][0][1] = 0.1
    elif fault == 'shape':
        model['B'] = [[1]]
    elif fault == 'nan':
        model['K_state'][0][0] = float('nan')
    else:
        model['dt'] = 0
    with pytest.raises(ValueError):
        analyze_model(model)


def test_report_and_plot(tmp_path):
    report, plot = write_outputs(nominal_model(), tmp_path)
    text = report.read_text()
    for requested in ['damping ratios=', 'peak=', 't63=', 'ts(2%)=', 'tr(10-90%)=',
                      'controllability:', 'observability:', 'PM=', 'GM=', 'Ms=']:
        assert requested in text
    assert plot.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
