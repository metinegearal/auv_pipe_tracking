"""Nominal discrete DLQR analysis, independent of ROS and controller execution."""

from pathlib import Path

import numpy as np
from scipy.optimize import brentq, minimize_scalar
from scipy.signal import dlsim


def matrix_metrics(a, b, c):
    """Return finite-horizon controllability/observability ranks and singular values."""
    n = len(a)
    controllability = np.hstack([np.linalg.matrix_power(a, i) @ b for i in range(n)])
    observability = np.vstack([c @ np.linalg.matrix_power(a, i) for i in range(n)])
    return {
        name: {'rank': int(np.linalg.matrix_rank(matrix)),
               'singular_values': np.linalg.svd(matrix, compute_uv=False)}
        for name, matrix in [('controllability', controllability),
                             ('observability', observability)]
    }


def step_metrics(t, y, ideal=1.0):
    """Measure a positive unit step, interpolating first threshold crossings."""
    def first_crossing(level):
        hits = np.flatnonzero(y >= level * ideal)
        if not len(hits):
            return None
        i = hits[0]
        if i == 0:
            return float(t[0])
        return float(t[i - 1] + (level * ideal - y[i - 1]) *
                     (t[i] - t[i - 1]) / (y[i] - y[i - 1]))

    t10, t90 = first_crossing(0.1), first_crossing(0.9)
    outside = np.flatnonzero(np.abs(y - ideal) > 0.02 * abs(ideal))
    settling = (float(t[outside[-1] + 1]) if len(outside) and outside[-1] < len(t) - 1
                else (0.0 if not len(outside) else None))
    return {
        'peak': float(np.max(y)), 't63': first_crossing(0.63),
        'ts': settling, 'tr': None if t10 is None or t90 is None else t90 - t10,
    }


def loop_response(a, b, k, theta):
    """Input return ratio K(zI-A)^-1 B at theta in radians/sample."""
    z = np.exp(1j * np.asarray(theta))
    matrix = z[..., None, None] * np.eye(len(a)) - a
    rhs = np.broadcast_to(b, matrix.shape[:-1] + (1,))
    return (k @ np.linalg.solve(matrix, rhs))[..., 0, 0]


def frequency_metrics(a, b, k, dt):
    """Refine crossings and sensitivity peaks, including the Nyquist endpoint."""
    # Combined grids resolve slow poles and the upper discrete frequency band.
    theta = np.unique(np.concatenate((np.geomspace(1e-9, np.pi, 8192),
                                      np.linspace(1e-6, np.pi, 8192))))
    response = loop_response(a, b, k, theta)

    def response_at(value):
        return complex(loop_response(a, b, k, value))

    def roots(values, function):
        indices = np.flatnonzero(values[:-1] * values[1:] < 0)
        result = [brentq(function, theta[i], theta[i + 1], xtol=1e-14)
                  for i in indices]
        result.extend(theta[np.flatnonzero(values == 0)].tolist())
        return result

    gain_crossings = roots(np.abs(response) - 1,
                           lambda value: abs(response_at(value)) - 1)
    phase_candidates = roots(response.imag, lambda value: response_at(value).imag)
    # exp(j*pi) has roundoff imaginary residue; a negative real endpoint is -180 deg.
    if response[-1].real < 0 and abs(response[-1].imag) < 1e-10:
        phase_candidates.append(np.pi)
    phase_crossings = sorted({round(value, 14) for value in phase_candidates
                              if response_at(value).real < 0})
    phase_margins = [
        ((np.angle(response_at(value), deg=True) + 360) % 360 - 180, value / dt)
        for value in gain_crossings]
    gain_margins = [(1 / abs(response_at(value)), value / dt)
                    for value in phase_crossings]

    sensitivity = 1 / np.abs(1 + response)
    candidates = [(float(sensitivity[0]), float(theta[0])),
                  (float(sensitivity[-1]), float(theta[-1]))]
    maxima = np.flatnonzero(
        (sensitivity[1:-1] > sensitivity[:-2]) &
        (sensitivity[1:-1] >= sensitivity[2:])) + 1
    for i in maxima:
        optimum = minimize_scalar(
            lambda value: -1 / abs(1 + response_at(value)),
            bounds=(theta[i - 1], theta[i + 1]), method='bounded',
            options={'xatol': 1e-14})
        candidates.append((-float(optimum.fun), float(optimum.x)))
    # Exact DC value, or the low-frequency limit for a plant integrator.
    try:
        dc = 1 / abs(1 + response_at(0.0))
    except np.linalg.LinAlgError:
        dc = 0.0
    candidates.append((dc, 0.0))
    ms, peak_theta = max(candidates)
    return {
        'omega': theta / dt, 'loop': response, 'sensitivity': sensitivity,
        'pm': min(phase_margins, key=lambda item: abs(item[0])) if phase_margins else None,
        'gm': min(gain_margins, key=lambda item: abs(np.log(item[0])))
        if gain_margins else None,
        'ms': ms, 'ms_omega': peak_theta / dt,
    }


def analyze_model(model):
    """Analyze both modes from the controller's published matrices and gains."""
    if model.get('schema_version') != 1:
        raise ValueError('Unsupported DLQR model schema')
    dt, damping = float(model['dt']), float(model['damping'])
    if not np.isfinite(dt) or dt <= 0 or not np.isfinite(damping) or damping < 0:
        raise ValueError('Invalid sampling period or damping')
    arrays = {}
    for name, shape in [('A', (6, 6)), ('B', (6, 3)),
                        ('K_state', (3, 6)), ('K_velocity', (3, 3))]:
        value = np.asarray(model[name], dtype=float)
        if value.shape != shape or not np.all(np.isfinite(value)):
            raise ValueError(f'Invalid {name} matrix')
        arrays[name] = value
    a, b, ks, kv = [arrays[name] for name in ['A', 'B', 'K_state', 'K_velocity']]
    # This report uses independent SISO return ratios, so reject future coupled models.
    for i in range(3):
        indices = [i, i + 3]
        others = [j for j in range(6) if j not in indices]
        if (np.any(np.abs(a[np.ix_(indices, others)]) > 1e-12) or
                np.any(np.abs(b[others, i]) > 1e-12) or
                np.any(np.abs(ks[i, others]) > 1e-12) or
                np.any(np.abs(np.delete(kv[i], i)) > 1e-12)):
            raise ValueError('Coupled model requires MIMO analysis')

    channels = []
    for mode in ['state', 'velocity']:
        for i, axis in enumerate('XYZ'):
            indices = [i, i + 3] if mode == 'state' else [i]
            aa, bb = a[np.ix_(indices, indices)], b[np.ix_(indices, [i])]
            kk = ks[np.ix_([i], indices)] if mode == 'state' else kv[i:i + 1, i:i + 1]
            closed = aa - bb @ kk
            poles = np.linalg.eigvals(closed).astype(complex)
            stable = bool(np.all(np.abs(poles) < 1))
            with np.errstate(divide='ignore', invalid='ignore'):
                equivalent = np.log(poles) / dt
                zeta = -equivalent.real / np.abs(equivalent)
            # Position step uses Kp; velocity step also uses the node's drag feedforward.
            feedforward = kk[0, -1] + (damping if mode == 'velocity' else 0)
            output = np.zeros((1, len(indices)))
            output[0, -1] = 1
            slow_rate = -np.log(max(np.max(np.abs(poles)), 1e-15)) / dt
            duration = max(30.0, 12 / slow_rate) if stable else 30.0
            count = min(120000, max(2, int(np.ceil(duration / dt)) + 1))
            t = np.arange(count) * dt
            with np.errstate(over='ignore', invalid='ignore'):
                _, values, _ = dlsim((closed, bb * feedforward, output,
                                     np.zeros((1, 1)), dt), np.ones(count), t=t)
            metrics = step_metrics(t, values[:, 0]) if stable else {
                'peak': None, 't63': None, 'ts': None, 'tr': None}
            channels.append({
                'name': f'{mode}/{axis}', 'mode': mode, 'poles': poles, 'zeta': zeta,
                'stable': stable, 'duration': float(t[-1]), 'step': metrics,
                'structure': matrix_metrics(aa, bb, np.eye(len(indices))),
                'frequency': frequency_metrics(aa, bb, kk, dt),
            })
    return {'channels': channels, 'structure': matrix_metrics(a, b, np.eye(6))}


def _number(value):
    return 'N/A' if value is None else f'{value:.6g}'


def format_report(model, analysis):
    """Put all requested text metrics in four sections of one report."""
    lines = [
        'DLQR NOMINAL MODEL EVALUATION',
        '', '1. Model and definitions',
        f"dt={model['dt']:.9g} s; mass={model['mass']:.9g} kg; "
        f"drag={model['damping']:.9g} N/(m/s)",
        f"Configured mode={model['control_mode']}; "
        f"waypoint_cruise_speed={model['waypoint_cruise_speed']:.6g} m/s",
        'Both feedback modes are evaluated independently on X/Y/Z.',
        'state: unit position step (1 m), zero velocity reference; velocity: 1 m/s step.',
        'Nominal linear model: excludes saturation, yaw, sensor/transport delays,',
        'nonlinear hydrodynamics, waypoint shaping and switching between modes.',
        'L(z)=K(zI-A)^-1 B at the force input; S(z)=1/(1+L(z)).',
        'PM is additional phase lag in degrees at |L|=1, not a time in seconds.',
        'GM is 1/|L| at negative-real L (-180 deg modulo 360), including Nyquist.',
        'The limiting margin is the crossing closest to instability.',
        'Frequency search: 1e-9 rad/sample to pi, refined crossings/local peaks plus DC.',
        '', '2. Closed-loop damping and step response',
        'Damping ratios use s=log(z)/dt, zeta=-Re(s)/|s| for each closed-loop pole.',
        'Real positive stable poles have zeta=1; t63 is a first 63% crossing,',
        'not necessarily a single physical time constant for a multi-pole system.',
        'Peak is the largest sampled output over the reported horizon.',
        'ts: first sample staying within +/-2% of ideal=1 through the horizon.',
        'tr: first 10% to first 90% crossings; thresholds use linear interpolation.',
        'N/A: unstable response, threshold not reached, or not settled in horizon.',
    ]
    for channel in analysis['channels']:
        step = channel['step']
        lines.extend([
            f"\n{channel['name']}: stable={channel['stable']}; "
            f"horizon={channel['duration']:.6g} s",
            '  poles(z)=' + ', '.join(f'{p:.7g}' for p in channel['poles']),
            '  damping ratios=' + ', '.join(_number(z) for z in channel['zeta']),
            f"  peak={_number(step['peak'])}; t63={_number(step['t63'])} s; "
            f"ts(2%)={_number(step['ts'])} s; tr(10-90%)={_number(step['tr'])} s",
        ])
    lines.extend([
        '', '3. Controllability and observability',
        'Ctrb=[B AB ... A^(n-1)B]; Obsv=[C; CA; ...; CA^(n-1)].',
        'C=I assumes all modeled states are available from velocity/position sensors.',
        'Velocity mode analyzes only velocity states; position integrators are excluded.',
        'Rank uses NumPy SVD tolerance; singular values depend on units/scaling.',
    ])
    for name, structure, size in [('full plant', analysis['structure'], 6)] + [
            (c['name'], c['structure'], len(c['poles'])) for c in analysis['channels']]:
        for kind, metric in structure.items():
            singular = ', '.join(_number(v) for v in metric['singular_values'])
            lines.append(f"{name} {kind}: rank={metric['rank']}/{size}; "
                         f'singular values=[{singular}]')
    lines.extend(['', '4. Frequency margins and sensitivity peak',
                  'Frequencies are rad/s; Ms is max |S| over DC through Nyquist.',
                  'No phase crossover: GM=inf; no unity-gain crossover: PM=N/A.'])
    for channel in analysis['channels']:
        frequency = channel['frequency']
        pm, gm = frequency['pm'], frequency['gm']
        pm_text = 'N/A (no unity-gain crossover)' if pm is None else (
            f'{pm[0]:.6g} deg at {pm[1]:.6g} rad/s')
        gm_text = 'inf (no -180 deg crossover)' if gm is None else (
            f'{gm[0]:.6g} x / {20 * np.log10(gm[0]):.6g} dB at {gm[1]:.6g} rad/s')
        lines.extend([
            f"{channel['name']}: PM={pm_text}; GM={gm_text}",
            f"  Ms={frequency['ms']:.6g} / {20 * np.log10(frequency['ms']):.6g} dB "
            f"at {frequency['ms_omega']:.6g} rad/s",
        ])
    return '\n'.join(lines) + '\n'


def write_outputs(model, output_dir):
    """Write a single text report and one page with sensitivity and Nichols plots."""
    # Agg canvas is safe for headless ROS launches; do not change global pyplot backend.
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    analysis = analyze_model(model)
    directory = Path(output_dir).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    report_path = directory / 'dlqr_features.txt'
    plot_path = directory / 'dlqr_sensitivity_nichols.png'
    figure = Figure(figsize=(14, 9), layout='constrained')
    FigureCanvasAgg(figure)
    axes = figure.subplots(2, 2)
    for row, mode in enumerate(['state', 'velocity']):
        sensitivity_axis, nichols_axis = axes[row]
        channels = [c for c in analysis['channels'] if c['mode'] == mode]
        rates = np.abs(np.log(np.concatenate([c['poles'] for c in channels]))) / model['dt']
        finite_rates = rates[np.isfinite(rates) & (rates > 0)]
        plot_start = min(finite_rates) / 100 if len(finite_rates) else 0
        for index, channel in enumerate(channels):
            frequency = channel['frequency']
            style = ['-', '--', ':'][index]
            visible = frequency['omega'] >= plot_start
            sensitivity_axis.semilogx(
                frequency['omega'][visible],
                20 * np.log10(frequency['sensitivity'][visible]),
                style, label=f"{channel['name'][-1]} (Ms={frequency['ms']:.5g})")
            loop = frequency['loop'][visible]
            nichols_axis.plot(np.unwrap(np.angle(loop)) * 180 / np.pi,
                              20 * np.log10(np.abs(loop)), style,
                              label=channel['name'][-1])
        sensitivity_axis.set(title=f'{mode.capitalize()} feedback: sensitivity',
                             xlabel='Frequency (rad/s)', ylabel='20 log10 |S| (dB)')
        sensitivity_axis.axhline(0, color='gray', linewidth=0.7)
        nichols_axis.set(title=f'{mode.capitalize()} feedback: Nichols',
                         xlabel='Open-loop phase (deg)', ylabel='Open-loop gain (dB)')
        nichols_axis.plot(-180, 0, 'rx', label='Critical point (-180°, 0 dB)')
        nichols_axis.axvline(-180, color='gray', linewidth=0.7)
        nichols_axis.axhline(0, color='gray', linewidth=0.7)
        for axis in axes[row]:
            axis.grid(True, which='both', alpha=0.25)
            axis.legend(fontsize=8)
    figure.suptitle(
        'DLQR nominal model | independent X/Y/Z force-input loops\n'
        f"dt={model['dt']:.6g} s; overlapping axis curves use different line styles")
    figure.savefig(plot_path, dpi=160, bbox_inches='tight')
    report_path.write_text(format_report(model, analysis), encoding='utf-8')
    return report_path, plot_path
