"""YAML-driven deterministic headless experiments and portable reports."""

import argparse
import copy
import csv
import hashlib
import itertools
import json
from pathlib import Path
import platform
import tempfile
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

from massage_mujoco.contact import ContactMonitor
from massage_mujoco.model import default_config_path
from massage_mujoco.runtime import ControlMode, MujocoRuntime


def _merge(target, source):
    for key, value in source.items():
        if isinstance(value, dict):
            _merge(target[key], value)
        else:
            if key not in target:
                raise ValueError(f'unknown model configuration: {key}')
            target[key] = value


def _variants(scenario):
    sweep = scenario.get('sweep', {})
    if any(not values for values in sweep.values()):
        raise ValueError('sweep values must not be empty')
    for values in itertools.product(*sweep.values()):
        yield dict(zip(sweep, values))


def _configured_runtime(base, scenario, parameters):
    config = copy.deepcopy(base)
    _merge(config, scenario.get('model_overrides', {}))
    for path, value in parameters.items():
        parts, branch = path.split('.'), config
        for key in parts[:-1]:
            branch = branch[key]
        _merge(branch, {parts[-1]: value})
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'model.yaml'
        path.write_text(yaml.safe_dump(config))
        runtime = MujocoRuntime(path)
    return runtime, config


def _steps(value, timestep, label):
    if not np.isfinite(value):
        raise ValueError(f'{label} must be finite')
    count = round(float(value) / timestep)
    if count < 0 or abs(count * timestep - value) > 1e-9:
        raise ValueError(f'{label} must be a nonnegative multiple of timestep')
    return count


def _row(state, contact):
    row = {'time': state.time, 'contact_force': contact['normal_force'],
           'pad_position': state.contact_pad_position,
           'penetration': max((p['penetration'] for p in contact['contacts']), default=0.0)}
    for name, values in (
        ('actual', state.position), ('target', state.target_position),
        ('error', state.target_position - state.position),
        ('velocity', state.velocity), ('effort', state.actuator_effort),
        ('ft', state.tool_wrench), ('tcp', state.tool_position),
    ):
        row.update({f'{name}_{i + 1}': float(value) for i, value in enumerate(values)})
    return row


def run_case(runtime, scenario, monitor=True):
    """Execute per-physics-step targets and sample diagnostics at a fixed cadence."""
    dt = runtime.config.timestep
    count = _steps(scenario['duration'], dt, 'duration')
    stride = _steps(scenario.get('sample_period', 0.01), dt, 'sample_period')
    if count < 1 or stride < 1:
        raise ValueError('duration and sample_period must be positive')
    initial = runtime.reset(scenario.get('initial_positions')).position
    mode = ControlMode(scenario.get('control_mode', 'POSITION'))
    runtime.set_control_mode(mode)
    times, positions = [0], [initial]
    for point in scenario.get('trajectory', []):
        index = _steps(point['time'], dt, 'trajectory time')
        if index <= times[-1] or index > count:
            raise ValueError('trajectory times must increase within duration')
        if ('positions' in point) == ('delta' in point):
            raise ValueError('trajectory point needs exactly one of positions or delta')
        target = point.get('positions')
        if target is None:
            target = initial + np.asarray(point['delta'], dtype=float)
        positions.append(runtime.validate_positions(target))
        times.append(index)
    if mode is not ControlMode.POSITION and len(times) > 1:
        raise ValueError('trajectory requires POSITION mode')
    loads = []
    for load in scenario.get('external_wrenches', []):
        start = _steps(load['start'], dt, 'wrench start')
        end = _steps(load['end'], dt, 'wrench end')
        if not start < end <= count:
            raise ValueError('wrench interval must lie within duration')
        loads.append((start, end,
                      runtime._wrench_vector(load['force'], 'force'),
                      runtime._wrench_vector(load.get('torque', [0, 0, 0]), 'torque')))
    detector = ContactMonitor(runtime.config.contact_monitor)
    rows, events = [], []
    unexpected_samples = 0
    segment = 1
    started = time.perf_counter()
    for index in range(count + 1):
        if index % stride == 0 or index == count:
            state = runtime.state()
            contact = (detector.update(state.time, runtime.contacts()) if monitor else dict(
                normal_force=0.0, contacts=[], unexpected_contact=False, events=[],
            ))
            rows.append(_row(state, contact))
            unexpected_samples += int(contact['unexpected_contact'])
            events.extend({'time': state.time, 'event': event} for event in contact['events'])
        if index == count:
            break
        while segment < len(times) - 1 and index + 1 > times[segment]:
            segment += 1
        if len(times) > 1:
            alpha = min(1.0, (index + 1 - times[segment - 1])
                        / (times[segment] - times[segment - 1]))
            runtime.set_target(positions[segment - 1]
                               + alpha * (positions[segment] - positions[segment - 1]))
        if loads:
            force, torque = np.zeros(3), np.zeros(3)
            for start, end, f, t in loads:
                if start <= index < end:
                    force += f
                    torque += t
            # Reapply in current sensor axes, not the orientation at load onset.
            runtime.set_tool_external_wrench(force, torque)
        runtime._physics_step()
    elapsed = time.perf_counter() - started
    runtime.clear_tool_external_wrench()
    values = np.asarray([list(row.values()) for row in rows])
    if not np.isfinite(values).all():
        raise RuntimeError('simulation produced non-finite telemetry')
    if abs(rows[-1]['time'] - scenario['duration']) > 1e-7:
        raise RuntimeError('simulation clock reset or diverged')
    errors = np.asarray([[row[f'error_{i}'] for i in range(1, 7)] for row in rows])
    metrics = {
        'final_error': float(np.max(np.abs(errors[-1]))),
        'peak_error': float(np.max(np.abs(errors))),
        'rms_error': float(np.sqrt(np.mean(errors ** 2))),
        'peak_contact_force': max(row['contact_force'] for row in rows),
        'final_contact_force': rows[-1]['contact_force'],
        'max_penetration': max(row['penetration'] for row in rows),
        'max_effort': max(abs(row[f'effort_{i}']) for row in rows for i in range(1, 7)),
        'peak_ft_force': max(float(np.linalg.norm([row[f'ft_{i}'] for i in range(1, 4)]))
                             for row in rows),
        'unexpected_contact_samples': unexpected_samples,
    }
    return dict(metrics=metrics, rows=rows, events=events, wall_seconds=elapsed,
                realtime_factor=scenario['duration'] / elapsed)


def _assertions(result, assertions):
    failures = []
    for metric, limits in assertions.items():
        value = result['metrics'][metric]
        if 'min' in limits and value < limits['min']:
            failures.append(f'{metric}={value:.9g} < {limits["min"]}')
        if 'max' in limits and value > limits['max']:
            failures.append(f'{metric}={value:.9g} > {limits["max"]}')
    return failures


def _write_csv(path, rows):
    if not rows:
        return
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run_suite(suite, output, config_path=None):
    """Run repeats and Cartesian parameter sweeps; retain failed-case evidence."""
    if suite.get('version') != 1 or not suite.get('scenarios'):
        raise ValueError('suite requires version: 1 and nonempty scenarios')
    names = [scenario['name'] for scenario in suite['scenarios']]
    if len(set(names)) != len(names):
        raise ValueError('scenario names must be unique')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    base = yaml.safe_load(Path(config_path or default_config_path()).read_text())
    report = dict(schema_version=1, backend='mujoco', mujoco_version=mujoco.__version__,
                  machine=platform.machine(), suite=suite, model_config=base, cases=[])
    for scenario in suite['scenarios']:
        for variant_index, parameters in enumerate(_variants(scenario)):
            reference = None
            repeats = int(scenario.get('repeat', suite.get('repeat', 2)))
            if repeats < 1:
                raise ValueError('repeat must be positive')
            for repeat in range(repeats):
                case_id = f'{scenario["name"]}[{variant_index}]#{repeat + 1}'
                case = dict(case_id=case_id, parameters=parameters, failures=[])
                try:
                    runtime, config = _configured_runtime(base, scenario, parameters)
                    result = run_case(runtime, scenario)
                    rows = result.pop('rows')
                    array = np.asarray([list(row.values()) for row in rows])
                    difference = 0.0 if reference is None else float(
                        np.max(np.abs(array - reference)))
                    reference = array if reference is None else reference
                    result['metrics']['determinism_max_abs'] = difference
                    case.update(result)
                    case['model_config'] = config
                    case['failures'] = _assertions(case, scenario.get('assertions', {}))
                    seen = {item['event'] for item in case['events']}
                    for event in scenario.get('required_events', []):
                        if event not in seen:
                            case['failures'].append(f'missing event {event}')
                    if difference > suite.get('determinism_tolerance', 1e-10):
                        case['failures'].append(f'repeat differs by {difference:.9g}')
                    stem = hashlib.sha256(case_id.encode()).hexdigest()[:12]
                    case['trace'] = f'{stem}.csv'
                    _write_csv(output / case['trace'], rows)
                except (ValueError, KeyError, TypeError, RuntimeError) as error:
                    case['failures'].append(f'{type(error).__name__}: {error}')
                case['passed'] = not case['failures']
                report['cases'].append(case)
    report['passed'] = all(case['passed'] for case in report['cases'])
    (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    root = ET.Element('testsuite', name='mujoco_regression',
                      tests=str(len(report['cases'])),
                      failures=str(sum(not c['passed'] for c in report['cases'])))
    for case in report['cases']:
        element = ET.SubElement(root, 'testcase', name=case['case_id'],
                                time=str(case.get('wall_seconds', 0)))
        if not case['passed']:
            ET.SubElement(element, 'failure').text = '\n'.join(case['failures'])
    ET.ElementTree(root).write(output / 'junit.xml', encoding='utf-8', xml_declaration=True)
    return report


def compare_reports(reference, candidate, tolerances):
    """Compare matching case IDs using explicit absolute metric tolerances."""
    if not tolerances:
        raise ValueError('comparison requires explicit metric tolerances')
    if not reference['cases'] or not candidate['cases']:
        return dict(passed=False, failures=['reports must contain cases'], differences=[])
    before = {case['case_id']: case for case in reference['cases']}
    after = {case['case_id']: case for case in candidate['cases']}
    failures, differences = [], []
    if before.keys() != after.keys():
        failures.append('case IDs differ between reports')
    for case_id in sorted(before.keys() & after.keys()):
        if before[case_id].get('parameters', {}) != after[case_id].get('parameters', {}):
            failures.append(f'{case_id}: experiment parameters differ')
        if not before[case_id]['passed'] or not after[case_id]['passed']:
            failures.append(f'{case_id}: cannot accept a failed experiment')
        for metric, tolerance in tolerances.items():
            if not np.isfinite(tolerance) or tolerance < 0:
                raise ValueError('comparison tolerances must be finite and nonnegative')
            try:
                delta = abs(after[case_id]['metrics'][metric]
                            - before[case_id]['metrics'][metric])
                differences.append(dict(case_id=case_id, metric=metric, delta=delta))
                if not np.isfinite(delta) or delta > tolerance:
                    failures.append(f'{case_id}: {metric} difference {delta} > {tolerance}')
            except KeyError:
                failures.append(f'{case_id}: missing metric {metric}')
    return dict(passed=not failures, reference_backend=reference['backend'],
                candidate_backend=candidate['backend'], failures=failures,
                differences=differences)


def benchmark(suite, repetitions=5):
    """Pair identical core runs with/without contact extraction, excluding GUI/ROS."""
    results = []
    base = yaml.safe_load(default_config_path().read_text())
    experiments = [(s, p) for s in suite['scenarios'] for p in _variants(s)]
    for scenario, parameters in experiments:
        runtime, config = _configured_runtime(base, scenario, parameters)
        run_case(runtime, scenario)  # Warm model, Python paths and contact solver.
        samples = {False: [], True: []}
        for repeat in range(repetitions):
            for enabled in ((False, True) if repeat % 2 == 0 else (True, False)):
                samples[enabled].append(run_case(runtime, scenario, enabled)['wall_seconds'])
        baseline = float(np.median(samples[False]))
        monitored = float(np.median(samples[True]))
        degradation = max(0.0, 1.0 - baseline / monitored)
        results.append(dict(
            scenario=scenario['name'], baseline_seconds=baseline,
            parameters=parameters, model_config=config,
            monitored_seconds=monitored, speed_degradation=degradation,
            monitored_realtime_factor=scenario['duration'] / monitored,
            passed=degradation <= 0.2, samples_seconds={str(k): v for k, v in samples.items()},
        ))
    return dict(passed=all(item['passed'] for item in results), cases=results,
                scope='core plus sampled CSV telemetry; contact extraction on/off; no ROS or GUI',
                mujoco_version=mujoco.__version__, machine=platform.machine())


def main(args=None):
    """Run a suite or compare two reports; return nonzero on failed acceptance."""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    run = sub.add_parser('run')
    run.add_argument('suite', type=Path)
    run.add_argument('--output', type=Path, required=True)
    run.add_argument('--config', type=Path)
    bench = sub.add_parser('benchmark')
    bench.add_argument('suite', type=Path)
    bench.add_argument('--output', type=Path, required=True)
    compare = sub.add_parser('compare')
    compare.add_argument('reference', type=Path)
    compare.add_argument('candidate', type=Path)
    compare.add_argument('--tolerances', type=Path, required=True)
    compare.add_argument('--output', type=Path, required=True)
    options = parser.parse_args(args)
    try:
        if options.command == 'run':
            result = run_suite(yaml.safe_load(options.suite.read_text()),
                               options.output, options.config)
        elif options.command == 'benchmark':
            result = benchmark(yaml.safe_load(options.suite.read_text()))
            options.output.parent.mkdir(parents=True, exist_ok=True)
            options.output.write_text(json.dumps(result, indent=2, allow_nan=False))
        else:
            result = compare_reports(
                json.loads(options.reference.read_text()),
                json.loads(options.candidate.read_text()),
                yaml.safe_load(options.tolerances.read_text()))
            options.output.parent.mkdir(parents=True, exist_ok=True)
            options.output.write_text(json.dumps(result, indent=2, allow_nan=False))
        print(json.dumps({'passed': result['passed'], 'output': str(options.output)}))
        return 0 if result['passed'] else 1
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        parser.exit(2, f'{error}\n')
