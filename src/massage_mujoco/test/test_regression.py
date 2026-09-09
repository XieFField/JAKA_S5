"""Regression acceptance, determinism and portable comparison failure tests."""

import copy
import csv
import json
import xml.etree.ElementTree as ET

import numpy as np

from massage_mujoco.regression import compare_reports, main, run_suite


def test_repeats_sweeps_and_failed_assertions_produce_reports(tmp_path):
    suite = dict(version=1, repeat=2, scenarios=[dict(
        name='hold', duration=0.1, sweep={'collision_geometry': ['mesh', 'box']},
        assertions={'final_error': {'max': 0.001}},
    )])
    result = run_suite(suite, tmp_path)
    assert result['passed'] and len(result['cases']) == 4
    assert all(c['metrics']['determinism_max_abs'] == 0 for c in result['cases'])
    assert json.loads((tmp_path / 'report.json').read_text())['passed']
    assert ET.parse(tmp_path / 'junit.xml').getroot().attrib['failures'] == '0'
    with (tmp_path / result['cases'][0]['trace']).open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 11 and abs(float(rows[-1]['time']) - 0.1) < 1e-12
    suite['scenarios'][0]['assertions'] = {'final_contact_force': {'min': 1.0}}
    failed = run_suite(suite, tmp_path / 'failed')
    assert not failed['passed']
    assert all(case['failures'] for case in failed['cases'])


def test_external_wrench_and_trajectory_change_recorded_state(tmp_path):
    scenario = dict(name='motion', duration=0.5,
                    trajectory=[dict(time=0.2, delta=[0.01, 0, 0, 0, 0, 0])])
    unloaded = run_suite(dict(version=1, repeat=1, scenarios=[scenario]), tmp_path / 'a')
    scenario['external_wrenches'] = [dict(start=0.1, end=0.4, force=[0, 0, 2])]
    loaded = run_suite(dict(version=1, repeat=1, scenarios=[scenario]), tmp_path / 'b')
    assert loaded['passed'] and unloaded['passed']
    a = np.genfromtxt(tmp_path / 'a' / unloaded['cases'][0]['trace'], names=True, delimiter=',')
    b = np.genfromtxt(tmp_path / 'b' / loaded['cases'][0]['trace'], names=True, delimiter=',')
    assert a['target_1'][-1] > a['target_1'][0] + 0.009
    assert np.max(np.abs(a['ft_3'] - b['ft_3'])) > 1.0


def test_comparison_detects_missing_cases_and_metric_differences():
    report = dict(backend='gazebo', cases=[dict(
        case_id='contact[0]#1', passed=True, metrics={'final_contact_force': 0.2},
    )])
    candidate = copy.deepcopy(report)
    candidate['backend'] = 'mujoco'
    tolerances = {'final_contact_force': 0.05}
    assert compare_reports(report, candidate, tolerances)['passed']
    candidate['cases'][0]['metrics']['final_contact_force'] = 0.3
    assert not compare_reports(report, candidate, tolerances)['passed']
    candidate['cases'].clear()
    assert not compare_reports(report, candidate, tolerances)['passed']


def test_invalid_trajectory_is_reported_and_cli_returns_failure(tmp_path):
    path = tmp_path / 'invalid.yaml'
    path.write_text('''version: 1
repeat: 1
scenarios:
  - name: invalid
    duration: 0.1
    trajectory:
      - time: 0.2
        delta: [0, 0, 0, 0, 0, 0]
''')
    assert main(['run', str(path), '--output', str(tmp_path / 'out')]) == 1
    report = json.loads((tmp_path / 'out/report.json').read_text())
    assert 'trajectory times' in report['cases'][0]['failures'][0]
