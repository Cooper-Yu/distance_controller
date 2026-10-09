import os
import json
import subprocess
import time
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '167'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
root = Path(os.environ.get('FIXTURE_OUTPUT_ROOT', '/tmp/distance_controller_fixture'))
out = root / 'runtime_logs' / ('right_wall_parameters_' + time.strftime('%Y%m%d_%H%M%S'))
out.mkdir(parents=True)
prefix = subprocess.check_output(
    ['ros2', 'pkg', 'prefix', 'distance_controller'], text=True
).strip()
exe = str(Path(prefix) / 'lib/distance_controller/distance_controller')
names = [
    'wall_heading_half_angle',
    'wall_heading_min_span',
    'wall_heading_max_rms',
    'rear_target_distance',
    'rear_min_distance',
    'rear_window_half_angle',
    'centering_gain',
    'centering_max_speed',
    'centering_tolerance',
    'side_window_half_angle',
    'side_min_distance',
    'side_max_distance',
    'side_max_mad',
    'scan_timeout',
    'preparation_timeout',
    'preparation_max_travel',
]
cases = [(n, v) for n in names for v in ['0.0', '.nan']] + [
    ('wall_heading_half_angle', '0.9'),
    ('side_window_half_angle', '0.9'),
    ('side_min_distance', '2.0'),
    ('start_x', '0.0'),
    ('start_y', '0.0'),
    ('rear_target_distance', '0.22'),
    ('rear_target_distance', '1.5'),
    ('rear_window_half_angle', '0.9'),
]
results = []
for n, v in cases:
    r = subprocess.run(
        [exe, '2', '--ros-args', '-p', n + ':=' + v], capture_output=True, text=True, timeout=5
    )
    assert r.returncode == 1, (n, v, r.stdout, r.stderr)
    results.append({'parameter': n, 'value': v, 'rejected': True})
(out / 'results.json').write_text(json.dumps(results, indent=2))
print(json.dumps({'out': str(out), 'passed': len(results)}))
