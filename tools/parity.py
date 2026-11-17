"""Compare the exact archived hardware allocator with the C++ implementation."""
from pathlib import Path
import importlib.util
import json
import subprocess
import numpy as np

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('hardware_allocator', root / 'reference/thruster_solver.py')
original = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = original
spec.loader.exec_module(original)
rng = np.random.default_rng(20261102)
samples = []
for i in range(150):
    angles = rng.uniform(-np.pi / 2, np.pi / 2, 2)
    if i < 3:
        angles = np.array([(0, np.pi/2, -np.pi/2)[i]] * 2)
    target = rng.uniform(-1, 1, 6) * [60, 20, 30, 5, 5, 5]
    ref = rng.uniform(-24.5, 33, 4)
    step = 0.2 if i % 2 else -1.0
    samples.append((angles, target, ref, step))
payload = ''.join(' '.join(map(str, [*a, s, *t, *r])) + '\n' for a,t,r,s in samples)
exe = root / 'build/uw_robot_control/parity_cli'
output = subprocess.run([str(exe)], input=payload, text=True, capture_output=True, check=True)
rows = np.loadtxt(output.stdout.splitlines())
force_error = 0.0
matrix_error = 0.0
for row, (a,t,r,s) in zip(rows, samples):
    expected = original.MuJoCoAllocator(None).solve_fixed_angles(*a,t,reference_forces=r,
                                                               force_step=s if s > 0 else None)
    force_error = max(force_error, float(np.max(np.abs(row[:4] - expected.forces()))))
    matrix_error = max(matrix_error, float(np.max(np.abs(row[4:].reshape(6,4) - original.wrench_matrix(*a)))))
assert force_error < 1e-6, force_error
assert matrix_error < 1e-12, matrix_error
result = dict(samples=len(samples), max_force_error_n=force_error,
              max_geometry_matrix_error=matrix_error,
              scope='fixed-angle allocation and geometry only; nonlinear planner/hardware not certified')
(root / 'validation').mkdir(exist_ok=True)
(root / 'validation/parity.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
