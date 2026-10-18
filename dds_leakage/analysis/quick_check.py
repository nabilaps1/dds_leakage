"""
Quick check of one robot in one run: summary of the CSVs and x-y plot of the ground truth.

Usage: python3 quick_check.py data/<condition>/run_NN/<robot>
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

INK = '#0b0b0b'
INK_MUTED = '#52514e'
GRID = '#e4e3df'
SERIES_1 = '#2a78d6'

run = Path(sys.argv[1]).expanduser()
obs = pd.read_csv(run / 'observed_data.csv')
gt = pd.read_csv(run / 'ground_truth.csv')

dt = obs.t.diff().dropna()
print(f'observed_data.csv: {len(obs)} messages, t = {obs.t.iloc[0]:.3f} .. {obs.t.iloc[-1]:.3f} s')
print(f'  interval between messages: mean {dt.mean():.4f} s, min {dt.min():.4f} s, '
      f'max {dt.max():.4f} s')
print(f'  commands with v > 0: {(obs.linear_x > 0).sum()}  |  w > 0: {(obs.angular_z > 0).sum()}'
      f'  |  w < 0: {(obs.angular_z < 0).sum()}')

duration = gt.t.iloc[-1] - gt.t.iloc[0]
print(f'ground_truth.csv: {len(gt)} poses, t = {gt.t.iloc[0]:.3f} .. {gt.t.iloc[-1]:.3f} s '
      f'({len(gt) / duration:.1f} Hz)')
for label, i in (('initial', 0), ('final', -1)):
    print(f'  {label} pose: x={gt.x.iloc[i]:+.3f}  y={gt.y.iloc[i]:+.3f}  '
          f'yaw={gt.yaw.iloc[i]:+.3f} rad')

fig, ax = plt.subplots(figsize=(5, 5))
ax.plot(gt.x.to_numpy(), gt.y.to_numpy(), color=SERIES_1, linewidth=2)
ax.plot(gt.x.iloc[0], gt.y.iloc[0], 'o', color=SERIES_1, markersize=8)
ax.annotate('start', (gt.x.iloc[0], gt.y.iloc[0]), textcoords='offset points',
            xytext=(8, -14), color=INK_MUTED)
ax.set_title('Ground truth trajectory (Gazebo)', loc='left', color=INK)
ax.set_xlabel('x [m]', color=INK_MUTED)
ax.set_ylabel('y [m]', color=INK_MUTED)
ax.set_aspect('equal')
ax.grid(True, color=GRID, linewidth=0.8)
ax.tick_params(colors=INK_MUTED)
for side in ('top', 'right'):
    ax.spines[side].set_visible(False)

out = run / 'quick_check.png'
fig.savefig(out, dpi=120, bbox_inches='tight')
print(f'plot saved to {out}')
