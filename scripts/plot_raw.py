"""Plot raw-aligned error, tilt and flow quality without changing measurements."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser()
parser.add_argument('run',type=Path)
args=parser.parse_args()
with (args.run/'raw_diagnostics.csv').open() as f:
    rows=list(csv.DictReader(f))
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
for drone in sorted({r['drone_id'] for r in rows}):
    rs=[r for r in rows if r['drone_id']==drone]
    for ax,key in zip(axes,('xy_error_m','tilt_deg','flow_quality')):
        valid=[r for r in rs if r[key]!='']
        ax.plot([float(r['time_sim_s']) for r in valid],[float(r[key]) for r in valid],label=f'Drone {drone}')
for ax,label in zip(axes,('Horizontal error (m)','Estimated tilt (deg)','Flow quality (0-255)')):
    ax.set_ylabel(label)
    ax.grid(True,alpha=.3)
    ax.legend(loc='upper right')
axes[0].set_title(args.run.name+' — fixed preflight alignment')
axes[-1].set_xlabel('Simulation time (s)')
axes[-1].set_ylim(-5,260)
fig.tight_layout()
fig.savefig(args.run/'raw_diagnostics.png',dpi=150)
plt.close(fig)
