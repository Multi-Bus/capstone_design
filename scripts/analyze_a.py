"""Plot integrated CSV. Error is latest-sample paired, not synchronized accuracy."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    with (args.run/'integrated.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    summary = {'comparison': 'stationary offset + latest-sample pairing; not time-interpolated',
               'drift_reference': 'one preflight offset, never refitted during flight', 'drones': {}}
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    for drone in sorted({r['drone_id'] for r in rows}):
        rs = [r for r in rows if r['drone_id'] == drone and r['estimate_valid'] == 'true']
        if not rs:
            continue
        def values(column):
            return [float(r[column]) for r in rs]
        t = values('time_sim_s')
        xy = [math.hypot(float(r['estimate_x'])-float(r['truth_x']),
                         float(r['estimate_y'])-float(r['truth_y'])) for r in rs]
        stats = {'rows': len(rs), 'span_sim_s': t[-1]-t[0],
                 'median_interval_sim_s': statistics.median(b-a for a,b in zip(t,t[1:])) if len(t)>1 else None,
                 'xy_rmse_m': math.sqrt(statistics.mean(e*e for e in xy)),
                 'xy_max_m': max(xy), 'phases': {},
                 'uncertainty_valid_rows': sum(r['uncertainty_valid']=='true' for r in rs)}
        for phase in sorted({r['phase'] for r in rs}):
            errors = [e for e,r in zip(xy,rs) if r['phase']==phase]
            stats['phases'][phase] = {'rows': len(errors),
                                     'xy_rmse_m': math.sqrt(statistics.mean(e*e for e in errors)),
                                     'xy_max_m': max(errors)}
        summary['drones'][drone] = stats
        axes[0].plot(values('truth_x'), values('truth_y'), label=f'{drone} truth')
        axes[0].plot(values('estimate_x'), values('estimate_y'), '--', label=f'{drone} estimate')
        axes[1].plot(t, values('truth_z'), label=f'{drone} truth')
        axes[1].plot(t, values('estimate_z'), '--', label=f'{drone} estimate')
        axes[2].plot(t, xy, label=f'{drone} XY error')
    for ax, title, xlabel, ylabel in zip(axes,
            ('World trajectory', 'World height', 'Latest-sample horizontal error'),
            ('East (m)', 'Simulation time (s)', 'Simulation time (s)'),
            ('North (m)', 'Up (m)', 'Error (m)')):
        ax.set(title=title,xlabel=xlabel,ylabel=ylabel)
        ax.grid(True)
        ax.legend()
    axes[0].axis('equal')
    fig.tight_layout()
    fig.savefig(args.run/'trajectory.png',dpi=160)
    plt.close(fig)
    (args.run/'metrics.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
