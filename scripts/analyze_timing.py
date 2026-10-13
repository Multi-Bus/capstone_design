"""Compare at estimated sample time using recorded clock offset; no extrapolation.

This corrects pairing only. The preflight clock offset itself remains approximate.
"""
import argparse
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path
import statistics


def interpolate(times, positions, target, max_gap=.3):
    i = bisect_left(times, target)
    if i < len(times) and abs(times[i]-target) < 1e-9:
        return positions[i]
    if i == 0 or i == len(times) or times[i]-times[i-1] > max_gap:
        return None
    fraction = (target-times[i-1])/(times[i]-times[i-1])
    return [a+(b-a)*fraction for a,b in zip(positions[i-1],positions[i])]


def rmse(values):
    return math.sqrt(statistics.mean(v*v for v in values)) if values else None


def analyze(run, max_gap=.3):
    result = json.loads((run/'check_result.json').read_text())
    with (run/'integrated.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    report = {'method': 'Linear truth interpolation at boot time + recorded preflight clock offset',
              'limitation': 'Approximate clock alignment; retained CSV samples only; no extrapolation or invented 10Hz samples',
              'max_interpolation_gap_sim_s': max_gap, 'drones': {}}
    pairs = []
    for drone in sorted({r['drone_id'] for r in rows}):
        rs = [r for r in rows if r['drone_id']==drone and r['estimate_valid']=='true']
        clock = result['alignment'][drone]['sim_minus_boot_s']
        # Duplicate truth stamps contain one observation, not new measurements.
        truth = {float(r['truth_time_sim_s']): [float(r['truth_'+axis]) for axis in 'xyz'] for r in rs}
        times = sorted(truth)
        positions = [truth[t] for t in times]
        seen = set()
        errors, latest, ages, boot_times = [], [], [], []
        rejected = 0
        for r in rs:
            boot = int(r['estimate_time_boot_ms'])
            ages.append(float(r['truth_time_sim_s'])-(boot/1000+clock))
            if boot in seen:
                continue
            seen.add(boot)
            boot_times.append(boot/1000)
            target = boot/1000+clock
            matched = interpolate(times, positions, target, max_gap)
            if matched is None:
                rejected += 1
                continue
            estimate = [float(r['estimate_'+axis]) for axis in 'xyz']
            error = math.hypot(estimate[0]-matched[0], estimate[1]-matched[1])
            original = math.hypot(estimate[0]-float(r['truth_x']), estimate[1]-float(r['truth_y']))
            errors.append(error)
            latest.append(original)
            pairs.append(dict(drone_id=drone,phase=r['phase'],estimate_time_sim_s=target,
                              truth_x=matched[0],truth_y=matched[1],truth_z=matched[2],
                              estimate_x=estimate[0],estimate_y=estimate[1],estimate_z=estimate[2],
                              xy_error_m=error,latest_pair_xy_error_m=original))
        intervals = [b-a for a,b in zip(times,times[1:])]
        span = times[-1]-times[0] if times else 0
        report['drones'][drone] = dict(
            logged_rows=len(rs), unique_truth_samples=len(times), unique_estimate_samples=len(seen),
            duplicate_estimate_rows=len(rs)-len(seen), aligned_pairs=len(errors), rejected_pairs=rejected,
            retained_truth_hz=(len(times)-1)/span if span else None,
            median_truth_interval_sim_s=statistics.median(intervals) if intervals else None,
            max_truth_interval_sim_s=max(intervals) if intervals else None,
            estimate_stamp_backsteps=sum(b<a for a,b in zip(boot_times,boot_times[1:])),
            negative_pair_age_rows=sum(a<0 for a in ages),
            pair_age_min_sim_s=min(ages) if ages else None,
            pair_age_max_sim_s=max(ages) if ages else None,
            same_subset_latest_xy_rmse_m=rmse(latest), interpolated_xy_rmse_m=rmse(errors))
    (run/'timing_metrics.json').write_text(json.dumps(report,indent=2))
    if pairs:
        with (run/'time_aligned.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(pairs[0]))
            writer.writeheader()
            writer.writerows(pairs)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    args=parser.parse_args()
    print(json.dumps(analyze(args.run),indent=2))
