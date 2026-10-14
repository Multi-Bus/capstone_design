"""Raw stream timing and phase diagnostics. Correlations are not causal claims."""
import argparse
from bisect import bisect_right, bisect_left
import csv
import gzip
import json
import math
from pathlib import Path
import statistics
from analyze_timing import interpolate, rmse


def load(path):
    if path.exists():
        stream=path.open(encoding='utf-8')
    else:
        stream=gzip.open(str(path)+'.gz','rt',encoding='utf-8')
    with stream:
        return [json.loads(line) for line in stream if line.strip()]


def percentile(values, p):
    if not values:
        return None
    xs = sorted(values)
    return xs[round((len(xs)-1)*p)]


def stream_stats(rows, time_key, scale=1):
    pairs = [(r['data'][time_key]*scale,r['host_monotonic_s']) for r in rows]
    intervals = [b[0]-a[0] for a,b in zip(pairs,pairs[1:])]
    host = [b[1]-a[1] for a,b in zip(pairs,pairs[1:])]
    span = pairs[-1][0]-pairs[0][0] if len(pairs)>1 else 0
    return dict(samples=len(rows),duplicates=sum(t==0 for t in intervals),
                backwards=sum(t<0 for t in intervals),
                rate_source_hz=(len(pairs)-1)/span if span>0 else None,
                interval_source_p50=percentile(intervals,.5),interval_source_p95=percentile(intervals,.95),
                interval_host_p95=percentile(host,.95),interval_host_max=max(host) if host else None,
                source_to_host_rate=span/(pairs[-1][1]-pairs[0][1]) if span>0 and pairs[-1][1]>pairs[0][1] else None)


def nearest(times, values, t, tolerance=.2):
    i=bisect_left(times,t)
    candidates=[j for j in (i-1,i) if 0<=j<len(times)]
    if not candidates:
        return None
    j=min(candidates,key=lambda j:abs(times[j]-t))
    return values[j] if abs(times[j]-t)<=tolerance else None


def correlation(pairs):
    if len(pairs)<3:
        return None
    x,y=zip(*pairs)
    mx,my=statistics.mean(x),statistics.mean(y)
    xx=sum((v-mx)**2 for v in x)
    yy=sum((v-my)**2 for v in y)
    return sum((a-mx)*(b-my) for a,b in pairs)/math.sqrt(xx*yy) if xx*yy>1e-18 else None


def analyze(run):
    result=json.loads((run/'check_result.json').read_text())
    events=load(run/'raw_state.jsonl')
    summary={'method':'Raw unique estimates vs interpolated truth using fixed preflight clock/position offsets',
             'limitations':['Approximate source clock mapping','Correlation does not identify cause',
                            'Observer receives messages, not internal EKF covariance'], 'drones':{}}
    diagnostic=[]
    for drone, alignment in result.get('alignment',{}).items():
        rows=[r for r in events if str(r['drone_id'])==drone]
        sources={name:[r for r in rows if r['source']==name] for name in
                 ('truth','LOCAL_POSITION_NED','ATTITUDE','EKF_STATUS_REPORT','HEARTBEAT','phase')}
        bridge=load(run/f'raw_bridge_{drone}.jsonl')
        flow=[r for r in bridge if r['source']=='flow']
        ranges=[r for r in bridge if r['source']=='range']
        stats={'streams':{name:stream_stats(sources[name],key,scale) for name,key,scale in
                         [('truth','time_sim_s',1),('LOCAL_POSITION_NED','time_boot_ms',.001),('ATTITUDE','time_boot_ms',.001)]},
               'flow':stream_stats(flow,'time_sim_s'), 'range':stream_stats(ranges,'time_sim_s'),
               'camera_skips':sum(r['source']=='camera_skip' for r in bridge),
               'phases':{}}
        for name in ('HEARTBEAT','EKF_STATUS_REPORT'):
            hosts=[r['host_monotonic_s'] for r in sources[name]]
            gaps=[b-a for a,b in zip(hosts,hosts[1:])]
            stats[name+'_host_gap_max_s']=max(gaps) if gaps else None
        truth={r['data']['time_sim_s']:r['data']['position'] for r in sources['truth']}
        truth_yaw={r['data']['time_sim_s']:r['data']['yaw_enu_rad'] for r in sources['truth']}
        tt=sorted(truth)
        tp=[truth[t] for t in tt]
        ty=[truth_yaw[t] for t in tt]
        phase={r['data']['time_sim_s']:r['data']['phase'] for r in sources['phase']}
        pt=sorted(phase)
        clock=alignment['sim_minus_boot_s']
        offset=alignment['world_minus_local_enu_m']
        attitudes=sorted([(r['data']['time_boot_ms']/1000+clock,r['data']) for r in sources['ATTITUDE']],key=lambda v:v[0])
        at=[v[0] for v in attitudes]
        av=[v[1] for v in attitudes]
        flows=sorted([(r['data']['time_sim_s'],r['data']) for r in flow],key=lambda v:v[0])
        ft=[v[0] for v in flows]
        fv=[v[1] for v in flows]
        seen=set()
        drone_records=[]
        for event in sources['LOCAL_POSITION_NED']:
            p=event['data']
            boot=p['time_boot_ms']
            if boot in seen:
                continue
            seen.add(boot)
            t=boot/1000+clock
            if not pt or t<pt[0] or t>pt[-1]:
                continue
            truth_p=interpolate(tt,tp,t)
            if truth_p is None:
                continue
            ep=[p['y']+offset[0],p['x']+offset[1],-p['z']+offset[2]]
            error=[a-b for a,b in zip(ep,truth_p)]
            attitude=nearest(at,av,t)
            optical=nearest(ft,fv,t)
            yaw=nearest(tt,ty,t)
            yaw_error=(math.pi/2-attitude['yaw']-yaw+math.pi)%(2*math.pi)-math.pi if attitude and yaw is not None else None
            record=dict(drone_id=drone,time_sim_s=t,phase=phase[pt[bisect_right(pt,t)-1]],
                        error_e_m=error[0],error_n_m=error[1],error_u_m=error[2],
                        xy_error_m=math.hypot(*error[:2]),
                        yaw_error_deg=math.degrees(yaw_error) if yaw_error is not None else None,
                        tilt_deg=math.degrees(math.acos(max(-1,min(1,math.cos(attitude['roll'])*math.cos(attitude['pitch']))))) if attitude else None,
                        flow_quality=optical['quality'] if optical else None,
                        flow_processing_host_s=optical['processing_host_s'] if optical else None)
            drone_records.append(record)
        for name in sorted({r['phase'] for r in drone_records}):
            group=[r for r in drone_records if r['phase']==name]
            qualities=[r['flow_quality'] for r in group if r['flow_quality'] is not None]
            tilts=[r['tilt_deg'] for r in group if r['tilt_deg'] is not None]
            yaw_errors=[r['yaw_error_deg'] for r in group if r['yaw_error_deg'] is not None]
            stats['phases'][name]=dict(samples=len(group),xy_rmse_m=rmse([r['xy_error_m'] for r in group]),
                xy_max_m=max(r['xy_error_m'] for r in group),tilt_p95_deg=percentile(tilts,.95),
                flow_quality_median=statistics.median(qualities) if qualities else None,
                yaw_error_median_deg=statistics.median(yaw_errors) if yaw_errors else None,
                flow_quality_zero_fraction=sum(v==0 for v in qualities)/len(qualities) if qualities else None)
        stats['xy_rmse_m']=rmse([r['xy_error_m'] for r in drone_records])
        stats['tilt_error_correlation']=correlation([(r['tilt_deg'],r['xy_error_m']) for r in drone_records if r['tilt_deg'] is not None])
        stats['flow_error_correlation']=correlation([(r['flow_quality'],r['xy_error_m']) for r in drone_records if r['flow_quality'] is not None])
        stats['flow_processing_host_p95_s']=percentile([r['data']['processing_host_s'] for r in flow],.95)
        accuracy=[r['data'] for r in rows if r['source']=='UTM_GLOBAL_POSITION']
        usable=[r for r in accuracy if all(0<r[k]<65535 for k in ('h_acc','v_acc','vel_acc'))]
        stats['accuracy_summary']=dict(samples=len(accuracy),usable_samples=len(usable),
            h_acc_m_range=[min(r['h_acc'] for r in usable)/1000,max(r['h_acc'] for r in usable)/1000] if usable else None,
            v_acc_m_range=[min(r['v_acc'] for r in usable)/1000,max(r['v_acc'] for r in usable)/1000] if usable else None)
        summary['drones'][drone]=stats
        diagnostic.extend(drone_records)
    summary['recorder_health']={p.name:json.loads(p.read_text()) for p in run.glob('raw*.summary.json')}
    with (run/'integrated.csv').open() as f:
        integrated=list(csv.DictReader(f))
    summary['csv']={}
    for drone in summary['drones']:
        rs=[r for r in integrated if r['drone_id']==drone]
        times=[float(r['time_sim_s']) for r in rs]
        summary['csv'][drone]=dict(rows=len(rs),rate_sim_hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else None,
                                  skipped_slots=sum(int(r.get('sample_skipped_slots',0)) for r in rs),
                                  repeated_estimate_rows=sum(r.get('estimate_is_new')=='False' for r in rs))
    if diagnostic:
        with (run/'raw_diagnostics.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(diagnostic[0]),lineterminator='\n')
            w.writeheader()
            w.writerows(diagnostic)
    (run/'raw_diagnostics.json').write_text(json.dumps(summary,indent=2))
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    parser.add_argument('--require-10hz',action='store_true')
    args=parser.parse_args()
    report=analyze(args.run)
    if args.require_10hz:
        failures=[]
        for drone,data in report['drones'].items():
            for label,rate in (('LOCAL_POSITION_NED',data['streams']['LOCAL_POSITION_NED']['rate_source_hz']),
                               ('CSV',report['csv'][drone]['rate_sim_hz'])):
                if rate is None or rate<9.5:
                    failures.append(f'drone {drone}: {label} {rate} Hz < 9.5 Hz')
        if not report['drones']:
            failures.append('no drone data')
        print('RATE_CHECK_FAIL' if failures else 'RATE_CHECK_PASS',failures)
        raise SystemExit(1 if failures else 0)
    print(json.dumps(report,indent=2))
