"""Sensor / GPS-off API / flight + integrated logger verification."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan
from get_truth import Truth
from fleet_api import Fleet
from csv_logger import COLUMNS, make_row
from visibility import visible_drones


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--stage',required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--ids',nargs='+',type=int,default=[0])
    a=ap.parse_args()
    truth=Truth()
    node=Node()
    sensor={i:dict(images=0, scans=0, texture=False, range_valid=False) for i in a.ids}
    callbacks=[]
    for i in a.ids:
        def image_cb(m,i=i):
            sensor[i]['images']+=1
            sensor[i]['texture']=len(m.data)>0 and max(m.data)!=min(m.data)
        def scan_cb(m,i=i):
            sensor[i]['scans']+=1
            sensor[i]['range_valid']=bool(m.ranges) and math.isfinite(m.ranges[0]) and .1<=m.ranges[0]<=5
        callbacks.extend([image_cb,scan_cb])
        assert node.subscribe(Image,f'/drone_{i}/down_camera/image',image_cb)
        assert node.subscribe(LaserScan,f'/drone_{i}/down_range/scan',scan_cb)
    fleet=None
    result={'stage':a.stage,'passed':False,'sensor':sensor}
    log=None
    phase='prepare'
    alignment={}
    last_log=-1
    def wait_until(predicate,seconds,label):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if predicate():
                return
            time.sleep(.1)
        raise TimeoutError(label)

    try:
        wait_until(lambda:all(s['images']>=10 and s['scans']>=10 and s['texture'] and s['range_valid']
                              and truth.get(i).get('valid') for i,s in sensor.items()),90,'sensor readiness')
        print('SENSORS_PASS',sensor,flush=True)
        if a.stage=='sensors':
            result['passed']=True
            return 0
        fleet=Fleet(a.ids,stale_s=1.0)  # slower software rendering: explicit integration tolerance
        wait_until(lambda:all(fleet.get_estimate(i)['valid'] for i in a.ids),180,'EKF/position readiness')
        for d in fleet.drones.values():
            d.set_origin()
        wait_until(lambda:all(d.origin is not None for d in fleet.drones.values()),30,'EKF origin readback')
        for d in fleet.drones.values():
            if abs(d.origin['latitude_deg']-37)>1e-6 or abs(d.origin['longitude_deg']-127)>1e-6:
                raise RuntimeError('EKF geographic origin mismatch')
        expected={'FRAME_CLASS':1,'FRAME_TYPE':1,'AHRS_EKF_TYPE':3,'EK3_ENABLE':1,
                  'GPS1_TYPE':0,'GPS2_TYPE':0,'FLOW_TYPE':5,'RNGFND1_TYPE':10,
                  'EK3_SRC1_POSXY':0,'EK3_SRC1_VELXY':5,'EK3_SRC1_POSZ':2}
        for d in fleet.drones.values():
            for key in expected:
                d.request_parameter(key)
        wait_until(lambda:all(all(k in d.parameters for k in expected) for d in fleet.drones.values()),30,'parameter readback')
        for i,d in fleet.drones.items():
            for key,value in expected.items():
                if abs(d.parameters[key]-value)>1e-6:
                    raise RuntimeError(f'drone {i}: {key} mismatch')
        samples={i:[] for i in a.ids}
        for _ in range(30):
            for i in a.ids:
                e,t=fleet.get_estimate(i),truth.get(i)
                if e['valid'] and t.get('valid'):
                    samples[i].append((e,t))
            time.sleep(.1)
        for i in a.ids:
            if len(samples[i])<10:
                raise RuntimeError('insufficient alignment samples')
            spatial=[statistics.median(t['position_enu_m'][j]-e['raw_position_enu_m'][j]
                                      for e,t in samples[i]) for j in range(3)]
            clock=statistics.median(t['time_sim_s']-e['time_boot_ms']/1000 for e,t in samples[i])
            alignment[i]={'world_minus_local_enu_m':spatial,'sim_minus_boot_s':clock,
                          'method':'stationary latest-sample median; not hardware time synchronization'}
            fleet.drones[i].set_alignment(spatial,clock)
        result['alignment']=alignment
        result['parameters']={i:d.parameters for i,d in fleet.drones.items()}
        print('ESTIMATE_PASS', {i:fleet.get_estimate(i) for i in a.ids},flush=True)
        log=open(a.out/'integrated.csv','w',newline='')
        columns=COLUMNS+['phase','raw_estimate_e','raw_estimate_n','raw_estimate_u','truth_time_sim_s']
        writer=csv.DictWriter(log,columns)
        writer.writeheader()

        def tick():
            nonlocal last_log
            ts={i:truth.get(i) for i in a.ids}
            if not all(t.get('valid') for t in ts.values()):
                raise RuntimeError('truth unavailable/stale; abort test')
            now=max(t['time_sim_s'] for t in ts.values())
            es={i:fleet.get_estimate(i) for i in a.ids}
            if not all(e['valid'] for e in es.values()):
                result['invalid_estimates'] = es
                raise RuntimeError('estimate unavailable/stale; abort test')
            if now-last_log>=.099:
                pos={i:t['position_enu_m'] for i,t in ts.items()}
                for i,t in ts.items():
                    vis=visible_drones(i,t['camera_position_enu_m'],pos,t['camera_yaw_enu_rad'])
                    r=make_row(a.out.name,now,i,t,es[i],vis)
                    r.update(phase=phase,raw_estimate_e=es[i]['raw_position_enu_m'][0],
                             raw_estimate_n=es[i]['raw_position_enu_m'][1],
                             raw_estimate_u=es[i]['raw_position_enu_m'][2],truth_time_sim_s=t['time_sim_s'])
                    writer.writerow(r)
                    if abs(t['position_enu_m'][0])>5.3 or abs(t['position_enu_m'][1])>5.3:
                        raise RuntimeError('truth outside test area')
                last_log=now
            return now,es

        def mode_all(mode):
            for d in fleet.drones.values():
                d.set_mode(mode)
            wait_until(lambda:all(fleet.get_estimate(i)['mode']==mode for i in a.ids),20,f'mode {mode}')

        mode_all('GUIDED')
        arm_deadline = time.monotonic()+90
        next_arm = 0
        while not all(fleet.get_estimate(i)['armed'] for i in a.ids):
            if time.monotonic() > arm_deadline:
                raise TimeoutError('arm: '+repr({i:d.texts[-10:] for i,d in fleet.drones.items()}))
            if time.monotonic() >= next_arm:
                for i,d in fleet.drones.items():
                    if not fleet.get_estimate(i)['armed']:
                        d.command(400,1)
                next_arm = time.monotonic()+5
            time.sleep(.1)
        phase='takeoff'
        for d in fleet.drones.values():
            d.command(22,0,0,0,0,0,0,2)
        end=time.monotonic()+150
        while True:
            now,es=tick()
            if all(e['raw_position_enu_m'][2]>=1.8 for e in es.values()):
                break
            if time.monotonic()>end:
                raise TimeoutError('takeoff')
            time.sleep(.05)
        print('TAKEOFF_PASS',flush=True)
        # Targets in raw EKF-local ENU: truth is only used for evaluation/abort.
        base={i:es[i]['raw_position_enu_m'][:] for i in a.ids}
        for b in base.values():
            b[2]=2.0

        def fly(name,targets,duration=None):
            nonlocal phase
            phase=name
            start, _=tick()
            held=None
            fixed_truth = {i:truth.get(i)['position_enu_m'][:] for i in a.ids}
            end=time.monotonic()+180
            while time.monotonic()<end:
                now,est=tick()
                near=True
                for i,e in est.items():
                    if e['mode']!='GUIDED' or not e['armed']:
                        raise RuntimeError('lost armed GUIDED state')
                    p=e['raw_position_enu_m']
                    delta=[targets[i][j]-p[j] for j in range(3)]
                    speed=math.hypot(delta[0],delta[1])
                    scale=min(1,.3/max(speed,1e-9))
                    fleet.send_velocity(i,delta[0]*scale,delta[1]*scale,max(-.3,min(.3,.5*delta[2])))
                    near &= speed<.2 and abs(delta[2])<.3
                    if name.startswith('move_') and i != int(name.split('_')[1]):
                        p_truth = truth.get(i)['position_enu_m']
                        if math.hypot(p_truth[0]-fixed_truth[i][0], p_truth[1]-fixed_truth[i][1]) > .35:
                            raise RuntimeError(f'non-target drone {i} moved over 0.35 m')
                held=(now if held is None else held) if near else None
                if duration is not None and now-start>=duration:
                    if held is None or now-held < 2:
                        raise RuntimeError(name+' did not hold target for final 2 simulation seconds')
                    print(name,'PASS',flush=True)
                    return
                if duration is None and held is not None and now-held>=2:
                    print(name,'PASS',flush=True)
                    return
                if duration is None and now-start>30:
                    raise TimeoutError(name+' simulation timeout')
                time.sleep(.05)
            raise TimeoutError(name+' host timeout')

        fly('hover',base,10)
        if a.stage=='square':
            for n,(x,y) in enumerate(((2,0),(2,2),(0,2),(0,0)),1):
                fly(f'square{n}',{i:[b[0]+x,b[1]+y,b[2]] for i,b in base.items()})
        if a.stage=='two':
            targets={i:b[:] for i,b in base.items()}
            for i in a.ids:
                targets[i][0]+=1.5
                fly(f'move_{i}',targets)
        # Observe an unrefreshed command expire, without issuing another movement.
        phase='command_expiry'
        for i in a.ids:
            fleet.send_velocity(i,0,0,0)
        time.sleep(.8)
        if not all(fleet.get_estimate(i)['command_expired'] for i in a.ids):
            raise RuntimeError('command expiry not observed')
        print('COMMAND_EXPIRY_PASS',flush=True)
        result['passed']=True
    except Exception as exc:
        result['error']=repr(exc)
        print('FAIL',repr(exc),flush=True)
    finally:
        if fleet:
            for d in fleet.drones.values():
                d.set_mode('LAND')
            try:
                wait_until(lambda:all(not fleet.get_estimate(i)['armed'] and
                                     fleet.get_estimate(i)['heartbeat_fresh'] for i in a.ids),150,'landing/disarm')
                result['landed']=True
            except Exception as exc:
                result.update(passed=False,landed=False,landing_error=repr(exc))
            result['telemetry']={i:dict(position_messages=d.received_positions,texts=d.texts,
                                        acks=d.acks,origin=d.origin,covariance_available=d.get_estimate()['uncertainty_valid'])
                                 for i,d in fleet.drones.items()}
            fleet.close()
        if log:
            log.close()
        (a.out/'check_result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
        print('RESULT',result['passed'],flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':
    rc=main()
    sys.stdout.flush()
    sys.stderr.flush()
    # B documented gz.transport interpreter finalization crash; files already closed.
    os._exit(rc)
