"""Isolated A verification. No folder deletion, EEPROM wipe, or global pkill.

Starts the built SITL binary directly to avoid sim_vehicle startup task cleanup.
Every subprocess has its own session and a unique persistent run directory.
"""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stage', choices=['sensors', 'hover', 'square', 'two'], default='sensors')
    ap.add_argument('--timeout', type=float, default=900)
    ap.add_argument('--mavproxy-streamrate',type=int,default=-1,
                    help='-1 leaves per-message rate control to Fleet; 4 reproduces old conflict')
    a = ap.parse_args()
    run = ROOT / 'runs' / time.strftime('a_%Y%m%d_%H%M%S')
    run.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env['GZ_PARTITION'] = f'a_{os.getpid()}_{time.time_ns()}'
    ids = [0, 1] if a.stage == 'two' else [0]
    ap_root = Path(env.get('AP_ROOT', str(Path.home()/'ardupilot')))
    binary = ap_root/'build/sitl/bin/arducopter'
    procs, handles = [], []
    outcome = {'stage': a.stage, 'run_dir': str(run), 'status': 'starting'}

    def launch(name, cmd, cwd=ROOT):
        f = open(run/f'{name}.log', 'w')
        handles.append(f)
        p = subprocess.Popen([str(x) for x in cmd], cwd=cwd, env=env,
                             stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        procs.append(p)
        return p

    def command(cmd, timeout=20):
        return subprocess.run([str(x) for x in cmd], cwd=ROOT, env=env,
                              capture_output=True, text=True, timeout=timeout, check=True)

    def cleanup():
        for p in reversed(procs):
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        until = time.monotonic()+3
        while time.monotonic()<until and any(p.poll() is None for p in procs):
            time.sleep(.1)
        for p in reversed(procs):
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            p.wait(timeout=5)
        for f in handles:
            f.close()

    def interrupted(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    print(f'RUN_DIR={run}', flush=True)
    rc = 1
    try:
        for i in ids:
            for kind, ports in ((socket.SOCK_STREAM, [5760+10*i,5762+10*i,5763+10*i]),
                                (socket.SOCK_DGRAM, [9002+10*i,14550+10*i,15550+10*i])):
                for port in ports:
                    with socket.socket(socket.AF_INET, kind) as s:
                        if kind == socket.SOCK_STREAM:
                            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                        s.bind(('127.0.0.1',port))
        for name in ('gz.transport13', 'gz.msgs10', 'cv2', 'numpy', 'pymavlink'):
            __import__(name)
        metadata = dict(python=sys.version, partition=env['GZ_PARTITION'], stage=a.stage,
                        gazebo=command(['gz','sim','--versions']).stdout.strip(),
                        ardupilot_commit=command(['git','-C',ap_root,'rev-parse','HEAD']).stdout.strip(),
                        project_commit=command(['git','rev-parse','HEAD']).stdout.strip(),
                        project_diff=command(['git','diff']).stdout,
                        plugin_commit=command(['git','-C',env['AP_GZ_ROOT'],'rev-parse','HEAD']).stdout.strip(),
                        software_rendering=env.get('LIBGL_ALWAYS_SOFTWARE'))
        metadata['source_sha256'] = {
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for group in ('scripts', 'config', 'worlds', 'sensors')
            for p in (ROOT/group).rglob('*')
            if p.is_file() and '__pycache__' not in p.parts
        }
        metadata['packages'] = {name: importlib.metadata.version(name)
                                for name in ('numpy', 'pymavlink', 'MAVProxy', 'opencv-python')}
        metadata['model'] = 'baseline_iris; physical vehicle specifications not applied'
        metadata['mavproxy_streamrate'] = a.mavproxy_streamrate
        metadata['arducopter_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
        (run/'metadata.json').write_text(json.dumps(metadata,indent=2))
        gz = launch('gazebo',['gz','sim','-s','-r','-v4','--headless-rendering',ROOT/'worlds/room.sdf'])
        for _ in range(60):
            if gz.poll() is not None:
                raise RuntimeError('Gazebo exited; see gazebo.log')
            topics = command(['gz','topic','-l']).stdout
            if '/world/room/dynamic_pose/info' in topics:
                break
            time.sleep(1)
        else:
            raise TimeoutError('Gazebo world topics did not appear')
        print(command([sys.executable,ROOT/'scripts/spawn_drones.py',*map(str,ids)]).stdout,flush=True)
        if a.stage != 'sensors':
            if not binary.is_file():
                raise FileNotFoundError(binary)
            for i in ids:
                folder=run/f'drone_{i}'
                folder.mkdir()
                defaults=','.join(map(str,[ap_root/'Tools/autotest/default_params/copter.parm',
                                          ap_root/'Tools/autotest/default_params/gazebo-iris.parm',
                                          ROOT/'config/a_flow.parm']))
                launch(f'sitl_{i}',[binary,'--model','JSON',f'-I{i}','--sysid',str(i+1),
                                    '--defaults',defaults,'--home','37.0,127.0,0,90'],folder)
                launch(f'mavproxy_{i}',[sys.executable,'-m','MAVProxy.mavproxy',
                       '--master',f'tcp:127.0.0.1:{5760+10*i}',
                       '--out',f'udp:127.0.0.1:{14550+10*i}',
                       '--out',f'udp:127.0.0.1:{15550+10*i}',
                       '--streamrate',str(a.mavproxy_streamrate),
                       '--daemon','--state-basedir',str(folder)],folder)
                launch(f'bridge_{i}',[sys.executable,'-u',ROOT/'scripts/sensor_bridge.py',
                       '--mav',f'udpin:127.0.0.1:{15550+10*i}',
                       '--cam',f'/drone_{i}/down_camera/image','--range',f'/drone_{i}/down_range/scan',
                       '--record',run/f'raw_bridge_{i}.jsonl','--drone-id',str(i)])
        check = launch('check',[sys.executable,'-u',ROOT/'scripts/a_check.py',
                                '--stage',a.stage,'--out',run,'--ids',*map(str,ids)])
        deadline=time.monotonic()+a.timeout
        while check.poll() is None and time.monotonic()<deadline:
            time.sleep(1)
        if check.poll() is None:
            raise TimeoutError('check timeout; run logs retained')
        rc=check.returncode
        outcome['status']='passed' if rc==0 else 'failed'
        print((run/'check.log').read_text(),flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        outcome.update(status='failed',error=repr(exc))
        print(repr(exc),flush=True)
    finally:
        cleanup()
        health={p.name:json.loads(p.read_text()) for p in run.glob('raw*.summary.json')}
        outcome['recorder_health']=health
        if outcome['status']=='passed' and a.stage!='sensors':
            expected=['raw_state.summary.json']+[f'raw_bridge_{i}.summary.json' for i in ids]
            if any(name not in health for name in expected) or any(
                    h['dropped'] or h['error'] or h['accepted']!=h['written'] for h in health.values()):
                outcome.update(status='failed',error='raw recording incomplete')
                rc=1
        outcome['exit_code']=rc
        (run/'runner_result.json').write_text(json.dumps(outcome,indent=2))
    return rc


if __name__=='__main__':
    sys.exit(main())
