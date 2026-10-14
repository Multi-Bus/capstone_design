"""Sequential repeat tests; each child owns its processes, ports and run folder."""
import argparse
import json
from pathlib import Path
import subprocess
import signal
import sys
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--repeat',type=int,default=2)
    parser.add_argument('--stages',nargs='+',choices=['square','two','hover'],default=['square','two'])
    args=parser.parse_args()
    if not 1<=args.repeat<=10:
        parser.error('repeat must be 1..10')
    manifest=ROOT/'runs'/time.strftime('batch_%Y%m%d_%H%M%S.json')
    manifest.parent.mkdir(exist_ok=True)
    records=[]
    for repetition in range(args.repeat):
        for stage in args.stages:
            record=dict(repetition=repetition+1,stage=stage,run_dir=None)
            with subprocess.Popen(['bash',str(ROOT/'scripts/run_a.sh'),'--stage',stage],
                                  cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True) as process:
                try:
                    for line in process.stdout:
                        print(line,end='',flush=True)
                        if line.startswith('RUN_DIR='):
                            record['run_dir']=line.strip().split('=',1)[1]
                    record['flight_exit_code']=process.wait()
                except KeyboardInterrupt:
                    # Also forward when only this PID was interrupted programmatically.
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=20)
                    raise
            if record['run_dir'] and record['flight_exit_code']==0:
                analyzed=subprocess.run([sys.executable,str(ROOT/'scripts/analyze_raw.py'),
                                         record['run_dir'],'--require-10hz'])
                record['rate_exit_code']=analyzed.returncode
            record['passed']=record['flight_exit_code']==0 and record.get('rate_exit_code')==0
            records.append(record)
            manifest.write_text(json.dumps(records,indent=2))
    print('BATCH_MANIFEST='+str(manifest))
    return 0 if all(r['passed'] for r in records) else 1


if __name__=='__main__':
    sys.exit(main())
