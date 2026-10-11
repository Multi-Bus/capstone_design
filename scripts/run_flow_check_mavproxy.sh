#!/bin/bash
# A 가이드 연결 경로 점검: SITL + MAVProxy(--out 14550 제어, 15550 센서 브리지), 대화형 입력 없음(--daemon).
# 사용: scripts/run_flow_check_mavproxy.sh [실행할 시험 명령...]
#   기본 시험: scripts/flow_accuracy_check.py --mav udpin:127.0.0.1:14550
# 환경변수: PART(gz 파티션, 기본 agentA) RUN(실행 폴더 이름, 기본 agentA) INST(SITL 인스턴스, 기본 0) LOGD(로그 폴더, 기본 /tmp/$RUN)
source ~/drone_project/env.sh
PART=${PART:-agentA}; RUN=${RUN:-agentA}; INST=${INST:-0}; LOGD=${LOGD:-/tmp/$RUN}
export GZ_PARTITION=$PART
cd ~/drone_project
PY=~/venv-ardupilot/bin/python
export MAVPROXY_CMD=~/venv-ardupilot/bin/mavproxy.py  # sim_vehicle이 PATH에서 mavproxy.py를 찾는다
mkdir -p $LOGD; rm -rf runs/$RUN; mkdir -p runs/$RUN
CTL=$((14550 + 10 * INST)); BRG=$((15550 + 10 * INST))

setsid gz sim -s -r -v4 --headless-rendering worlds/room.sdf > $LOGD/gz.log 2>&1 &
GZ=$!
sleep 10
gz service -s /world/room/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 8000 \
  --req 'sdf: "<sdf version=\"1.9\"><include><uri>model://iris_flow_test</uri><name>iris_flow_test</name><pose>-2 0 0.2 0 0 0</pose></include></sdf>"' 2>&1
sleep 3
cd runs/$RUN
# --no-extra-ports: 기본 --out(WSL2에서는 Windows 호스트로 감)을 빼고 아래 --out만 쓴다. --daemon: MAVProxy 대화형 입력 끔.
setsid $PY ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I$INST --sysid $((INST + 1)) \
  --no-rebuild -w --add-param-file=$HOME/drone_project/config/flow_test.parm --no-extra-ports \
  --out=udp:127.0.0.1:$CTL --out=udp:127.0.0.1:$BRG --mavproxy-args=--daemon < /dev/null > $LOGD/sitl.log 2>&1 &
SITL=$!
sleep 12
cd ~/drone_project
setsid $PY scripts/sensor_bridge.py --mav udpin:127.0.0.1:$BRG > $LOGD/bridge.log 2>&1 &
BR=$!
echo "== 시험"
if [ $# -eq 0 ]; then set -- $PY scripts/flow_accuracy_check.py --mav udpin:127.0.0.1:$CTL; fi
timeout 900 "$@" 2>&1 | tail -150
echo "== bridge log tail"; tail -4 $LOGD/bridge.log
echo "== EKF/flow 메시지 (sitl.log)"; grep -E "EKF3 IMU0|optical flow|Arm|PreArm|failsafe|EKF" $LOGD/sitl.log | head -20
echo "== gz log: errors"; grep -iE "error|fail" $LOGD/gz.log | head -5
kill -- -$BR 2>/dev/null; kill -- -$SITL 2>/dev/null; kill -- -$GZ 2>/dev/null; sleep 3
echo "== leftover (내 프로세스 그룹)"; pgrep -a -g $SITL,$GZ,$BR | head -5 || true
