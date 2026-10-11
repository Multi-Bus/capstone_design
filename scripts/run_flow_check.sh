#!/bin/bash
# GPS 끈 광류 비행 점검 (격리된 헤드리스 Gazebo + SITL + sensor_bridge). 사용: scripts/run_flow_check.sh flow_test.parm bridge
source ~/drone_project/env.sh
export GZ_PARTITION=probe
cd ~/drone_project
PY=~/venv-ardupilot/bin/python
rm -rf runs/probe; mkdir -p runs/probe
setsid gz sim -s -r -v4 --headless-rendering worlds/room.sdf > /tmp/probe_gz.log 2>&1 &
GZ=$!
sleep 10
gz service -s /world/room/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 8000 \
  --req 'sdf: "<sdf version=\"1.9\"><include><uri>model://iris_flow_test</uri><name>iris_flow_test</name><pose>-2 0 0.2 0 0 0</pose></include></sdf>"' 2>&1
sleep 3
cd runs/probe
setsid $PY ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I0 --sysid 1 \
  --no-mavproxy --no-rebuild -w --add-param-file=$HOME/drone_project/config/$1 > /tmp/probe_sitl.log 2>&1 &
SITL=$!
sleep 12
cd ~/drone_project
if [ "$2" = "bridge" ]; then
  setsid $PY scripts/sensor_bridge.py --mav tcp:127.0.0.1:5762 > /tmp/probe_bridge.log 2>&1 &
  BR=$!
fi
echo "== flight"
timeout 330 $PY scripts/flow_flight_check.py 2>&1 | tail -60
echo "== bridge log tail"; tail -6 /tmp/probe_bridge.log 2>/dev/null
echo "== gz log: errors"; grep -iE "error|fail" /tmp/probe_gz.log | head -5
[ -n "$BR" ] && kill -- -$BR 2>/dev/null
kill -- -$SITL 2>/dev/null; kill -- -$GZ 2>/dev/null; sleep 3
echo "== leftover"; pgrep -a "arducopter" | head -3 || true
