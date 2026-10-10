#!/bin/bash
# 사용: scripts/set_fov.sh 도(degree)   예) scripts/set_fov.sh 120
rad=$(awk -v d="$1" 'BEGIN{printf "%.4f", d*3.14159265/180}')
sed -i "s|<horizontal_fov>.*</horizontal_fov>|<horizontal_fov>$rad</horizontal_fov>|" ~/drone_project/sensors/front_camera/model.sdf
echo "horizontal_fov = $1 deg ($rad rad)"
