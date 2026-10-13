# source this file from bash; paths may be overridden before sourcing.
export PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export AP_ROOT="${AP_ROOT:-$HOME/ardupilot}"
export AP_GZ_ROOT="${AP_GZ_ROOT:-$HOME/gz_ws/src/ardupilot_gazebo}"
export A_PYTHON="${A_PYTHON:-/usr/bin/python3}"
export GZ_VERSION=harmonic
export GZ_IP=127.0.0.1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$AP_GZ_ROOT/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$PROJECT_ROOT/models:$PROJECT_ROOT/worlds:$PROJECT_ROOT/sensors:$AP_GZ_ROOT/models:$AP_GZ_ROOT/worlds:${GZ_SIM_RESOURCE_PATH:-}"
# A's verified display workaround; applies to sensor rendering as well.
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export MAVLINK20=1
