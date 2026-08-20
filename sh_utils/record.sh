#!/bin/bash

# 将 bag 固定保存到脚本同级目录下的 Bag 文件夹
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BAG_DIR="${SCRIPT_DIR}/Bag"
mkdir -p "${BAG_DIR}"
cd "${BAG_DIR}" || exit 1

rosbag record --tcpnodelay \
-O "ham_flight_$(date +%Y-%m-%d-%H-%M).bag" \
/debugPx4ctrl \
/drone0/ctrl_node/takeoff_land \
/mavros/state \
/mavros/setpoint_raw/attitude \
/mavros/local_position/odom \
/mavros/local_position/velocity_local \
/mavros/global_position/raw/fix \
/drone0/planning/cmd \
/desire_pose_current_traj \
/mavros/imu/data \
/odom/remap \
/ir_pose_topic
