rosbag record --tcpnodelay \
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
