gnome-terminal --window -e 'bash -c "roslaunch uav_utils base_sim_single_vehicle_ground_truth.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch estimator add_bias.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch ctrl_node run_ctrl.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0; "' \
--tab -e 'bash -c "sleep 6; rosrun mavros mavcmd long 511 331 5000 0 0 0 0 0; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch realflight_utils rc_remap_sim.launch; exec bash"' \
# odomRemap 输出 /odom/remap (供 rosbag 离线分析/traj_analyse; 控制链路不消费,仅占少量 CPU)
--tab -e 'bash -c "sleep 9; roslaunch realflight_utils odom_remap_sim.launch; exec bash"' \
