gnome-terminal --window -e 'bash -c "sleep 2; rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0;
                            rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0;
                            rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0;
                            rosrun mavros mavcmd long 511 331 5000 0 0 0 0 0; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch ctrl_node run_ctrl.launch; exec bash"' \

# --tab -e 'bash -c "sleep 16; roslaunch realflight_utils traj_analyse.launch; exec bash"' \

#  --tab -e 'bash -c "sleep 3; rosrun rosserial_server socket_node /mavros/vision_pose/pose; exec bash"' \
# --tab -e 'bash -c "sleep 3; roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0"; exec bash"' \
