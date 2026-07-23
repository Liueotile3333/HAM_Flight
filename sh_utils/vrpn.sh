#!/bin/bash

# 确保ROS环境已加载
source /opt/ros/noetic/setup.bash  # 根据你的ROS版本修改路径

# 启动roscore
gnome-terminal --tab --title="roscore" -- bash -c "roscore; bash"

# 等待roscore启动完成（可选，避免启动顺序问题）
sleep 5

# 通过串口连接无人机
gnome-terminal --tab --title="mavros" -- bash -c "roslaunch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:57600; bash"

sleep 5

# 启动rpn动捕
gnome-terminal --tab --title="vrpn_client_ros" -- bash -c "cd ~/vrpn_client_ros; source devel/setup.bash; roslaunch vrpn_client_ros sample.launch; bash"

sleep 5

# 发布mavros消息
gnome-terminal --tab --title="topic_tools" -- bash -c "rosrun topic_tools relay /vrpn_client_node/RigidBody/pose /mavros/vision_pose/pose; bash"

sleep 5

gnome-terminal --window -e 'bash -c "sleep 2; rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0;
                            rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0;
                            rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0;
                            rosrun mavros mavcmd long 511 331 5000 0 0 0 0 0; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch ctrl_node run_ctrl.launch; exec bash"' \
