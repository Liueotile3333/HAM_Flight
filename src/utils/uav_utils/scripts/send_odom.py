#!/usr/bin/env python3

import rospy
from tf import transformations as tfs
from nav_msgs.msg import Odometry

if __name__ == "__main__":
    rospy.init_node("odom_sender")

    msg = Odometry()
    msg.header.stamp = rospy.Time.now() - rospy.Duration(0.2)
    msg.header.frame_id = "world"

    quaternion = tfs.quaternion_from_euler(0, 0, 0, "rzyx")
    msg.pose.pose.orientation.x = quaternion[0]
    msg.pose.pose.orientation.y = quaternion[1]
    msg.pose.pose.orientation.z = quaternion[2]
    msg.pose.pose.orientation.w = quaternion[3]

    publisher = rospy.Publisher("odom", Odometry, queue_size=10)
    counter = 0
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        counter += 1
        msg.header.stamp = rospy.Time.now() - rospy.Duration(0.2)
        publisher.publish(msg)
        rospy.loginfo("Send %3d msg(s).", counter)
        rate.sleep()
