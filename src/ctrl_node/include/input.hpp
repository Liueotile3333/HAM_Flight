#ifndef CTRL_NODE_INPUT_HPP_
#define CTRL_NODE_INPUT_HPP_

#include <cstdint>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>

namespace ctrl_node
{

    class Mission_Trigger_t
    {
    public:
        // geometry_msgs::PoseStamped;

        std::uint64_t sequence = 0U;
        std::uint64_t pending_sequence = 0U;
        // sequence 只表示任务代次；active 才表示当前控制周期是否处于任务控制。
        // 两者分离可避免第一次任务结束后，后续悬停/起飞继续使用旧估计器状态。
        bool active = false;
        ros::Time received_at;
        void feed(geometry_msgs::PoseStampedConstPtr pMsg);
        bool activatePending(const ros::Time &now, double max_age);
        void deactivate();
        void cancelPending();
        Mission_Trigger_t();
    };

    class Odom_Data_t
    {
    public:
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Quaterniond q;
        Eigen::Vector3d w;

        // Odometry convention:
        //   0: pose and linear velocity are already expressed in the world frame.
        //   1: pose is in the world frame, while linear velocity is expressed in
        //      the child/body frame (MAVROS local_position/odom: map -> base_link).
        // The flight stack used by this project publishes the latter, so keep 1
        // as the default. feed() converts v to the world frame before it is used
        // by the position/velocity feedback loops. Angular velocity w remains body-frame.
        int odom_source = 0;

        // 接收时间：供 watchdog 判断消息是否新鲜
        ros::Time rcv_stamp;

        Odom_Data_t();
        void feed(nav_msgs::OdometryConstPtr pMsg);
    };

    class Imu_Data_t
    {
    public:
        Eigen::Quaterniond q;
        Eigen::Vector3d w;
        Eigen::Vector3d a;

        ros::Time rcv_stamp;

        Imu_Data_t();
        void feed(sensor_msgs::ImuConstPtr pMsg);
    };

    class State_Data_t
    {
    public:
        mavros_msgs::State current_state;
        mavros_msgs::State previous_state;
        ros::Time rcv_stamp;

        State_Data_t();
        void feed(mavros_msgs::StateConstPtr pMsg);
    };

    class ExtendedState_Data_t
    {
    public:
        mavros_msgs::ExtendedState current_extended_state;
        ros::Time rcv_stamp;

        ExtendedState_Data_t();
        void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
    };

    class Command_Data_t
    {
    public:
        Eigen::Vector3d p   = Eigen::Vector3d::Zero();
        Eigen::Vector3d v   = Eigen::Vector3d::Zero();
        Eigen::Vector3d a   = Eigen::Vector3d::Zero();
        Eigen::Vector3d j   = Eigen::Vector3d::Zero();
        // 原 Eigen::Vector3d omg 已删除: PositionCommand.msg 无三轴角速度字段,
        // feed() 从未赋值 → 下游 des.omg 读到未定义内存。体轴前馈改由 yaw_rate 提供。
        double yaw      = 0.0;
        double yaw_rate = 0.0;
        std::uint32_t trajectory_id = 0U;
        std::uint8_t trajectory_flag =
            quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;

        ros::Time rcv_stamp;

        Command_Data_t();
        void invalidate();
        void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
        bool isReadyForTrajectory(std::uint32_t expected_id) const;
    };
}
#endif  // CTRL_NODE_INPUT_HPP_
