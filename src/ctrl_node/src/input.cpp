#include "input.hpp"

#include <cmath>

namespace ctrl_node
{

    template <typename Scalar_t>
    Scalar_t normalize_angle(Scalar_t a)
    {
        // NaN/Inf 防御: 与任何值的比较恒为 false, 旧的 while(true) 在此情形下无法退出
        // (assert 在 NDEBUG/Release 下被编译掉 → 死循环)。此处原样返回,
        // 由调用方(Command_Data_t::feed 的整帧 finite 校验)负责拒收。
        if (!std::isfinite(a))
            return a;

        // std::remainder 一次归一化到 (-π, π], 无循环、无迭代次数上限,
        // 亦无旧 assert(cnt<10) 对大角度合法输入(>|10π|)的误杀。
        return std::remainder(a, 2.0 * M_PI);
    };

    Mission_Trigger_t::Mission_Trigger_t() : received_at(ros::Time(0)) {}

    void Mission_Trigger_t::feed(geometry_msgs::PoseStampedConstPtr pMsg)
    {
        (void)pMsg;
        ++pending_sequence;
        // 0 保留为“尚未收到任务触发”；处理 uint64_t 回绕。
        if (pending_sequence == 0U)
        {
            ++pending_sequence;
        }
        received_at = ros::Time::now();
        ROS_INFO("\033[32m[planning]:plan trigger pending (sequence=%llu).\033[32m",
                 static_cast<unsigned long long>(pending_sequence));
    }

    bool Mission_Trigger_t::activatePending(const ros::Time &now, double max_age)
    {
        if (pending_sequence == 0U || pending_sequence == sequence)
        {
            return false;
        }

        const double age = (now - received_at).toSec();
        if (!std::isfinite(age) || age < 0.0 || age > max_age)
        {
            ROS_WARN("[planning]:pending trigger expired (age=%.3f s, limit=%.3f s).",
                     age, max_age);
            cancelPending();
            return false;
        }

        sequence = pending_sequence;
        active = true;
        received_at = now;
        ROS_INFO("[planning]:plan trigger activated (sequence=%llu).",
                 static_cast<unsigned long long>(sequence));
        return true;
    }

    void Mission_Trigger_t::deactivate()
    {
        active = false;
    }

    void Mission_Trigger_t::cancelPending()
    {
        pending_sequence = sequence;
        received_at = ros::Time(0);
    }

    Odom_Data_t::Odom_Data_t()
    {
        rcv_stamp = ros::Time(0);
        q.setIdentity();
    };

    void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
    {
        // validate→commit (与 Imu_Data_t / Command_Data_t 同策略): 先在局部变量里做
        // finite + 四元数范数校验, 全部通过才一次性写入成员 + rcv_stamp;
        // 任一字段非法(NaN/Inf/零四元数)整帧丢弃, 保持上一帧, rcv_stamp 不更新
        // → odom_is_received() 超时 → FSM 停止使用旧数据并请求 AUTO.LAND。
        // 原实现先写 rcv_stamp/p/msg 再查四元数, 且不校验 p/v/w → 含 NaN 的帧仍被
        // watchdog 判为"新鲜", 并把非法四元数替换为单位四元数后继续接收整帧。
        ros::Time now = ros::Time::now();

        // Pose is always consumed in the odometry/world frame.
        Eigen::Vector3d p_new(
            pMsg->pose.pose.position.x,
            pMsg->pose.pose.position.y,
            pMsg->pose.pose.position.z);
        Eigen::Quaterniond q_new(
            pMsg->pose.pose.orientation.w,
            pMsg->pose.pose.orientation.x,
            pMsg->pose.pose.orientation.y,
            pMsg->pose.pose.orientation.z);
        Eigen::Vector3d v_raw(
            pMsg->twist.twist.linear.x,
            pMsg->twist.twist.linear.y,
            pMsg->twist.twist.linear.z);
        // Angular velocity is intentionally retained in the body frame.
        Eigen::Vector3d w_new(
            pMsg->twist.twist.angular.x,
            pMsg->twist.twist.angular.y,
            pMsg->twist.twist.angular.z);

        if (!p_new.allFinite() ||
            !q_new.coeffs().allFinite() || q_new.norm() < 1e-6 ||
            !v_raw.allFinite() || !w_new.allFinite())
        {
            ROS_ERROR_THROTTLE(1.0, "[INPUT]: non-finite/invalid odometry rejected "
                                    "(p/v/w/q contains NaN/Inf or zero quaternion); keeping previous frame.");
            return; // 拒收: rcv_stamp 不更新 → odom 超时后 FSM 回退悬停
        }

        // commit
        q_new.normalize();

        Eigen::Vector3d v_new;
        if (odom_source == 0)
        {
            // The source already expresses linear velocity in the world frame.
            v_new = v_raw;
        }
        else if (odom_source == 1)
        {
            // nav_msgs/Odometry twist is expressed in child_frame_id. For the
            // MAVROS odometry used here child_frame_id is base_link, while the
            // trajectory command is expressed in the world/map frame. Convert
            // body-frame velocity to world frame before feedback is calculated.
            v_new = q_new * v_raw;
            ROS_INFO_ONCE("[INPUT]: converting odom linear velocity from child/body frame to world frame (frame_id='%s', child_frame_id='%s').",
                          pMsg->header.frame_id.c_str(), pMsg->child_frame_id.c_str());
        }
        else
        {
            ROS_ERROR_THROTTLE(1.0, "[INPUT]: unsupported odom_source=%d; using raw linear velocity.", odom_source);
            v_new = v_raw;
        }

        // rcv_stamp = 接收时刻(与 hpp 注释及 Imu/Command/State 一致), watchdog/is_received 用它做断流检测。
        rcv_stamp = now;
        p = p_new;
        q = q_new;
        v = v_new;
        w = w_new;

        // check the frequency (仅统计合法帧)
        static int one_min_count = 9999;
        static ros::Time last_clear_count_time = ros::Time(0.0);
        if ((now - last_clear_count_time).toSec() > 1.0)
        {
            if (one_min_count < 100)
            {
                ROS_WARN("ODOM frequency seems lower than 100Hz, which is too low!");
            }
            one_min_count = 0;
            last_clear_count_time = now;
        }
        one_min_count++;
    };

    Imu_Data_t::Imu_Data_t()
    {
        rcv_stamp = ros::Time(0);
        // 必须给 q 一个合法初值: 控制器 u.q = imu.q * odom.q.inverse() * q,
        // 首帧 IMU 到达前若 imu.q 为零四元数 → u.q 归零 → 姿态防火墙拦截并报 ERROR。
        // (与 Odom_Data_t 构造函数的 q.setIdentity() 保持一致。)
        q.setIdentity();
        w.setZero();
        a.setZero();
    }

    void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
    {
        ros::Time now = ros::Time::now();

        // validate→commit(与 Odom_Data_t 同策略): 先构造临时变量做 finite + quaternion norm 校验,
        // 全部通过才写入成员 + rcv_stamp; 任一字段非法(NaN/Inf/零四元数)整帧丢弃, 保持上一帧,
        // rcv_stamp 不更新 → imu_is_received() 超时 → FSM 停止下发并请求 AUTO.LAND。
        Eigen::Quaterniond q_new(pMsg->orientation.w, pMsg->orientation.x,
                                 pMsg->orientation.y, pMsg->orientation.z);
        Eigen::Vector3d w_new(pMsg->angular_velocity.x, pMsg->angular_velocity.y, pMsg->angular_velocity.z);
        Eigen::Vector3d a_new(pMsg->linear_acceleration.x, pMsg->linear_acceleration.y, pMsg->linear_acceleration.z);

        if (!q_new.coeffs().allFinite() || q_new.norm() < 1e-6 ||
            !w_new.allFinite() || !a_new.allFinite())
        {
            ROS_ERROR_THROTTLE(1.0, "[INPUT]: non-finite/invalid IMU data rejected "
                                    "(q/w/a contains NaN/Inf or zero quaternion); keeping previous frame.");
            return; // 拒收: rcv_stamp 不更新 → IMU 超时后 FSM 跳过下发
        }

        // commit
        q_new.normalize();
        q = q_new;
        w = w_new;
        a = a_new;
        rcv_stamp = now;

        // check the frequency (仅统计合法帧)
        static int one_min_count = 9999;
        static ros::Time last_clear_count_time = ros::Time(0.0);
        if ((now - last_clear_count_time).toSec() > 1.0)
        {
            if (one_min_count < 100)
            {
                ROS_WARN("IMU , which is too low!");
            }
            one_min_count = 0;
            last_clear_count_time = now;
        }
        one_min_count++;
    }

    State_Data_t::State_Data_t()
    {
        previous_state.mode = "OFFBOARD";
        rcv_stamp = ros::Time(0);
    };

    void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
    {
        current_state = *pMsg;
        rcv_stamp = ros::Time::now();
    };

    ExtendedState_Data_t::ExtendedState_Data_t() : rcv_stamp(ros::Time(0)) {}

    void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg)
    {
        current_extended_state = *pMsg;
        rcv_stamp = ros::Time::now();
    };

    Command_Data_t::Command_Data_t()
    {
        invalidate();
    }

    void Command_Data_t::invalidate()
    {
        rcv_stamp = ros::Time(0);
        p.setZero();
        v.setZero();
        a.setZero();
        j.setZero();
        yaw = 0.0;
        yaw_rate = 0.0;
        trajectory_id = 0U;
        trajectory_flag =
            quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;
    }

    void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
    {
        // validate→commit: 先把整帧读到局部变量并做 finite 校验, 全部通过才一次性提交到成员。
        // 任一字段非法(NaN/Inf, 多源于规划器/轨迹生成器异常)则拒收并告警, 保持旧值,
        // 避免坏数据污染下游期望状态 des → 控制器 NaN 输出。
        // 注: nodelet 为 MT 回调(getMTPrivateNodeHandle), 本模式缩小了"半更新"窗口,
        // 但 Eigen 赋值非原子, 不能消除与 fsm_timer 的并发竞态(彻底消除需 mutex/双缓冲)。
        // 拒收时不更新 rcv_stamp → cmd_is_received() 会因超时自然判为失效 → FSM 回退悬停(故障安全)。

        Eigen::Vector3d p_new(pMsg->position.x, pMsg->position.y, pMsg->position.z);
        Eigen::Vector3d v_new(pMsg->velocity.x, pMsg->velocity.y, pMsg->velocity.z);
        Eigen::Vector3d a_new(pMsg->acceleration.x, pMsg->acceleration.y, pMsg->acceleration.z);
        Eigen::Vector3d j_new(pMsg->jerk.x, pMsg->jerk.y, pMsg->jerk.z);
        const double yaw_new = pMsg->yaw;
        const double yaw_rate_new = pMsg->yaw_dot;
        const std::uint32_t trajectory_id_new = pMsg->trajectory_id;
        const std::uint8_t trajectory_flag_new = pMsg->trajectory_flag;

        const bool finite =
            p_new.allFinite() && v_new.allFinite() &&
            a_new.allFinite() && j_new.allFinite() &&
            std::isfinite(yaw_new) && std::isfinite(yaw_rate_new);

        if (!finite)
        {
            ROS_ERROR_THROTTLE(1.0, "[INPUT]: non-finite PositionCommand rejected "
                                    "(p/v/a/j/yaw/yaw_rate contains NaN/Inf); keeping previous command.");
            return; // 拒收: rcv_stamp 不更新 → cmd 超时后 FSM 回退悬停
        }

        // commit: 一次性写入成员
        rcv_stamp = ros::Time::now();
        p = p_new;
        v = v_new;
        a = a_new;
        j = j_new;
        yaw = normalize_angle(yaw_new);
        yaw_rate = yaw_rate_new;
        trajectory_id = trajectory_id_new;
        trajectory_flag = trajectory_flag_new;
    }

    bool Command_Data_t::isReadyForTrajectory(
        std::uint32_t expected_id) const
    {
        return expected_id != 0U &&
               trajectory_id == expected_id &&
               trajectory_flag ==
                   quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    }
}
