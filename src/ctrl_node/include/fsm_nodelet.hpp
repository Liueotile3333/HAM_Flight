#ifndef CTRL_NODE_FSM_NODELET_HPP_
#define CTRL_NODE_FSM_NODELET_HPP_

#include <array>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <thread>
#include <atomic>
#include <mutex>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <Eigen/Dense>
#include <nodelet/nodelet.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>

// RTK / EKF 持续状态监测
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/EstimatorStatus.h>

#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <quadrotor_msgs/TakeoffLand.h>

#include "input.hpp"
#include "controller.hpp"
#include "param.hpp"

namespace ctrl_node
{

    // 服务调用结果分类(对应 MAV_RESULT 语义)
    enum class SrvResult
    {
        Accepted,      // ARM: MAV_RESULT_ACCEPTED；SetMode: 请求已发送，模式仍须由 state 确认
        Retryable,     // TEMPORARILY_REJECTED / IN_PROGRESS / mode_sent=false, 可重试
        Rejected,      // DENIED / NOT_SUPPORTED, 永久拒绝, 重试无意义
        TransportError // srv.call() 返回 false, mavros 不通
    };

    enum class NavigationFailsafeStage
    {
        Grace,
        Hover,
        AutoLand
    };

    inline NavigationFailsafeStage classify_navigation_failsafe(
        double unhealthy_duration,
        double failure_grace,
        double auto_land_timeout)
    {
        if (unhealthy_duration < failure_grace)
        {
            return NavigationFailsafeStage::Grace;
        }
        if (unhealthy_duration < auto_land_timeout)
        {
            return NavigationFailsafeStage::Hover;
        }
        return NavigationFailsafeStage::AutoLand;
    }

    // Critical input loss must not force LAND after the pilot/PX4 has already
    // left OFFBOARD. If state feedback itself is stale, retain the last known
    // OFFBOARD/armed state and the internal FSM state as conservative evidence
    // that this controller was flying the vehicle.
    inline bool critical_input_loss_requires_auto_land(
        bool state_feedback_fresh,
        bool armed,
        bool offboard,
        bool controller_flight_active,
        bool arm_transition_pending)
    {
        if (state_feedback_fresh)
        {
            return offboard &&
                   (armed || controller_flight_active || arm_transition_pending);
        }
        return (armed && offboard) || controller_flight_active ||
               arm_transition_pending;
    }

    // 轻量重试节流门: 限频 service 重试 + 最大重试次数, 防 fsm_timer 100Hz 周期下的服务调用风暴。
    // (仅时间节流 + 计数, 不加锁; fsm_timer 与 service 调用同在 fsm_queue_ 单线程串行, 无竞态)
    class RetryGate
    {
    public:
        RetryGate(double min_interval, int max_retries)
            : min_interval_(min_interval), max_retries_(max_retries) {}

        // 本周期是否允许发起调用: 未永久拒绝/未耗尽 且 距上次调用 >= min_interval_
        bool should_attempt(const ros::Time &now) const
        {
            if (rejected_ || retry_count_ >= max_retries_)
                return false;
            if (last_attempt_time_.isZero())
                return true;
            return (now - last_attempt_time_).toSec() >= min_interval_;
        }

        bool exhausted() const { return rejected_ || retry_count_ >= max_retries_; }
        bool rejected() const { return rejected_; }

        // 记录一次调用结果, 更新节流时间戳/计数/拒绝标志
        void update(const ros::Time &now, SrvResult result)
        {
            last_attempt_time_ = now;
            switch (result)
            {
            case SrvResult::Accepted:
                retry_count_ = 0;
                rejected_ = false;
                break;
            case SrvResult::Rejected:
                rejected_ = true;
                ++retry_count_;
                break;
            case SrvResult::Retryable:
            case SrvResult::TransportError:
                ++retry_count_;
                break;
            }
        }

        void reset()
        {
            retry_count_ = 0;
            rejected_ = false;
            last_attempt_time_ = ros::Time(0);
        }

    private:
        double min_interval_;
        int max_retries_;
        int retry_count_ = 0;
        ros::Time last_attempt_time_;
        bool rejected_ = false;
    };

    struct AutoTakeoff_t
    {
        ros::Time toggle_takeoff_time;
        Eigen::Vector4d start_pose;
        std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};

        static constexpr double MOTORS_SPEEDUP_TIME = 1.0; // motors idle for 1s before takeoff
        static constexpr double DELAY_TRIGGER_TIME = 2.0;  // Time to be delayed when reach at target height
    };

    // 自动降落运行时状态(对称 AutoTakeoff_t)
    struct AutoLanding_t
    {
        ros::Time toggle_land_time;
        Eigen::Vector4d start_pose; // land 起点 [x,y,z,yaw]
        double land_duration = 0.0; // 旧版平滑下降兼容状态（当前 LAND 不使用）
    };

    class FSM : public nodelet::Nodelet
    {
    private:
        enum CurrentState
        {
            MANUAL = 0,
            TAKEOFF = 1,
            HOVER,
            MISSION,
            LAND
        };

        Eigen::Vector4d hover_pose;

        bool takeoff_trigger_received = false;
        std::atomic<bool> land_trigger_received{false};

        CurrentState current_state;
        AutoTakeoff_t takeoff_state;
        AutoLanding_t landing_state;
        quadrotor_msgs::Px4ctrlDebug debug_msg;

        ros::CallbackQueue fsm_queue_;                   // FSM 专属回调队列: 与 manager MT 队列解耦
        std::shared_ptr<ros::AsyncSpinner> fsm_spinner_; // 单线程 spinner 串行执行 fsm_queue_, 消除回调间数据竞争
        ros::Timer fsm_timer_;

        ros::Subscriber mission_trigger_sub_;

        ros::Subscriber takeoff_trigger_sub_;
        ros::Subscriber state_sub;
        ros::Subscriber extended_state_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber cmd_sub;
        ros::Subscriber gps_raw_sub_;
        ros::Subscriber estimator_status_sub_;

        ros::Publisher ctrl_pv_pub; // position, velocity ctrl cmd
        ros::Publisher ctrl_aw_pub; // attitude, angle velocity ctrl cmd
        ros::Publisher traj_start_trigger_pub;
        ros::Publisher debug_pub; // debug

        ros::ServiceClient arming_client_srv;
        ros::ServiceClient set_mode_client_srv;

        enum class ServiceKind : std::size_t
        {
            Arm = 0U,
            AutoLand = 1U
        };

        struct ServiceCompletion
        {
            ServiceKind kind;
            SrvResult result;
        };

        std::mutex service_mutex_;
        std::condition_variable service_cv_;
        std::deque<ServiceKind> service_requests_;
        std::deque<ServiceCompletion> service_completions_;
        std::array<bool, 2U> service_pending_{{false, false}};
        std::thread service_worker_;
        bool stop_service_worker_ = false;

        // service 重试节流门: 防 ARM / AUTO.LAND 失败时 100Hz 服务调用风暴
        RetryGate arm_gate_{0.5, 20};  // ARM:  0.5s 节流, 最多 20 次(~10s); DENIED 时立即停止
        RetryGate land_gate_{0.5, 40}; // LAND: 0.5s 节流, 最多 40 次(~20s); 耗尽后 reset 继续(降落必须完成)

        // RTK/EKF 监测沿用 FSM 单线程回调队列，无需额外加锁。
        bool navigation_monitor_enabled_ = false;
        int rtk_fixed_min_type_ = 6;
        double navigation_status_timeout_ = 0.5;
        double navigation_failure_grace_ = 1.0;
        double navigation_auto_land_timeout_ = 3.0;
        double takeoff_timeout_margin_ = 5.0;
        double mission_trigger_timeout_ = 30.0;
        bool critical_input_failsafe_active_ = false;
        std::uint32_t trajectory_generation_ = 0U;
        std::uint32_t expected_trajectory_id_ = 0U;
        int current_rtk_fix_type_ = -1;
        bool ekf_pos_horiz_abs_ok_ = false;
        bool ekf_pos_vert_abs_ok_ = false;
        bool ekf_gps_glitch_ = true;
        ros::Time gps_status_rcv_stamp_;
        ros::Time estimator_status_rcv_stamp_;
        ros::Time navigation_unhealthy_since_;

        void init(ros::NodeHandle &nh);

        void fsm_timer(const ros::TimerEvent &event);

        void takeoff_trigger_callback(const quadrotor_msgs::TakeoffLandConstPtr &msgPtr);

        void gps_raw_callback(const mavros_msgs::GPSRAWConstPtr &msgPtr);

        void estimator_status_callback(const mavros_msgs::EstimatorStatusConstPtr &msgPtr);

        bool navigation_status_is_healthy(const ros::Time &now) const;

        void update_navigation_failsafe(const ros::Time &now);

        void handle_critical_input_loss(const ros::Time &now,
                                        const char *input_name,
                                        bool state_feedback_fresh,
                                        bool arm_result_accepted);

        SrvResult toggle_arm_disarm(bool arm);

        SrvResult request_auto_land();

        bool enqueue_service_request(ServiceKind kind);
        bool service_request_pending(ServiceKind kind);
        bool take_service_result(ServiceKind kind, SrvResult &result);
        void service_worker_loop();
        void stop_service_worker();

        void set_hov_with_odom();

        void set_start_pose_for_takeoff(const Odom_Data_t &odom);

        void set_start_pose_for_landing(const Odom_Data_t &odom);

        // 旧版平滑下降辅助函数，暂留以兼容现有接口；当前 LAND 直接交接 PX4。
        bool validateLandingRequest(const Odom_Data_t &odom);

        Controller::Desired_State_t get_pv_speed_up_des(const ros::Time &now);

        Controller::Desired_State_t get_takeoff_des(const Odom_Data_t &odom);

        Controller::Desired_State_t get_hover_des();

        Controller::Desired_State_t get_cmd_des();

        Controller::Desired_State_t get_land_des(const ros::Time &now);

        void publish_trigger(const Odom_Data_t &odom);

        void cancel_planner_handshake();

        void publish_position_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp);

        void publish_velocity_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp);

        void publish_bodyrate_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp);

        void publish_attitude_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp);

        double sanitize_thrust(double thrust) const;
        // MAVROS 最终指令防火墙
        Eigen::Vector3d sanitize_bodyrates(const Eigen::Vector3d &bodyrates) const;

        bool sanitize_attitude(const Eigen::Quaterniond &q_in, Eigen::Quaterniond &q_out) const;

        bool odom_is_received(const ros::Time &now_time);

        bool imu_is_received(const ros::Time &now_time);

        bool state_is_received(const ros::Time &now_time);

        bool extended_state_is_received(const ros::Time &now_time);

        bool cmd_is_received(const ros::Time &now_time);

    public:
        ~FSM() override;

        State_Data_t state_data;
        ExtendedState_Data_t extended_state_data;

        Mission_Trigger_t mission_trigger_data;

        Odom_Data_t odom_data;
        Imu_Data_t imu_data;
        Command_Data_t cmd_data;
        Parameter_t param;
        Controller::Velocity_Control vel_controller;
        Controller::Position_Control pos_controller;
        Controller::Attitude_Angular_Control att_ang_controller;

        void onInit(void);
    };
}

#endif  // CTRL_NODE_FSM_NODELET_HPP_
