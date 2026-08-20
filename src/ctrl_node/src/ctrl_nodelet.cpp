#include "fsm_nodelet.hpp"

#include <cmath>

namespace ctrl_node
{

    FSM::~FSM()
    {
        fsm_timer_.stop();
        if (fsm_spinner_)
        {
            fsm_spinner_->stop();
        }
        stop_service_worker();
    }

    bool FSM::enqueue_service_request(ServiceKind kind)
    {
        const std::size_t index = static_cast<std::size_t>(kind);
        std::lock_guard<std::mutex> lock(service_mutex_);
        if (stop_service_worker_ || service_pending_[index])
        {
            return false;
        }
        service_pending_[index] = true;
        service_requests_.push_back(kind);
        service_cv_.notify_one();
        return true;
    }

    bool FSM::service_request_pending(ServiceKind kind)
    {
        const std::size_t index = static_cast<std::size_t>(kind);
        std::lock_guard<std::mutex> lock(service_mutex_);
        return service_pending_[index];
    }

    bool FSM::take_service_result(ServiceKind kind, SrvResult &result)
    {
        std::lock_guard<std::mutex> lock(service_mutex_);
        for (auto it = service_completions_.begin();
             it != service_completions_.end(); ++it)
        {
            if (it->kind == kind)
            {
                result = it->result;
                service_completions_.erase(it);
                return true;
            }
        }
        return false;
    }

    void FSM::service_worker_loop()
    {
        while (true)
        {
            ServiceKind kind = ServiceKind::Arm;
            {
                std::unique_lock<std::mutex> lock(service_mutex_);
                service_cv_.wait(lock, [this]
                {
                    return stop_service_worker_ || !service_requests_.empty();
                });
                if (stop_service_worker_)
                {
                    return;
                }
                kind = service_requests_.front();
                service_requests_.pop_front();
            }

            const SrvResult result =
                kind == ServiceKind::Arm
                    ? toggle_arm_disarm(true)
                    : request_auto_land();

            {
                std::lock_guard<std::mutex> lock(service_mutex_);
                service_pending_[static_cast<std::size_t>(kind)] = false;
                service_completions_.push_back(ServiceCompletion{kind, result});
            }
        }
    }

    void FSM::stop_service_worker()
    {
        {
            std::lock_guard<std::mutex> lock(service_mutex_);
            stop_service_worker_ = true;
            service_requests_.clear();
        }
        arming_client_srv.shutdown();
        set_mode_client_srv.shutdown();
        service_cv_.notify_all();
        if (service_worker_.joinable())
        {
            service_worker_.join();
        }
    }

    void FSM::takeoff_trigger_callback(const quadrotor_msgs::TakeoffLandConstPtr &msgPtr)
    {
        // 仅接受显式 TAKEOFF(1)/LAND(2), 其他值拒绝并告警, 防脏数据误触发起飞
        const uint8_t cmd = msgPtr->takeoff_land_cmd;

        if (cmd == quadrotor_msgs::TakeoffLand::TAKEOFF)
        {
            if (!takeoff_trigger_received)
            {
                takeoff_trigger_received = true;
                ROS_INFO("\033[32m[FSM]:Takeoff trigger received!\033[32m");
            }
            else
            {
                ROS_ERROR("[FSM]:Takeoff trigger duplicated!");
            }
        }
        else if (cmd == quadrotor_msgs::TakeoffLand::LAND)
        {
            if (!land_trigger_received)
            {
                land_trigger_received = true;
                ROS_INFO("\033[32m[FSM]:Land trigger received!\033[32m");
            }
            else
            {
                ROS_ERROR("[FSM]:Land trigger duplicated!");
            }
        }
        else
        {
            ROS_WARN("[FSM]:Unknown takeoff_land_cmd=%d, ignored (only TAKEOFF=1/LAND=2 accepted).",
                     static_cast<int>(cmd));
        }
    }

    void FSM::gps_raw_callback(const mavros_msgs::GPSRAWConstPtr &msgPtr)
    {
        current_rtk_fix_type_ = static_cast<int>(msgPtr->fix_type);
        gps_status_rcv_stamp_ = ros::Time::now();
    }

    void FSM::estimator_status_callback(
        const mavros_msgs::EstimatorStatusConstPtr &msgPtr)
    {
        ekf_pos_horiz_abs_ok_ = msgPtr->pos_horiz_abs_status_flag;
        ekf_pos_vert_abs_ok_ = msgPtr->pos_vert_abs_status_flag;
        ekf_gps_glitch_ = msgPtr->gps_glitch_status_flag;
        estimator_status_rcv_stamp_ = ros::Time::now();
    }

    bool FSM::navigation_status_is_healthy(const ros::Time &now) const
    {
        if (!navigation_monitor_enabled_)
        {
            return true;
        }

        const double gps_age = (now - gps_status_rcv_stamp_).toSec();
        const double estimator_age = (now - estimator_status_rcv_stamp_).toSec();
        const bool timestamps_fresh =
            !gps_status_rcv_stamp_.isZero() &&
            !estimator_status_rcv_stamp_.isZero() &&
            std::isfinite(gps_age) && gps_age >= 0.0 &&
            gps_age < navigation_status_timeout_ &&
            std::isfinite(estimator_age) && estimator_age >= 0.0 &&
            estimator_age < navigation_status_timeout_;

        return timestamps_fresh &&
               current_rtk_fix_type_ >= rtk_fixed_min_type_ &&
               ekf_pos_horiz_abs_ok_ && ekf_pos_vert_abs_ok_ &&
               !ekf_gps_glitch_;
    }

    void FSM::init(ros::NodeHandle &nh)
    {
        // 先把本 NodeHandle 的回调挂到 FSM 专属队列: 之后所有订阅与 fsm_timer 均由
        // init 末尾的单线程 spinner 在 fsm_queue_ 上串行执行, 与 manager 的多 worker
        // spinner 解耦, 消除回调间数据竞争。勿删 setCallbackQueue / 勿加阻塞调用。
        nh.setCallbackQueue(&fsm_queue_);

        // 参数缺失/非法 → 直接放弃启动 (不创建订阅/timer/spinner)
        if (!param.config_from_ros_handle(nh))
        {
            ROS_FATAL(
                "[FSM]: parameter loading/validation failed (see FATAL logs above). "
                "FSM timer will NOT be started.");

            return;
        }

        nh.param("navigation_monitor/enabled", navigation_monitor_enabled_, false);
        nh.param("navigation_monitor/rtk_fixed_min_type", rtk_fixed_min_type_, 6);
        nh.param("navigation_monitor/status_timeout", navigation_status_timeout_, 0.5);
        nh.param("navigation_monitor/failure_grace", navigation_failure_grace_, 1.0);
        nh.param("navigation_monitor/auto_land_timeout", navigation_auto_land_timeout_, 3.0);
        nh.param("takeoff_state/timeout_margin", takeoff_timeout_margin_, 5.0);
        nh.param("mission_trigger_timeout", mission_trigger_timeout_, 30.0);

        if (rtk_fixed_min_type_ < 0 || rtk_fixed_min_type_ > 8 ||
            !std::isfinite(navigation_status_timeout_) || navigation_status_timeout_ <= 0.0 ||
            !std::isfinite(navigation_failure_grace_) || navigation_failure_grace_ < 0.0 ||
            !std::isfinite(navigation_auto_land_timeout_) ||
            navigation_auto_land_timeout_ <= navigation_failure_grace_ ||
            !std::isfinite(takeoff_timeout_margin_) || takeoff_timeout_margin_ <= 0.0 ||
            !std::isfinite(mission_trigger_timeout_) || mission_trigger_timeout_ <= 0.0)
        {
            ROS_FATAL("[FSM]: invalid safety-monitor parameter; controller not started.");
            return;
        }
        trajectory_generation_ =
            static_cast<std::uint32_t>(ros::WallTime::now().toNSec());
        if (trajectory_generation_ == 0U)
        {
            trajectory_generation_ = 1U;
        }
        odom_data.odom_source = param.odom_source;
        vel_controller.init(param);
        pos_controller.init(param);

        if (!att_ang_controller.init(param, nh))
        {
            ROS_FATAL(
                "[FSM]: attitude/aerodynamic controller "
                "initialization failed. "
                "FSM timer will NOT be started.");

            return;
        }

        takeoff_trigger_sub_ = nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land", 1, &FSM::takeoff_trigger_callback, this,
                                                                        ros::TransportHints().tcpNoDelay());

        mission_trigger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/landing_trigger", 1,
                                                                       boost::bind(&Mission_Trigger_t::feed, &mission_trigger_data, _1),
                                                                       ros::VoidConstPtr(),
                                                                       ros::TransportHints().tcpNoDelay());
        state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, boost::bind(&State_Data_t::feed, &state_data, _1));

        extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10,
                                                                      boost::bind(&ExtendedState_Data_t::feed, &extended_state_data, _1));

        // queue=1 只处理最新帧: 单线程 spinner 下大队列会积压陈旧数据放大控制延迟;
        // roscpp 缓冲满丢旧保新。(0 是无限队列, 切勿改成 0)
        odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1,
                                                    boost::bind(&Odom_Data_t::feed, &odom_data, _1),
                                                    ros::VoidConstPtr(),
                                                    ros::TransportHints().tcpNoDelay());

        imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                                 1,
                                                 boost::bind(&Imu_Data_t::feed, &imu_data, _1),
                                                 ros::VoidConstPtr(),
                                                 ros::TransportHints().tcpNoDelay());

        cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("cmd", 1,
                                                                boost::bind(&Command_Data_t::feed, &cmd_data, _1),
                                                                ros::VoidConstPtr(),
                                                                ros::TransportHints().tcpNoDelay());

        if (navigation_monitor_enabled_)
        {
            gps_raw_sub_ = nh.subscribe<mavros_msgs::GPSRAW>(
                "/mavros/gpsstatus/gps1/raw", 1,
                &FSM::gps_raw_callback, this,
                ros::TransportHints().tcpNoDelay());
            estimator_status_sub_ = nh.subscribe<mavros_msgs::EstimatorStatus>(
                "/mavros/estimator_status", 1,
                &FSM::estimator_status_callback, this,
                ros::TransportHints().tcpNoDelay());
        }

        ctrl_pv_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        ctrl_aw_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
        traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
        debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

        arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        service_worker_ = std::thread(&FSM::service_worker_loop, this);

        // 连接检查移入 fsm_timer (与 state feed 同队列串行, 无竞态), init 不再阻塞等待
        current_state = MANUAL;
        fsm_timer_ = nh.createTimer(ros::Duration(1.0 / param.fsmparam.frequency), &FSM::fsm_timer, this);

        // 单线程 spinner, 在全部订阅/timer 注册完成后启动
        fsm_spinner_ = std::make_shared<ros::AsyncSpinner>(1, &fsm_queue_);
        fsm_spinner_->start();

        ROS_INFO("[FSM]: odom_source=%d (0=world linear velocity, 1=body->world conversion), online thrust estimator=%s",
                 param.odom_source, param.thr_map.accurate_thrust_model ? "ENABLED" : "DISABLED");
        ROS_INFO("[FSM]: RTK/EKF monitor=%s (min_fix=%d, timeout=%.2fs, grace=%.2fs, auto_land=%.2fs)",
                 navigation_monitor_enabled_ ? "ENABLED" : "DISABLED",
                 rtk_fixed_min_type_, navigation_status_timeout_, navigation_failure_grace_,
                 navigation_auto_land_timeout_);
        ROS_INFO("\033[32m[FSM]:Init completed, change to MANUAL state!\033[32m");
    }

    void FSM::onInit(void)
    {
        // init() 已无阻塞, 同步执行即可; 也避免后台线程未 join 导致 std::terminate
        ros::NodeHandle nh(getMTPrivateNodeHandle());
        init(nh);
    }

    bool FSM::odom_is_received(const ros::Time &now_time)
    {
        const double age = (now_time - odom_data.rcv_stamp).toSec();
        return !odom_data.rcv_stamp.isZero() &&
               std::isfinite(age) && age >= 0.0 &&
               age < param.msg_timeout.odom;
    }

    bool FSM::cmd_is_received(const ros::Time &now_time)
    {
        const double age = (now_time - cmd_data.rcv_stamp).toSec();
        const bool fresh = !cmd_data.rcv_stamp.isZero() &&
                           std::isfinite(age) && age >= 0.0 &&
                           age < param.msg_timeout.cmd;
        if (fresh &&
            !cmd_data.isReadyForTrajectory(expected_trajectory_id_))
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[FSM]: ignoring PositionCommand with trajectory_id=%u flag=%u; expected READY id=%u.",
                static_cast<unsigned int>(cmd_data.trajectory_id),
                static_cast<unsigned int>(cmd_data.trajectory_flag),
                static_cast<unsigned int>(expected_trajectory_id_));
            return false;
        }
        return fresh;
    }

    // IMU 新鲜度看门狗: 断流时 false, fsm_timer 停止使用旧数据并进入紧急 AUTO.LAND 路径。
    // 使用独立的 msg_timeout.imu 门限。
    bool FSM::imu_is_received(const ros::Time &now_time)
    {
        const double age = (now_time - imu_data.rcv_stamp).toSec();
        return !imu_data.rcv_stamp.isZero() &&
               std::isfinite(age) && age >= 0.0 &&
               age < param.msg_timeout.imu;
    }

    bool FSM::state_is_received(const ros::Time &now_time)
    {
        const double age = (now_time - state_data.rcv_stamp).toSec();
        return !state_data.rcv_stamp.isZero() &&
               std::isfinite(age) && age >= 0.0 &&
               age < param.msg_timeout.state;
    }

    bool FSM::extended_state_is_received(const ros::Time &now_time)
    {
        const double age = (now_time - extended_state_data.rcv_stamp).toSec();
        return !extended_state_data.rcv_stamp.isZero() &&
               std::isfinite(age) && age >= 0.0 &&
               age < param.msg_timeout.extended_state;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ctrl_node::FSM, nodelet::Nodelet);
