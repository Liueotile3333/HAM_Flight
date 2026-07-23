#include "controller.hpp"

namespace Controller
{

    /*************** Position_Controller *****************/
    void Position_Control::init(ctrl_node::Parameter_t &param){
        param_ = param;
    }

    quadrotor_msgs::Px4ctrlDebug Position_Control::calculateControl(const Desired_State_t &des, 
                                                                    const ctrl_node::Odom_Data_t &odom, 
                                                                    Controller_Output_t &u){
    // 
        quadrotor_msgs::Px4ctrlDebug data;
    //    std::cout << "Position Controller" << std::endl;
        return data;
    }


    /*************** Velocity_Controller *****************/
    void Velocity_Control::init(ctrl_node::Parameter_t &param){
        param_ = param;
        input.vel_last.setZero();
        input.yaw_last = 0;
    }


    quadrotor_msgs::Px4ctrlDebug Velocity_Control::calculateControl(const Desired_State_t &des, 
                                                                    const ctrl_node::Odom_Data_t &odom, 
                                                                    Controller_Output_t &u){
    //    std::cout << "Velocity Controller" << std::endl;

        Eigen::Vector3d Kp(param_.gain.vel.Kvp0, param_.gain.vel.Kvp1, param_.gain.vel.Kvp2);
        Eigen::Vector3d Kd(param_.gain.vel.Kvd0, param_.gain.vel.Kvd1, param_.gain.vel.Kvd2);
        
        Eigen::Vector3d vel_max(param_.kine_cons.vel_hor_max, param_.kine_cons.vel_hor_max, param_.kine_cons.vel_ver_max);
        Eigen::Vector3d acc_max(param_.kine_cons.acc_hor_max, param_.kine_cons.acc_hor_max, param_.kine_cons.acc_ver_max);
        
        //PD control
        Eigen::Vector3d pos_err(des.p - odom.p);
        u.velocity = Kp.asDiagonal() * pos_err + Kd.asDiagonal()*(pos_err - odom_last_.p);
        odom_last_.p = pos_err;

        //limit vel
        u.velocity = u.velocity.cwiseMax(-vel_max).cwiseMin(vel_max);

        //limit acc
        Eigen::Vector3d vel_err(u.velocity-input.vel_last);
        vel_err = vel_err.cwiseMax(-acc_max).cwiseMin(acc_max);
        u.velocity = input.vel_last + vel_err;
        input.vel_last = u.velocity;
        
        //limit yaw
        // double odomYaw = q2yaw(odom.q);
        double omega_err(des.yaw - input.yaw_last);
        omega_err = std::remainder(omega_err, 2 * M_PI); // ±π 回绕修正(避免 des.yaw 与 yaw_last 跨边界时误差突变为 ~2π)
        omega_err = omega_err > param_.kine_cons.omega_yaw_max?param_.kine_cons.omega_yaw_max:omega_err;
        omega_err = omega_err < -param_.kine_cons.omega_yaw_max?-param_.kine_cons.omega_yaw_max:omega_err;
        u.yaw =  input.yaw_last + omega_err;
        input.yaw_last = u.yaw;

        //debug
        debug_msg_.des_p_x = des.p(0);
        debug_msg_.des_p_y = des.p(1);
        debug_msg_.des_p_z = des.p(2);
        
        debug_msg_.des_v_x = pos_err(0);//(des.p - odom.p)(0);
        debug_msg_.des_v_y = pos_err(1);//(des.p - odom.p)(1);
        debug_msg_.des_v_z = pos_err(2);//(des.p - odom.p)(2);

        debug_msg_.cmd_v_x = u.velocity(0);
        debug_msg_.cmd_v_y = u.velocity(1);
        debug_msg_.cmd_v_z = u.velocity(2);
        
        debug_msg_.des_yaw = u.yaw;

        return debug_msg_;
    }


    /*************** Attitude_Controller *****************/
    void Attitude_Angular_Control::init(ctrl_node::Parameter_t &param, const ros::NodeHandle& nh)
    {
        param_ = param;
        resetThrustMapping();

        // Cache control gains once (perf item 1): avoids rebuilding these vectors
        // from param_.gain.* on every control cycle.
        Kp_  << param_.gain.att_pid.Kp0,  param_.gain.att_pid.Kp1,  param_.gain.att_pid.Kp2;
        Ki_  << param_.gain.att_pid.Ki0,  param_.gain.att_pid.Ki1,  param_.gain.att_pid.Ki2;
        Kd_  << param_.gain.att_pid.Kd0,  param_.gain.att_pid.Kd1,  param_.gain.att_pid.Kd2;
        Trou_ << param_.gain.att_ude.Trou0, param_.gain.att_ude.Trou1, param_.gain.att_ude.Trou2;
        Trou_min_ << param_.gain.att_tvude.Trou_min0, param_.gain.att_tvude.Trou_min1, param_.gain.att_tvude.Trou_min2;
        Trou_max_ << param_.gain.att_tvude.Trou_max0, param_.gain.att_tvude.Trou_max1, param_.gain.att_tvude.Trou_max2;
        t_min_    = param_.gain.att_tvude.t_min;
        t_max_    = param_.gain.att_tvude.t_max;
        t_z_min_  = param_.gain.att_tvude.t_z_min;
        t_z_max_  = param_.gain.att_tvude.t_z_max;
        t_xy_min_ = param_.gain.att_tvude.t_xy_min;
        t_xy_max_ = param_.gain.att_tvude.t_xy_max;
        KAngp_ << param_.gain.att_pid.KAngp0, param_.gain.att_pid.KAngp1, param_.gain.att_pid.KAngp2;
        KAngi_ << param_.gain.att_pid.KAngi0, param_.gain.att_pid.KAngi1, param_.gain.att_pid.KAngi2;
        KAngd_ << param_.gain.att_pid.KAngd0, param_.gain.att_pid.KAngd1, param_.gain.att_pid.KAngd2;
        t_att_min_ = param_.gain.att_tvude.t_att_min;
        t_att_max_ = param_.gain.att_tvude.t_att_max;
        Tatt_ << param_.gain.att_ude.Tatt0, param_.gain.att_ude.Tatt1, param_.gain.att_ude.Tatt2;
        Tatt_min_ << param_.gain.att_tvude.Tatt_min0, param_.gain.att_tvude.Tatt_min1, param_.gain.att_tvude.Tatt_min2;
        Tatt_max_ << param_.gain.att_tvude.Tatt_max0, param_.gain.att_tvude.Tatt_max1, param_.gain.att_tvude.Tatt_max2;

        last_eul_err.setZero();

        // 保存节点句柄
        nh_ = nh;
        // 初始化订阅的变量
        time_diff_ = 0.0;
        time_future_ = 0.0;
        current_position_.setZero();
        current_velocity_.setZero();
        future_position_.setZero();
        future_velocity_.setZero();
        initial_velocity_.setZero();
        // 初始化航向角相关变量
        last_yaw_ = 0.0;
        is_first_in_calculate_yaw = true;

        // 直接在这里初始化订阅者，而不是在单独的函数中
        time_diff_sub_ = nh_.subscribe<std_msgs::Float64>("/drone0/sim_odom/time_diff", 1, 
                        &Attitude_Angular_Control::timeDiffCallback, this);
        time_future_sub_ = nh_.subscribe<std_msgs::Float64>("/drone0/sim_odom/time_future", 1, 
                        &Attitude_Angular_Control::timefutureCallback, this);
        current_position_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/drone0/sim_odom/current_position", 1,
                        &Attitude_Angular_Control::currentPositionCallback, this);
        current_velocity_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/drone0/sim_odom/current_velocity", 1,
                        &Attitude_Angular_Control::currentvelocityCallback, this);
        future_position_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/drone0/sim_odom/future_position", 1,
                        &Attitude_Angular_Control::futurePositionCallback, this);
        future_velocity_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/drone0/sim_odom/future_velocity", 1,
            &Attitude_Angular_Control::futurevelocityCallback, this);
        initial_velocity_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/drone0/sim_odom/initial_velocity", 1,
                        &Attitude_Angular_Control::initialvelocityCallback, this);
    }

    // 回调函数
    void Attitude_Angular_Control::timeDiffCallback(const std_msgs::Float64::ConstPtr& msg)
    {    time_diff_ = msg->data;    }
    void Attitude_Angular_Control::timefutureCallback(const std_msgs::Float64::ConstPtr& msg)
    {    time_future_ = msg->data;    }
    void Attitude_Angular_Control::currentPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {    current_position_ << msg->point.x, msg->point.y, msg->point.z;    }
    void Attitude_Angular_Control::currentvelocityCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {    current_velocity_ << msg->point.x, msg->point.y, msg->point.z;    }
    void Attitude_Angular_Control::futurePositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {    future_position_ << msg->point.x, msg->point.y, msg->point.z;    }
    void Attitude_Angular_Control::futurevelocityCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {    future_velocity_ << msg->point.x, msg->point.y, msg->point.z;    }
    void Attitude_Angular_Control::initialvelocityCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {    initial_velocity_ << msg->point.x, msg->point.y, msg->point.z;    }

    // 获取值
    double Attitude_Angular_Control::getTimeDiff() const {    return time_diff_;    }
    double Attitude_Angular_Control::getTimefuture() const {    return time_future_;    }
    Eigen::Vector3d Attitude_Angular_Control::getCurrentPosition() const {    return current_position_;     }
    Eigen::Vector3d Attitude_Angular_Control::getCurrentvelocity() const {    return current_velocity_;    }
    Eigen::Vector3d Attitude_Angular_Control::getfuturePosition() const { return future_position_; }
    Eigen::Vector3d Attitude_Angular_Control::getfuturevelocity() const { return future_velocity_; }
    Eigen::Vector3d Attitude_Angular_Control::getinitialvelocity() const {    return initial_velocity_;    }


    quadrotor_msgs::Px4ctrlDebug Attitude_Angular_Control::calculateControl(const Desired_State_t &des,
                                                                const ctrl_node::Mission_Triger_t& mission,
                                                                const ctrl_node::Odom_Data_t& odom,
                                                                const ctrl_node::Imu_Data_t &imu, 
                                                                Controller_Output_t& u,
                                                                ros::Time &t) {
        /*                               */
        //    std::cout << "Attitude Controller" << std::endl;
            if(is_first_in_control){
                last_time = t;
                init_t = t;
                init_state = odom.v;                
            }
        /*                                              */
            Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
            Eigen::Vector3d T_pose(0.0, 0.0, 0.0);
            Eigen::Vector3d Tdot_pose(0.0, 0.0, 0.0);
            const Eigen::Vector3d& Trou = Trou_; // Pose-UDE参数

            const double& t_min = t_min_;
            const double& t_max = t_max_;
            const double& t_z_min = t_z_min_;
            const double& t_z_max = t_z_max_;
            const double& t_xy_min = t_xy_min_;
            const double& t_xy_max = t_xy_max_;

            const Eigen::Vector3d& Trou_min = Trou_min_;
            const Eigen::Vector3d& Trou_max = Trou_max_;

            const Eigen::Vector3d& Kp = Kp_;
            const Eigen::Vector3d& Ki = Ki_;
            const Eigen::Vector3d& Kd = Kd_;


            des_acc = des.a + Kd.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);

            Eigen::Vector3d d_acc = Eigen::Vector3d(0.0, 0.0, 0.0);            
            Eigen::Vector3d d_acc_pid = Eigen::Vector3d(0.0, 0.0, 0.0);
            Eigen::Vector3d d_acc_sat = Eigen::Vector3d(0.0, 0.0, 0.0);


            //0:UDE 1:TVUDE 2:PD 3:TTVUDE 4:PID
            if (param_.estimator_type == 1)//1:TVUDE
            {
                if (mission.triger_TTVUDE){
					if (is_first_in_pubtriger) {
						init_TTV_t = t;
						init_taj_state = odom.v; 
						is_first_in_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) {
						u0_integral_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_control = false;
					}
					else{
					
						double t_rel = t.toSec() - init_TTV_t.toSec();
						double dt    = t.toSec() - last_time.toSec();
						Eigen::Vector3d T = gettimevaryingT(t_rel, t_min, t_max, Trou_min, Trou_max);
						Eigen::Vector3d T_dot = gettimevaryingTdot(t_rel, t_min, t_max, Trou_min, Trou_max);
						// 1
						d_acc(0) = odom.v(0) / T(0);
						d_acc(1) = odom.v(1) / T(1);  
						d_acc(2) = odom.v(2) / T(2);
						// 2
						Eigen::Vector3d T_init = gettimevaryingT(0, t_min, t_max, Trou_min, Trou_max);
						d_acc(0) -= init_taj_state(0) / T_init(0);
						d_acc(1) -= init_taj_state(1) / T_init(1);
						d_acc(2) -= init_taj_state(2) / T_init(2);                    
						// 3                    
						u0_integral_pos(0) +=  (odom.v(0) * T_dot(0) + des_acc(0) / T(0)) * dt;
						u0_integral_pos(1) +=  (odom.v(1) * T_dot(1) + des_acc(1) / T(1)) * dt;
						u0_integral_pos(2) +=  (odom.v(2) * T_dot(2) + des_acc(2) / T(2)) * dt;
						// 
						d_acc(0) -= u0_integral_pos(0);
						d_acc(1) -= u0_integral_pos(1);
						d_acc(2) -= u0_integral_pos(2);
						// 
						T_pose(0) = T(0);
						T_pose(1) = T(1);
						T_pose(2) = T(2);
						//
						Tdot_pose(0) = T_dot(0);
						Tdot_pose(1) = T_dot(1);
						Tdot_pose(2) = T_dot(2);
						// 
						last_time = t;
					}
                }
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_acc(0) = 0;
					d_acc(1) = 0;
					d_acc(2) = 0;
					last_time = t;
                }
            }
			else if (param_.estimator_type == 0)//0:UDE
            {
                if (mission.triger_TTVUDE){
					if (is_first_in_pubtriger) {
						init_taj_state = odom.v; 
						is_first_in_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) {
						u0_integral_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_control = false;
					}
					else{
					
						u0_integral_pos += des_acc * (t.toSec() - last_time.toSec());
						// 
						d_acc = odom.v - init_taj_state - u0_integral_pos;
						// 
						d_acc(0) = d_acc(0) / Trou(0);
						d_acc(1) = d_acc(1) / Trou(1);
						d_acc(2) = d_acc(2) / Trou(2);
						//  
						T_pose(0) = Trou(0);
						T_pose(1) = Trou(1);
						T_pose(2) = Trou(2);
						//  
						last_time = t;
					}
                }
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_acc(0) = 0;
					d_acc(1) = 0;
					d_acc(2) = 0;
					last_time = t;
                }
            }
			else if (param_.estimator_type == 2)//2:PD
            {
                d_acc(0) = 0;
				d_acc(1) = 0;
				d_acc(2) = 0;
				last_time = t; 
            }
			else if (param_.estimator_type == 3)//3:TTVUDE
            {
                if (mission.triger_TTVUDE){
					if (is_first_in_pubtriger) {
						init_TTV_t = t;
						init_taj_state = odom.v; 
						is_first_in_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) {
						u0_integral_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_control = false;
					}
					else{
					
                        double t_rel = t.toSec() - init_TTV_t.toSec();
                        double dt    = t.toSec() - last_time.toSec();
						Eigen::Vector3d T = gettimevaryingT(t_rel, t_xy_min, t_xy_max, Trou_min, Trou_max);
                        Eigen::Vector3d T_dot = gettimevaryingTdot(t_rel, t_xy_min, t_xy_max, Trou_min, Trou_max);
                        double T_z = gettimevaryingT_z(t_rel, t_z_min, t_z_max, Trou_min(2), Trou_max(2));
                        double T_z_dot = gettimevaryingT_z_dot(t_rel, t_z_min, t_z_max, Trou_min(2), Trou_max(2));
                        // 1
                        d_acc(0) = odom.v(0) / T(0);
                        d_acc(1) = odom.v(1) / T(1); 
                        d_acc(2) = odom.v(2) / T_z;
                        // 2
                        Eigen::Vector3d T_init = gettimevaryingT(0, t_xy_min, t_xy_max, Trou_min, Trou_max);
                        double T_z_init = gettimevaryingT_z(0, t_z_min, t_z_max, Trou_min(2), Trou_max(2));
                        d_acc(0) -= init_taj_state(0) / T_init(0);
                        d_acc(1) -= init_taj_state(1) / T_init(1);
                        d_acc(2) -= init_taj_state(2) / T_z_init;
                        // 3
                        u0_integral_pos(0) +=  (odom.v(0) * T_dot(0) + des_acc(0) / T(0)) * dt;
                        u0_integral_pos(1) +=  (odom.v(1) * T_dot(1) + des_acc(1) / T(1)) * dt;
                        u0_integral_pos(2) +=  (odom.v(2) * T_z_dot + des_acc(2) / T_z) * dt;
                        //
                        d_acc(0) -= u0_integral_pos(0);
                        d_acc(1) -= u0_integral_pos(1);
                        d_acc(2) -= u0_integral_pos(2);
                        //
                        T_pose(0) = T(0);
                        T_pose(1) = T(1);
                        T_pose(2) = T(2);
                        // 
                        last_time = t;
					}
                }
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_acc(0) = 0;
					d_acc(1) = 0;
					d_acc(2) = 0;
					last_time = t;
                }
            }
            else if (param_.estimator_type == 4)//4:PID 抗饱和
            {
                if (mission.triger_TTVUDE){
					if (is_first_in_pubtriger) {
						init_taj_state = odom.v; 
						is_first_in_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) {
                        pos_err_integral_ = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_control = false;
					}
					else{					
						double dt = (t - last_time).toSec();
                        if (dt <= 0.0) dt = 0.01;

                        /* ---------- 位置环 PID + 抗饱和 ---------- */
                        Eigen::Vector3d pos_err = des.p - odom.p;
                        pos_err_integral_ += pos_err * dt;
                        // 积分项取负号:后续 des_acc 执行 des_acc -= d_acc,
                        // 故此处令 d_acc_pid = -Ki*∫pos_err,使积分项以 +Ki*∫e 并入 des_acc,
                        // 与 Kp*e、Kd*ė 同号(否则积分正反馈会导致悬停持续漂移)。
                        d_acc_pid(0) -= Ki(0)*pos_err_integral_(0);
                        d_acc_pid(1) -= Ki(1)*pos_err_integral_(1);
                        d_acc_pid(2) -= Ki(2)*pos_err_integral_(2);
                        // === 对 des_acc_pid 做限幅（作为饱和对象，用你现有的加速度约束）===
                        Eigen::Vector3d acc_max(param_.kine_cons.acc_hor_max,
                                                param_.kine_cons.acc_hor_max,   
                                                param_.kine_cons.acc_ver_max);
                        Eigen::Vector3d d_acc_sat = d_acc_pid.cwiseMax(-acc_max).cwiseMin(acc_max);

                        // === 反算式抗饱和（back-calculation）：对积分器回灌修正 ===
                        // I_dot += Kaw * (u_sat - u_unsat)
                        // Eigen::Vector3d back = Kaw_pos.asDiagonal() * (des_acc_sat - des_acc_pid);
                        // integ_pos_err_ += back * dt;

                        // === 本分支下，d_acc 视为0（与 PD 类似不估计扰动）===
                        // d_acc.setZero();

                        // === 用饱和后的 des_acc ===
                        // d_acc = d_acc_sat;
                        d_acc(0) = d_acc_sat(0);
                        d_acc(1) = d_acc_sat(1);
                        d_acc(2) = d_acc_sat(2);
                        // 其后的 +g、thrust 计算保持原样（在分支外已有）
                        last_time = t;
					}
                }
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_acc(0) = 0;
					d_acc(1) = 0;
					d_acc(2) = 0;
					last_time = t;
                }              
                
            }

            des_acc -= d_acc;
            
            des_acc += Eigen::Vector3d(0, 0, param_.gra);

            u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
            double roll,pitch;
            double yaw_odom = q2yaw(odom.q);
            double sin = std::sin(yaw_odom);
            double cos = std::cos(yaw_odom);
            roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
            pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;

            //  "*******在YAW ANGLE 控制计算中可以使用订阅到的数据*******"
            double current_time = getTimeDiff();
            double future_time  = getTimefuture();        
        //    std::cout << "**当下轨迹时间**=" << current_time << std::endl;                
        //    std::cout << "**前瞻轨迹时间**=" << future_time  << std::endl;
            Eigen::Vector3d current_pose = getCurrentPosition();
            Eigen::Vector3d future_pose  = getfuturePosition();
            Eigen::Vector3d current_velo = getCurrentvelocity();
            Eigen::Vector3d future_velo = getfuturevelocity();
            Eigen::Vector3d initial_velo = getinitialvelocity();
            //  "*******基于速度方向进行 YAW ANGLE 计算(比位置差更平滑,延迟小)*******"
            double des_trajectory_yaw = calculate_yaw_velo(current_time, future_velo, initial_velo);
        //    std::cout << "YAW ANGLE 计算为：" << des_trajectory_yaw << std::endl;

            // yaw_imu = q2yaw(imu.q);
            Eigen::Quaterniond q = Eigen::AngleAxisd(des_trajectory_yaw,Eigen::Vector3d::UnitZ())
              * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
              * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
            // Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
            //     * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
            //     * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
            u.q = imu.q * odom.q.inverse() * q;
            u.bodyrates.z() = last_des_yaw_rate_; // yaw_rate 前馈:消除恒速跟踪稳态误差

        /*                        */
        debug_msg_.des_p_x = des.p(0);
        debug_msg_.des_p_y = des.p(1);
        debug_msg_.des_p_z = des.p(2);
        
        debug_msg_.des_v_x = des.v(0);
        debug_msg_.des_v_y = des.v(1);
        debug_msg_.des_v_z = des.v(2);

        debug_msg_.odom_v_x = odom.v(0);
        debug_msg_.odom_v_y = odom.v(1);
        debug_msg_.odom_v_z = odom.v(2);

        debug_msg_.odom_p_x = odom.p(0);
        debug_msg_.odom_p_y = odom.p(1);
        debug_msg_.odom_p_z = odom.p(2);

        debug_msg_.des_a_x = des_acc(0);
        debug_msg_.des_a_y = des_acc(1);
        debug_msg_.des_a_z = des_acc(2);

        debug_msg_.des_q_x = u.q.x();
        debug_msg_.des_q_y = u.q.y();
        debug_msg_.des_q_z = u.q.z();
        debug_msg_.des_q_w = u.q.w();

        debug_msg_.des_yaw = des_trajectory_yaw;

        debug_msg_.des_thr = u.thrust;
        
        // Used for thrust-accel mapping estimation
        timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
        while (timed_thrust_.size() > 100)
        {
            timed_thrust_.pop();
        }
        return debug_msg_;
    }

    quadrotor_msgs::Px4ctrlDebug Attitude_Angular_Control::calculateControlCMD(const Desired_State_t &des,
                                                                                const ctrl_node::Mission_Triger_t& mission,
                                                                                const ctrl_node::Odom_Data_t& odom,
                                                                                const ctrl_node::Imu_Data_t &imu, 
                                                                                Controller_Output_t &u,
                                                                                ros::Time &t){   
        /*                               */
        //    std::cout << "Angular Controller" << std::endl;
            if(is_first_in_control){
                last_time = t;
                init_t = t;
                init_state = odom.v;                
            }
        /*                                              */
            Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
            Eigen::Vector3d T_pose(0.0, 0.0, 0.0);
            Eigen::Vector3d Tdot_pose(0.0, 0.0, 0.0);
            // 位置环
            const Eigen::Vector3d& Kp = Kp_;
            const Eigen::Vector3d& Ki = Ki_;
            const Eigen::Vector3d& Kd = Kd_;
            const Eigen::Vector3d& Trou = Trou_; // Pose-UDE参数

            const double& t_min = t_min_;
            const double& t_max = t_max_;
            const double& t_z_min = t_z_min_;
            const double& t_z_max = t_z_max_;
            const double& t_xy_min = t_xy_min_;
            const double& t_xy_max = t_xy_max_;

            const Eigen::Vector3d& Trou_min = Trou_min_;
            const Eigen::Vector3d& Trou_max = Trou_max_;
            
            Eigen::Vector3d Omg_eso_pos ( param_.gain.att_eso.Omg_eso_pos0, param_.gain.att_eso.Omg_eso_pos1, param_.gain.att_eso.Omg_eso_pos2);
            Eigen::Vector3d Beta_x ( 2 * Omg_eso_pos(0), 2 * Omg_eso_pos(0) * Omg_eso_pos(0), Omg_eso_pos(0) * Omg_eso_pos(0) * Omg_eso_pos(0));
            Eigen::Vector3d Beta_y ( 2 * Omg_eso_pos(1), 2 * Omg_eso_pos(1) * Omg_eso_pos(1), Omg_eso_pos(1) * Omg_eso_pos(1) * Omg_eso_pos(1));
            Eigen::Vector3d Beta_z ( 2 * Omg_eso_pos(2), 2 * Omg_eso_pos(2) * Omg_eso_pos(2), Omg_eso_pos(2) * Omg_eso_pos(2) * Omg_eso_pos(2));

            
            des_acc = des.a + Kd.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);

            Eigen::Vector3d d_acc = Eigen::Vector3d(0.0, 0.0, 0.0);            
            Eigen::Vector3d d_acc_pid = Eigen::Vector3d(0.0, 0.0, 0.0);
            Eigen::Vector3d d_acc_sat = Eigen::Vector3d(0.0, 0.0, 0.0);


            //0:UDE 1:TVUDE 2:PD 3:TTVUDE 4:PID
            if (param_.estimator_type == 1)//1:TVUDE
            {
                if (mission.triger_TTVUDE){
					if (is_first_in_pubtriger) {
						init_TTV_t = t;
						init_taj_state = odom.v; 
						is_first_in_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) {
						u0_integral_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_control = false;
					}
					else{
					
						double t_rel = t.toSec() - init_TTV_t.toSec();
						double dt    = t.toSec() - last_time.toSec();
						Eigen::Vector3d T = gettimevaryingT(t_rel, t_min, t_max, Trou_min, Trou_max);
						Eigen::Vector3d T_dot = gettimevaryingTdot(t_rel, t_min, t_max, Trou_min, Trou_max);
						// 1
						d_acc(0) = odom.v(0) / T(0);
						d_acc(1) = odom.v(1) / T(1);  
						d_acc(2) = odom.v(2) / T(2);
						// 2
						Eigen::Vector3d T_init = gettimevaryingT(0, t_min, t_max, Trou_min, Trou_max);
						d_acc(0) -= init_taj_state(0) / T_init(0);
						d_acc(1) -= init_taj_state(1) / T_init(1);
						d_acc(2) -= init_taj_state(2) / T_init(2);                    
						// 3                    
						u0_integral_pos(0) +=  (odom.v(0) * T_dot(0) + des_acc(0) / T(0)) * dt;
						u0_integral_pos(1) +=  (odom.v(1) * T_dot(1) + des_acc(1) / T(1)) * dt;
						u0_integral_pos(2) +=  (odom.v(2) * T_dot(2) + des_acc(2) / T(2)) * dt;
						// 
						d_acc(0) -= u0_integral_pos(0);
						d_acc(1) -= u0_integral_pos(1);
						d_acc(2) -= u0_integral_pos(2);
						// 
						T_pose(0) = T(0);
						T_pose(1) = T(1);
						T_pose(2) = T(2);
						//
						Tdot_pose(0) = T_dot(0);
						Tdot_pose(1) = T_dot(1);
						Tdot_pose(2) = T_dot(2);
						// 
						last_time = t;
					}
                }
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_acc(0) = 0;
					d_acc(1) = 0;
					d_acc(2) = 0;
					last_time = t;
                }
            }
			else if (param_.estimator_type == 0)//0:UDE
            {
                if (mission.triger_TTVUDE){
					if (is_first_in_pubtriger) {
						init_taj_state = odom.v; 
						is_first_in_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) {
						u0_integral_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_control = false;
					}
					else{
					
						u0_integral_pos += des_acc * (t.toSec() - last_time.toSec());
						// 
						d_acc = odom.v - init_taj_state - u0_integral_pos;
						// 
						d_acc(0) = d_acc(0) / Trou(0);
						d_acc(1) = d_acc(1) / Trou(1);
						d_acc(2) = d_acc(2) / Trou(2);
						//  
						T_pose(0) = Trou(0);
						T_pose(1) = Trou(1);
						T_pose(2) = Trou(2);
						//  
						last_time = t;
					}
                }
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_acc(0) = 0;
					d_acc(1) = 0;
					d_acc(2) = 0;
					last_time = t;
                }
            }
			else if (param_.estimator_type == 2)//2:PD
            {
                d_acc(0) = 0;
				d_acc(1) = 0;
				d_acc(2) = 0;
				last_time = t; 
            }
			else if (param_.estimator_type == 3)//3:TTVUDE
            {
                if (mission.triger_TTVUDE){
					if (is_first_in_pubtriger) {
						init_TTV_t = t;
						init_taj_state = odom.v; 
						is_first_in_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) {
						u0_integral_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_control = false;
					}
					else{
					
                        double t_rel = t.toSec() - init_TTV_t.toSec();
                        double dt    = t.toSec() - last_time.toSec();
						Eigen::Vector3d T = gettimevaryingT(t_rel, t_xy_min, t_xy_max, Trou_min, Trou_max);
                        Eigen::Vector3d T_dot = gettimevaryingTdot(t_rel, t_xy_min, t_xy_max, Trou_min, Trou_max);
                        double T_z = gettimevaryingT_z(t_rel, t_z_min, t_z_max, Trou_min(2), Trou_max(2));
                        double T_z_dot = gettimevaryingT_z_dot(t_rel, t_z_min, t_z_max, Trou_min(2), Trou_max(2));
                        // 1
                        d_acc(0) = odom.v(0) / T(0);
                        d_acc(1) = odom.v(1) / T(1); 
                        d_acc(2) = odom.v(2) / T_z;
                        // 2
                        Eigen::Vector3d T_init = gettimevaryingT(0, t_xy_min, t_xy_max, Trou_min, Trou_max);
                        double T_z_init = gettimevaryingT_z(0, t_z_min, t_z_max, Trou_min(2), Trou_max(2));
                        d_acc(0) -= init_taj_state(0) / T_init(0);
                        d_acc(1) -= init_taj_state(1) / T_init(1);
                        d_acc(2) -= init_taj_state(2) / T_z_init;
                        // 3
                        u0_integral_pos(0) +=  (odom.v(0) * T_dot(0) + des_acc(0) / T(0)) * dt;
                        u0_integral_pos(1) +=  (odom.v(1) * T_dot(1) + des_acc(1) / T(1)) * dt;
                        u0_integral_pos(2) +=  (odom.v(2) * T_z_dot + des_acc(2) / T_z) * dt;
                        //
                        d_acc(0) -= u0_integral_pos(0);
                        d_acc(1) -= u0_integral_pos(1);
                        d_acc(2) -= u0_integral_pos(2);
                        //
                        T_pose(0) = T(0);
                        T_pose(1) = T(1);
                        T_pose(2) = T(2);
                        // 
                        last_time = t;
					}
                }
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_acc(0) = 0;
					d_acc(1) = 0;
					d_acc(2) = 0;
					last_time = t;
                }
            }
            else if (param_.estimator_type == 4)//4:PID 抗饱和
            {
                if (mission.triger_TTVUDE){
					if (is_first_in_pubtriger) {
						init_taj_state = odom.v; 
						is_first_in_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) {
                        pos_err_integral_ = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_control = false;
					}
					else{					
						double dt = (t - last_time).toSec();
                        if (dt <= 0.0) dt = 0.01;

                        /* ---------- 位置环 PID + 抗饱和 ---------- */
                        Eigen::Vector3d pos_err = des.p - odom.p;
                        pos_err_integral_ += pos_err * dt;
                        // 积分项取负号:后续 des_acc 执行 des_acc -= d_acc,
                        // 故此处令 d_acc_pid = -Ki*∫pos_err,使积分项以 +Ki*∫e 并入 des_acc,
                        // 与 Kp*e、Kd*ė 同号(否则积分正反馈会导致悬停持续漂移)。
                        d_acc_pid(0) -= Ki(0)*pos_err_integral_(0);
                        d_acc_pid(1) -= Ki(1)*pos_err_integral_(1);
                        d_acc_pid(2) -= Ki(2)*pos_err_integral_(2);
                        // === 对 des_acc_pid 做限幅（作为饱和对象，用你现有的加速度约束）===
                        Eigen::Vector3d acc_max(param_.kine_cons.acc_hor_max,
                                                param_.kine_cons.acc_hor_max,   
                                                param_.kine_cons.acc_ver_max);
                        Eigen::Vector3d d_acc_sat = d_acc_pid.cwiseMax(-acc_max).cwiseMin(acc_max);

                        // === 反算式抗饱和（back-calculation）：对积分器回灌修正 ===
                        // I_dot += Kaw * (u_sat - u_unsat)
                        // Eigen::Vector3d back = Kaw_pos.asDiagonal() * (des_acc_sat - des_acc_pid);
                        // integ_pos_err_ += back * dt;

                        // === 本分支下，d_acc 视为0（与 PD 类似不估计扰动）===
                        // d_acc.setZero();

                        // === 用饱和后的 des_acc ===
                        // d_acc = d_acc_sat;
                        d_acc(0) = d_acc_sat(0);
                        d_acc(1) = d_acc_sat(1);
                        d_acc(2) = d_acc_sat(2);
                        // 其后的 +g、thrust 计算保持原样（在分支外已有）
                        last_time = t;
					}
                }
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_acc(0) = 0;
					d_acc(1) = 0;
					d_acc(2) = 0;
					last_time = t;
                }              
                
            }

            des_acc -= d_acc;
            des_acc += Eigen::Vector3d(0, 0, param_.gra);
            u.thrust = computeDesiredCollectiveThrustSignal(des_acc);

            /* Obtain initial state-attitude */ 
            if(is_first_in_att_control){
                last_att_time = t;
                init_att_t = t;
                init_att_state = odom.q;//  imu.q;
            }

            double roll, pitch;
            double yaw_odom = q2yaw(odom.q);
            double sin = std::sin(yaw_odom);
            double cos = std::cos(yaw_odom);
            roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;
            pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;
            
            //  "*******在YAW ANGLE 控制计算中可以使用订阅到的数据*******"
            double current_time = getTimeDiff();
            double future_time  = getTimefuture();        
        //    std::cout << "**当下轨迹时间**=" << current_time << std::endl;                
        //    std::cout << "**前瞻轨迹时间**=" << future_time  << std::endl;
            Eigen::Vector3d current_pose = getCurrentPosition();
            Eigen::Vector3d future_pose  = getfuturePosition();
            Eigen::Vector3d current_velo = getCurrentvelocity();
            Eigen::Vector3d future_velo  = getfuturevelocity();
            Eigen::Vector3d initial_velo = getinitialvelocity();
            //  "*******基于速度方向进行 YAW ANGLE 计算(比位置差更平滑,延迟小)*******"
            double des_trajectory_yaw = calculate_yaw_velo(current_time, future_velo, initial_velo);
        //    std::cout << "YAW ANGLE 计算为：" << des_trajectory_yaw << std::endl;

            // yaw_imu = q2yaw(imu.q);
            Eigen::Quaterniond q = Eigen::AngleAxisd(des_trajectory_yaw,Eigen::Vector3d::UnitZ())
              * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
              * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
            // Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
            //     * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
            //     * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
            u.q = imu.q * odom.q.inverse() * q;

            // 姿态环
            const Eigen::Vector3d& KAngp = KAngp_;
            const Eigen::Vector3d& KAngi = KAngi_;
            const Eigen::Vector3d& KAngd = KAngd_;
            
            const double& t_att_min = t_att_min_;
            const double& t_att_max = t_att_max_;
            const Eigen::Vector3d& Tatt = Tatt_; // Att-UDE参数            
            const Eigen::Vector3d& Tatt_min = Tatt_min_;
            const Eigen::Vector3d& Tatt_max = Tatt_max_;

            Eigen::Vector3d eul_err = Eigen::Vector3d(roll - q2roll(odom.q), pitch - q2pitch(odom.q), std::remainder(des_trajectory_yaw - q2yaw(odom.q), 2 * M_PI));
            // Eigen::Vector3d eul_err = Eigen::Vector3d(roll - q2roll(odom.q), pitch - q2pitch(odom.q), des.yaw - q2yaw(odom.q));

            Eigen::Vector3d des_br(0.0, 0.0, 0.0);
            Eigen::Vector3d T_euler(0.0, 0.0, 0.0);
            Eigen::Vector3d Tdot_euler(0.0, 0.0, 0.0);

            des_br = des.omg + KAngp.asDiagonal() * eul_err + KAngd.asDiagonal() * (eul_err - last_eul_err);
            last_eul_err = eul_err;

            // 初始化估计器中的加速度 d_br 为零。
            Eigen::Vector3d d_br = Eigen::Vector3d(0.0, 0.0, 0.0); 
            Eigen::Vector3d d_br_pid = Eigen::Vector3d(0.0, 0.0, 0.0);
            Eigen::Vector3d d_br_sat = Eigen::Vector3d(0.0, 0.0, 0.0);            

            //0:UDE 1:TVUDE 2:PD 3:TTVUDE 4:PID
            // 根据控制器参数param_.estimator_type的值选择不同的扰动估计方法
            if (param_.estimator_type == 1)//1:TVUDE
            {
                if (mission.triger_TTVUDE){
					if (is_first_in_att_pubtriger) {
						init_att_TTV_t = t;
						init_taj_att_state = odom.q; 
						is_first_in_att_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) {
						u0_integral_att = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_att_control = false;
					}
					else{
					
						// 计算时间变化的参数 T 和它的导数 T_dot，用于调整加速度计算。
						double t_rel = t.toSec() - init_att_TTV_t.toSec();
						double dt    = t.toSec() - last_att_time.toSec();
						Eigen::Vector3d T_att = gettimevaryingT(t_rel, t_min, t_max, Tatt_min, Tatt_max);
						Eigen::Vector3d T_att_dot = gettimevaryingTdot(t_rel, t_min, t_max, Tatt_min, Tatt_max);
						// 1
						d_br.x() = q2roll(odom.q) / T_att(0);
						d_br.y() = q2pitch(odom.q) / T_att(1);
						d_br.z() = q2yaw(odom.q) / T_att(2);
						// 2
						Eigen::Vector3d T_att_init = gettimevaryingT(0, t_min, t_max, Tatt_min, Tatt_max);
						d_br.x() -= q2roll(init_att_state) / T_att_init(0);
						d_br.y() -= q2pitch(init_att_state)  / T_att_init(1);
						d_br.z() -= q2yaw(init_att_state)  / T_att_init(2);
						// 3
						//TODO 应该是1/T的导数而不是1/(T的导数)
						u0_integral_att.x() += (q2roll(odom.q) * T_att_dot(0) + des_br.x() / T_att(0)) * dt;
						u0_integral_att.y() += (q2pitch(odom.q) * T_att_dot(1) + des_br.y() / T_att(1)) * dt;
						u0_integral_att.z() += (q2yaw(odom.q) * T_att_dot(2) + des_br.z() / T_att(2)) * dt;
						//
						d_br.x() -= u0_integral_att.x();
						d_br.y() -= u0_integral_att.y();
						d_br.z() -= u0_integral_att.z();
						//
						T_euler(0) = T_att(0);
						T_euler(1) = T_att(1);
						T_euler(2) = T_att(2);
						//
						Tdot_euler(0) = T_att_dot(0);
						Tdot_euler(1) = T_att_dot(1);
						Tdot_euler(2) = T_att_dot(2);
						//
						last_att_time = t;
					}
				}
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_br.x() = 0;
                    d_br.y() = 0;
                    d_br.z() = 0;
                    //
                    last_att_time = t;
                }
            }
			else if (param_.estimator_type == 0)//0:UDE
            {
			    if (mission.triger_TTVUDE){
					if (is_first_in_att_pubtriger) {
						init_att_TTV_t = t;
						init_taj_att_state = odom.q; 
						is_first_in_att_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) {
						u0_integral_att = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_att_control = false;
					}
					else{
					
						 u0_integral_att += des_br * (t.toSec() - last_att_time.toSec());                    
						//
						d_br.x() = q2roll(odom.q) - q2roll(init_att_state) - u0_integral_att.x();
						d_br.y() = q2pitch(odom.q) - q2pitch(init_att_state) -u0_integral_att.y();
						d_br.z() = q2yaw(odom.q) - q2yaw(init_att_state) - u0_integral_att.z();
						//                      
						d_br.x() = d_br.x() / Tatt(0);
						d_br.y() = d_br.y() / Tatt(1);
						d_br.z() = d_br.z() / Tatt(2);                    
						// 
						T_euler(0) = Tatt(0);
						T_euler(1) = Tatt(1);
						T_euler(2) = Tatt(2);
						// 
						last_att_time = t;
					}
				}
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_br.x() = 0;
                    d_br.y() = 0;
                    d_br.z() = 0;
                    //
                    last_att_time = t;
                }
            }
			else if (param_.estimator_type == 2)//2:PD
            {
					d_br.x() = 0;
                    d_br.y() = 0;
                    d_br.z() = 0;
                    //
                    last_att_time = t;
            }
			else if (param_.estimator_type == 3)//3:TTVUDE
            {
                if (mission.triger_TTVUDE){
					if (is_first_in_att_pubtriger) {
						init_att_TTV_t = t;
						init_taj_att_state = odom.q; 
						is_first_in_att_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) {
						u0_integral_att = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_att_control = false;
					}
					else{
					
						// 计算时间变化的参数 T 和它的导数 T_dot，用于调整加速度计算。
                        double t_rel = t.toSec() - init_att_TTV_t.toSec();
                        double dt    = t.toSec() - last_att_time.toSec();
                        Eigen::Vector3d T_att = gettimevaryingT(t_rel, t_att_min, t_att_max, Tatt_min, Tatt_max);
                        Eigen::Vector3d T_att_dot = gettimevaryingTdot(t_rel, t_att_min, t_att_max, Tatt_min, Tatt_max);
                        // 1
                        d_br.x() = q2roll(odom.q) / T_att(0);
                        d_br.y() = q2pitch(odom.q) / T_att(1);
                        d_br.z() = q2yaw(odom.q) / T_att(2);    
                        // 2
                        Eigen::Vector3d T_att_init = gettimevaryingT(0, t_att_min, t_att_max, Tatt_min, Tatt_max);
                        d_br.x() -= q2roll(init_att_state) / T_att_init(0);
                        d_br.y() -= q2pitch(init_att_state)  / T_att_init(1);
                        d_br.z() -= q2yaw(init_att_state)  / T_att_init(2);    
                        // 3
                        //TODO 应该是1/T的导数而不是1/(T的导数)
                        // 累积位置误差，积分位置误差以修正扰动
                        u0_integral_att.x() += (q2roll(odom.q) * T_att_dot(0) + des_br.x() / T_att(0)) * dt;
                        u0_integral_att.y() += (q2pitch(odom.q) * T_att_dot(1) + des_br.y() / T_att(1)) * dt;
                        u0_integral_att.z() += (q2yaw(odom.q) * T_att_dot(2) + des_br.z() / T_att(2)) * dt;
                        // 
                        d_br.x() -= u0_integral_att.x();
                        d_br.y() -= u0_integral_att.y();
                        d_br.z() -= u0_integral_att.z();
                        // 
                        T_euler(0) = T_att(0);
                        T_euler(1) = T_att(1);
                        T_euler(2) = T_att(2);
                        // 
                        last_att_time = t;        
					}
				}
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_br.x() = 0;
                    d_br.y() = 0;
                    d_br.z() = 0;
                    //
                    last_att_time = t;
                }				
            }
            else if (param_.estimator_type == 4) // 4: PID
            {
                if (mission.triger_TTVUDE){
					if (is_first_in_att_pubtriger) {
						init_att_TTV_t = t;
						init_taj_att_state = odom.q; 
						is_first_in_att_pubtriger = false;                            
					}
					//累积误差清零
					if (is_first_in_control) { 
                        eul_integral_ = Eigen::Vector3d(0.0, 0.0, 0.0);
						is_first_in_att_control = false;
					}
					else{
					
						// === 时间步长
                        double dt = (t - last_time).toSec();
                        if (dt <= 0.0) dt = 0.01;
        
                        /* ---------- 姿态角 PID + 抗饱和 ---------- */
                        Eigen::Vector3d eul_att;
                        eul_att << roll  - q2roll(odom.q),
                                pitch - q2pitch(odom.q),
                                des.yaw - q2yaw(odom.q);
        
                        eul_integral_ += eul_att * dt;
        
                        /* PID 输出 */
                        Eigen::Vector3d des_br_raw;
                        // d_br_pid.x() += KAngi(0) * eul_integral_(0);
                        // d_br_pid.y() += KAngi(1) * eul_integral_(1);
                        // d_br_pid.z() += KAngi(2) * eul_integral_(2);
                        d_br_pid += KAngi.asDiagonal() * eul_integral_;
                        /* 限幅：±2 rad/s（可写参数） */
                        Eigen::Vector3d br_max(2.0, 2.0, 2.0);
                        Eigen::Vector3d d_br_sat = d_br_pid.cwiseMax(-br_max).cwiseMin(br_max);
        
                        // === 反算抗饱和：I_dot += Kaw * (u_sat - u_unsat) ===
                        // Eigen::Vector3d back_att = Kaw_att.asDiagonal() * (des_br_sat - des_br_pid);
                        // integ_att_err_ += back_att * dt;
        
                        // === 本分支下，不估计姿态扰动 d_br（与 PD 一致）===
                        // d_br.setZero();
        
                        // === 输出（注意你后面会做 des_br -= d_br; 这里先把 des_br 设成饱和后的）===
                        // d_br = d_br_sat;
                        d_br.x() = d_br_sat.x();
                        d_br.y() = d_br_sat.y();
                        d_br.z() = d_br_sat.z();
                        last_att_time = t;
					}
				}
                else if (!mission.triger_TTVUDE){
					// takeoff used PD
					d_br.x() = 0;
                    d_br.y() = 0;
                    d_br.z() = 0;
                    //
                    last_att_time = t;
                }                
            }
            
            des_br -= d_br;
            u.bodyrates += des_br;

        
        //used for debug
        debug_msg_.des_p_x = des.p(0);
        debug_msg_.des_p_y = des.p(1);
        debug_msg_.des_p_z = des.p(2);
        
        debug_msg_.des_v_x = des.v(0);
        debug_msg_.des_v_y = des.v(1);
        debug_msg_.des_v_z = des.v(2);

        debug_msg_.odom_v_x = odom.v(0);
        debug_msg_.odom_v_y = odom.v(1);
        debug_msg_.odom_v_z = odom.v(2);

        debug_msg_.odom_p_x = odom.p(0);
        debug_msg_.odom_p_y = odom.p(1);
        debug_msg_.odom_p_z = odom.p(2);

        debug_msg_.des_a_x = des_acc(0);
        debug_msg_.des_a_y = des_acc(1);
        debug_msg_.des_a_z = des_acc(2);

        debug_msg_.d_acc_x = d_acc(0);
        debug_msg_.d_acc_y = d_acc(1);
        debug_msg_.d_acc_z = d_acc(2);

        debug_msg_.T_x = T_pose(0);
        debug_msg_.T_y = T_pose(1);
        debug_msg_.T_z = T_pose(2);
        debug_msg_.Tdot_x = Tdot_pose(0);
        debug_msg_.Tdot_y = Tdot_pose(1);
        debug_msg_.Tdot_z = Tdot_pose(2);

        debug_msg_.des_q_x = q.x();
        debug_msg_.des_q_y = q.y();
        debug_msg_.des_q_z = q.z();
        debug_msg_.des_q_w = q.w();

        debug_msg_.cmd_q_x = u.q.x();
        debug_msg_.cmd_q_y = u.q.y();
        debug_msg_.cmd_q_z = u.q.z();
        debug_msg_.cmd_q_w = u.q.w();

        debug_msg_.des_yaw = des_trajectory_yaw;

        debug_msg_.imu_q_x = imu.q.x();
        debug_msg_.imu_q_y = imu.q.y();
        debug_msg_.imu_q_z = imu.q.z();
        debug_msg_.imu_q_w = imu.q.w();

        debug_msg_.des_bodyrate_x = u.bodyrates.x();
        debug_msg_.des_bodyrate_y = u.bodyrates.y();
        debug_msg_.des_bodyrate_z = u.bodyrates.z();
        
        debug_msg_.des_br_x = des_br.x();
        debug_msg_.des_br_y = des_br.y();
        debug_msg_.des_br_z = des_br.z();

        debug_msg_.d_br_x = d_br.x();
        debug_msg_.d_br_y = d_br.y();
        debug_msg_.d_br_z = d_br.z();

        debug_msg_.T_roll = T_euler(0);
        debug_msg_.T_pitch = T_euler(1);
        debug_msg_.T_yaw = T_euler(2);
        debug_msg_.Tdot_roll = Tdot_euler(0);
        debug_msg_.Tdot_pitch = Tdot_euler(1);
        debug_msg_.Tdot_yaw = Tdot_euler(2);

        debug_msg_.des_thr = u.thrust;

        // Used for thrust-accel mapping estimation
        timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
        while (timed_thrust_.size() > 100)
        {
            timed_thrust_.pop();
        }
        return debug_msg_;
    }

    // velocity vector实现方式
    double Attitude_Angular_Control::calculate_yaw_velo(double t_cur, 
                                        Eigen::Vector3d& velo,
                                        Eigen::Vector3d& initial_velo) {
        if (is_first_in_calculate_yaw)
        {
            last_yaw_ = 0.0;
            is_first_in_calculate_yaw = false;
            return last_yaw_;
        }
        // 用速度方向(future_velo)算 yaw:atan2(vy,vx) 直接得到运动航向
        // (原 velo-initial_velo 是速度差,方向错误,已修正为 velo 本身)
        Eigen::Vector3d dirvelo = (t_cur <= 60)
                                ? velo
                                : Eigen::Vector3d::Zero();

    //    std::cout << "dir_velocity=" << dirvelo << std::endl;
        // 阈值判断决定是否更新 yaw
        double yaw_temp = (dirvelo.head<2>().norm()) > 0.05
                            ? atan2(dirvelo(1), dirvelo(0))
                            : last_yaw_;

    //    std::cout << "velo(1)=" << dirvelo(1) << std::endl;
    //    std::cout << "velo(0)=" << dirvelo(0) << std::endl;
    //    std::cout << "velo=" << velo << std::endl;
     //   std::cout << "initial_velo=" << initial_velo << std::endl;

		double d_yaw = yaw_temp - last_yaw_;
		// 处理角度环绕问题(±π)
        if (d_yaw >= M_PI){d_yaw -= 2 * M_PI;}
        if (d_yaw <= -M_PI){d_yaw += 2 * M_PI;} 
		
		// if (fabs(d_yaw) > fabs(param_.kine_cons.omega_yaw_max * 0.02))
        // {   d_yaw = param_.kine_cons.omega_yaw_max;    }
		
        double yaw = last_yaw_ + d_yaw;
        yaw = std::remainder(yaw, 2 * M_PI);
        // if (yaw > M_PI){yaw -= 2 * M_PI;}
        // if (yaw < -M_PI){yaw += 2 * M_PI;}
        // 更新历史值
        last_yaw_ = yaw;

        // yaw_rate 前馈:差分 des_yaw,存成员供 calculateControl 填 body_rate.z (消除恒速跟踪稳态误差)
        if (yaw_rate_inited_) {
            double dyaw_ff = yaw - last_des_yaw_;
            if (dyaw_ff > M_PI) dyaw_ff -= 2 * M_PI;
            if (dyaw_ff < -M_PI) dyaw_ff += 2 * M_PI;
            double dt_ff = t_cur - last_des_yaw_t_;
            last_des_yaw_rate_ = (dt_ff > 1e-4) ? (dyaw_ff / dt_ff) : last_des_yaw_rate_;
        } else {
            last_des_yaw_rate_ = 0.0;
        }
        last_des_yaw_ = yaw;
        last_des_yaw_t_ = t_cur;
        yaw_rate_inited_ = true;

        return yaw;
    }

  

    double Attitude_Angular_Control::gettimevaryingT_z(double const t,
                                                    double const t_min,
                                                    double const t_max,
                                                    double const Tude_min,
                                                    double const Tude_max)
    {
        double T_z;
        if(t <= t_min){
            T_z = Tude_max;
            return T_z;
        }
        if(t > t_max){
            T_z = Tude_min;
            return T_z;
        }

        T_z = (Tude_max - Tude_min) / 2 * cos(M_PI * (t - t_min) / (t_max - t_min)) + (Tude_max + Tude_min) / 2;

        return T_z;
        // Sigmoid function parameters
        // double sigmoid_k = 10.0; // 控制过渡速度
        // double t_center = (t_z_min + t_z_max) / 2.0; // 过渡的中心
        // // 使用Sigmoid函数代替余弦函数
        // double sigmoid = 1.0 / (1.0 + exp(-sigmoid_k * (t - t_center)));
        // T_z = T_z_max -(T_z_max - T_z_min) * sigmoid;
        // return T_z;
    }

    double Attitude_Angular_Control::gettimevaryingT_z_dot(double const t,
                                                    double const t_min,
                                                    double const t_max,
                                                    double const Tude_min,
                                                    double const Tude_max)
    {
        double T_z_dot,c;
        
        // 如果 t 小于等于 t_min 或大于等于 t_max，返回零变化率
        if (t <= t_min || t > t_max) {
            T_z_dot = 0;
            return T_z_dot;
        }

        T_z_dot = - (Tude_max - Tude_min) / 2 * sin(M_PI * (t - t_min) / (t_max - t_min)) * M_PI / (t_max - t_min);
        c = (Tude_max - Tude_min) / 2 * cos(M_PI * (t - t_min) / (t_max - t_min)) + (Tude_max + Tude_min) / 2;
        T_z_dot /= pow(c, 2);

        return T_z_dot;
    }


    Eigen::Vector3d Attitude_Angular_Control::gettimevaryingT(double const t,
                                                    double const t_min,
                                                    double const t_max,
                                                    Eigen::Vector3d const Tude_min,
                                                    Eigen::Vector3d const Tude_max)
    {
        Eigen::Vector3d T;
        if(t <= t_min){
            T = Tude_max;
            return T;
        }
        if(t > t_max){
            T = Tude_min;
            return T;
        }

        T = (Tude_max - Tude_min) / 2 * cos(M_PI * (t - t_min) / (t_max - t_min)) + (Tude_max + Tude_min) / 2;
        return T;
    }

    Eigen::Vector3d Attitude_Angular_Control::gettimevaryingTdot(double const t,
                                                    double const t_min,
                                                    double const t_max,
                                                    Eigen::Vector3d const Tude_min,
                                                    Eigen::Vector3d const Tude_max)
    {
        Eigen::Vector3d Tdot,c;
        
        // 如果 t 小于等于 t_min 或大于等于 t_max，返回零变化率
        if (t <= t_min || t > t_max) {
            Tdot = Eigen::Vector3d(0, 0, 0);
            return Tdot;
        }

        Tdot = - (Tude_max - Tude_min) / 2 * sin(M_PI * (t - t_min) / (t_max - t_min)) * M_PI / (t_max - t_min);
        c = (Tude_max - Tude_min) / 2 * cos(M_PI * (t - t_min) / (t_max - t_min)) + (Tude_max + Tude_min) / 2;
        Tdot(0) /= pow(c(0),2);
        Tdot(1) /= pow(c(1),2);
        Tdot(2) /= pow(c(2), 2);
        
        return Tdot;
    }

    /*
        compute throttle percentage 
    */
    double Attitude_Angular_Control::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d& des_acc)
    {
        double throttle_percentage(0.0);
        
        /* compute throttle, thr2acc has been estimated before */
        throttle_percentage = des_acc(2) / thr2acc_;

        return throttle_percentage;
    }

    bool Attitude_Angular_Control::estimateThrustModel(const Eigen::Vector3d& est_a, const ctrl_node::Parameter_t& param)
    {
        ros::Time t_now = ros::Time::now();
        while (timed_thrust_.size() >= 1)
        {
            // Choose data before 35~45ms ago
            std::pair<ros::Time, double> t_t = timed_thrust_.front();
            double time_passed = (t_now - t_t.first).toSec();
            if (time_passed > 0.045) // 45ms
            {
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            continue;
            }
            if (time_passed < 0.035) // 35ms
            {
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
            }

            /***********************************************************/
            /* Recursive least squares algorithm with vanishing memory */
            /***********************************************************/
            double thr = t_t.second;
            timed_thrust_.pop();
            
            /***********************************/
            /* Model: est_a(2) = thr1acc_ * thr */
            /***********************************/
            double gamma = 1 / (rho2_ + thr * P_ * thr);
            double K = gamma * P_ * thr;
            thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
            P_ = (1 - K * thr) * P_ / rho2_;
            //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
            //fflush(stdout);

            debug_msg_.thr2acc = thr2acc_;
            return true;
        }
        return false;
    }

    void Attitude_Angular_Control::resetThrustMapping()
    {
        thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
        P_ = 1e6;
    }
}