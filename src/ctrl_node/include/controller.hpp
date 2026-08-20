#ifndef CTRL_NODE_CONTROLLER_HPP_
#define CTRL_NODE_CONTROLLER_HPP_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <queue>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <geometry_msgs/Vector3Stamped.h>

#include "input.hpp"
#include "param.hpp"
#include "aero_model.hpp"

namespace Controller
{

    inline double q2yaw(const Eigen::Quaterniond &ori)
    {
        double yawresult = atan2(2.0 * (ori.x() * ori.y() + ori.w() * ori.z()), 1.0 - 2.0 * (ori.y() * ori.y() + ori.z() * ori.z()));
        return yawresult;
    }
    inline double q2roll(const Eigen::Quaterniond &ori)
    {
        double rollresult = atan2(2.0 * (ori.w() * ori.x() + ori.y() * ori.z()), 1.0 - 2.0 * (ori.x() * ori.x() + ori.y() * ori.y()));
        return rollresult;
    }
    inline double q2pitch(const Eigen::Quaterniond &ori)
    {
        double sinp = +2.0 * (ori.w() * ori.y() - ori.z() * ori.x());
        double pitchresult;
        if (std::abs(sinp) >= 1)
        {
            pitchresult = copysign(M_PI / 2, sinp);
        }
        else
        {
            pitchresult = asin(2.0 * (ori.w() * ori.y() - ori.z() * ori.x()));
        }
        return pitchresult;
    }

    // 由 q2roll/q2pitch/q2yaw 组装的欧拉角 [roll, pitch, yaw]。必须是这三个函数的薄包装 ——
    // 切勿改用 Eigen 的 eulerAngles(2,1,0): 其象限/符号/万向锁约定不同, 会改变控制律数值。
    // 用于一次计算、多处复用, 避免每周期重复三角运算。
    inline Eigen::Vector3d q2euler(const Eigen::Quaterniond &ori)
    {
        return Eigen::Vector3d(q2roll(ori), q2pitch(ori), q2yaw(ori));
    }

    struct Desired_State_t
    {
        // 全部成员声明时初始化: 即便走空默认构造(各 get_*_des 路径), 也得到全零/单位状态,
        // 杜绝 Eigen 成员读到未定义内存。PositionCommand.msg 无三轴角速度字段, omg 默认置零,
        // 仅 yaw_rate 有来源(由 get_cmd_des 填 [0,0,yaw_rate] 作体轴前馈)。
        Eigen::Vector3d p   = Eigen::Vector3d::Zero();
        Eigen::Vector3d v   = Eigen::Vector3d::Zero();
        Eigen::Vector3d a   = Eigen::Vector3d::Zero();
        Eigen::Vector3d j   = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        Eigen::Vector3d omg = Eigen::Vector3d::Zero();
        double yaw      = 0.0;
        double yaw_rate = 0.0;

        Desired_State_t() = default;

        Desired_State_t(ctrl_node::Odom_Data_t &odom) : p(odom.p),
                                                        q(odom.q),
                                                        yaw(q2yaw(odom.q)) {};
    };

    struct Controller_Output_t
    {
        /* position and velocity controller output */
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel_last = Eigen::Vector3d::Zero();
        double yaw = 0.0, yaw_last = 0.0;

        /* attitude and rate controller output */
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity(); // Orientation of the body frame with respect to the world frame
        Eigen::Vector3d bodyrates = Eigen::Vector3d::Zero();   // Body rates in body frame, [rad/s]
        double thrust = 0.0;                                   // Collective mass normalized thrust
    };

    class Position_Control
    {
    private:
        ctrl_node::Parameter_t param_;
        quadrotor_msgs::Px4ctrlDebug debug_msg_;

    public:
        void init(ctrl_node::Parameter_t &param);
        quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
                                                      const ctrl_node::Odom_Data_t &odom,
                                                      Controller_Output_t &u);
    };

    class Velocity_Control
    {
    private:
        ctrl_node::Parameter_t param_;
        quadrotor_msgs::Px4ctrlDebug debug_msg_;
        Controller_Output_t input; // vel_last / yaw_last: 上一拍限幅后输出, 速率限幅基准
        ros::Time last_time_;      // 上一拍时间戳, 用于计算真实控制周期 dt

    public:
        void init(ctrl_node::Parameter_t &param);
        quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
                                                      const ctrl_node::Odom_Data_t &odom,
                                                      Controller_Output_t &u,
                                                      const ros::Time &now);
    };

    class Attitude_Angular_Control
    {
    private:
        ctrl_node::Parameter_t param_;
        AeroModel aero_model_; // 气动(升力翼)前馈模块 + 风场订阅
        quadrotor_msgs::Px4ctrlDebug debug_msg_;
        std::queue<std::pair<ros::Time, double>> timed_thrust_;
        static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;
        Eigen::Vector3d last_eul_err;
        // Thrust-accel mapping params
        const double rho2_ = 0.998; // do not change
        double thr2acc_;
        double P_;

        // PID 改成真正 anti-windup，并统一 dt 防跳变
        bool getSafeControlDt(const ros::Time &now, const ros::Time &last, double &dt) const;

        // Cached control gains: assigned once in init() so we don't rebuild these
        // Eigen vectors / chase param_.gain.* on every control cycle (perf item 1).
        Eigen::Vector3d Kp_, Ki_, Kd_;               // att_pid position-loop gains
        Eigen::Vector3d Trou_;                       // att_ude Pose-UDE param
        Eigen::Vector3d Trou_min_, Trou_max_;        // att_tvude Trou bounds
        double t_min_, t_max_;                       // att_tvude t bounds
        double t_z_min_, t_z_max_;                   // att_tvude z-axis t bounds
        double t_xy_min_, t_xy_max_;                 // att_tvude xy-axis t bounds
        Eigen::Vector3d KAngp_, KAngi_, KAngd_;      // att_pid angular-loop gains
        double t_att_min_, t_att_max_;               // att_tvude attitude t bounds
        Eigen::Vector3d Tatt_, Tatt_min_, Tatt_max_; // att_ude/tvude attitude T params

        // Safety envelope applied to the complete translational acceleration
        // command (feedforward + feedback - disturbance estimate). The returned
        // vector already includes gravity and is safe to use for thrust/attitude.
        Eigen::Vector3d makeSafeTotalAcceleration(const Eigen::Vector3d &acc_cmd, const ctrl_node::Odom_Data_t &odom) const;
        void accelerationToRollPitch(const Eigen::Vector3d &total_acc,
                                     double yaw,
                                     double &roll,
                                     double &pitch) const;
        double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);

        // 位置环共用核心: PD 前馈 + 扰动估计(UDE/TVUDE/PD/TTVUDE/PID) + 安全限幅。
        // pid_integral_always 控制 PID 分支是否在非轨迹段也积分(calculateControl=true,
        // calculateControlCMD=false)。返回含 g 的安全限幅期望总加速度。
        Eigen::Vector3d computePositionLoopAccel(const Desired_State_t &des,
                                                 const ctrl_node::Odom_Data_t &odom,
                                                 const ctrl_node::Mission_Trigger_t &mission,
                                                 const ros::Time &t,
                                                 bool pid_integral_always,
                                                 Eigen::Vector3d &d_acc,
                                                 Eigen::Vector3d &T_pose,
                                                 Eigen::Vector3d &Tdot_pose);
        // 填充 calculateControl/calculateControlCMD 共用的 debug 字段。
        void fillCommonDebug(const Desired_State_t &des,
                             const ctrl_node::Odom_Data_t &odom,
                             const Eigen::Vector3d &des_acc,
                             const Eigen::Quaterniond &des_q,
                             double des_yaw,
                             double thrust);
        // 推力采样记录(仅 accurate_thrust_model 启用), 供 estimateThrustModel。
        void recordThrustSample(double thrust);
        void handleMissionTrigger(const ctrl_node::Mission_Trigger_t &mission,
                                  const ctrl_node::Odom_Data_t &odom,
                                  const ros::Time &now);
        std::uint64_t last_mission_sequence_ = 0U;

        // Yaw Angle Calculation
        double last_yaw_;
        bool is_first_in_calculate_yaw;
        // yaw_rate 前馈差分(工程层 yaw 前馈,消除恒速 yaw 跟踪的稳态误差)
        double last_des_yaw_ = 0.0;
        double last_des_yaw_t_ = 0.0;
        double last_des_yaw_rate_ = 0.0;
        bool yaw_rate_inited_ = false;

        ros::NodeHandle nh_;
        ros::Subscriber time_diff_sub_;
        ros::Subscriber time_future_sub_;
        ros::Subscriber future_velocity_sub_;
        ros::Subscriber initial_velocity_sub_;

        double time_diff_;
        double time_future_;
        Eigen::Vector3d future_velocity_;
        Eigen::Vector3d initial_velocity_;

    public:
        bool init(ctrl_node::Parameter_t &param, const ros::NodeHandle &nh);
        quadrotor_msgs::Px4ctrlDebug calculateControlCMD(const Desired_State_t &des,
                                                         const ctrl_node::Mission_Trigger_t &mission,
                                                         const ctrl_node::Odom_Data_t &odom,
                                                         const ctrl_node::Imu_Data_t &imu,
                                                         Controller_Output_t &u,
                                                         ros::Time &t);
        quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
                                                      const ctrl_node::Mission_Trigger_t &mission,
                                                      const ctrl_node::Odom_Data_t &odom,
                                                      const ctrl_node::Imu_Data_t &imu,
                                                      Controller_Output_t &u,
                                                      ros::Time &t);
        bool estimateThrustModel(const Eigen::Vector3d &est_v, const ctrl_node::Parameter_t &param);
        void resetThrustMapping();

        // Callback function
        void timeDiffCallback(const std_msgs::Float64::ConstPtr &msg);
        void timefutureCallback(const std_msgs::Float64::ConstPtr &msg);
        void futurevelocityCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
        void initialvelocityCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

        double getTimeDiff() const;
        double getTimefuture() const;
        Eigen::Vector3d getfuturevelocity() const;
        Eigen::Vector3d getinitialvelocity() const;

        double calculate_yaw_velo(double t_cur,
                                  Eigen::Vector3d &current_velo, Eigen::Vector3d &initial_velo);

        // TTV-UDE Trigger Mechanism
        bool is_first_in_control = true;
        bool is_first_in_att_control = true;
        bool is_first_in_pubtrigger = true;
        bool is_first_in_att_pubtrigger = true;

        ros::Time last_time;
        ros::Time last_att_time;
        Eigen::Vector3d u0_integral_pos; // Interference estimation value
        Eigen::Vector3d u0_integral_att;
        Eigen::Vector3d pos_err_integral_; // Interference estimation value
        Eigen::Vector3d eul_integral_;
        Eigen::Vector3d init_taj_state;
        Eigen::Vector3d init_att_euler_ = Eigen::Vector3d::Zero(); // 初始姿态欧拉缓存(首拍算一次, 任务期间复用)
        ros::Time init_TTV_t;
        ros::Time init_att_TTV_t;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

#endif  // CTRL_NODE_CONTROLLER_HPP_
