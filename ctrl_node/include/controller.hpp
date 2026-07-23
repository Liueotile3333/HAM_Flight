#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <thread>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <std_msgs/Float64.h>// **新增
#include <geometry_msgs/PointStamped.h>// **新增
#include <cmath>// **新增

#include "input.hpp"
#include "param.hpp"

namespace Controller{

    // template <typename Scalar_t>
    // Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t>& q_) {
    //     Eigen::Quaternion<Scalar_t> q = q_.normalized();

    //     Eigen::Matrix<Scalar_t, 3, 1> ypr;
    //     ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    //     ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
    //     ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

    //     return ypr;
    // }
    inline double q2yaw(const Eigen::Quaterniond &ori)
    {
        double yawresult = atan2(2.0*(ori.x()*ori.y() + ori.w()*ori.z()), 1.0 - 2.0 * (ori.y() * ori.y() + ori.z() * ori.z()));
        return yawresult;
        //return atan2(2.0*(ori.x()*ori.y() + ori.w()*ori.z()), 1.0 - 2.0 * (ori.y() * ori.y() + ori.z() * ori.z()));
    }
    inline double q2roll(const Eigen::Quaterniond& ori)
    {
        double rollresult = atan2(2.0 * (ori.w() * ori.x() + ori.y() * ori.z()), 1.0 - 2.0 * (ori.x() * ori.x() + ori.y() * ori.y()));
        return rollresult;
        //return atan2(2.0 * (ori.w() * ori.x() + ori.y() * ori.z()), 1.0 - 2.0 * (ori.x() * ori.x() + ori.y() * ori.y()));
    }
    inline double q2pitch(const Eigen::Quaterniond& ori)
    {
    //     double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    //     if (std::abs(sinp) >= 1)
    //         euler(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    //     else
    //         euler(1) = std::asin(sinp);
        double sinp = +2.0 * (ori.w() * ori.y() - ori.z() * ori.x());
        double pitchresult;
        if (std::abs(sinp) >= 1) {
            //return copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            pitchresult = copysign(M_PI / 2, sinp);            
        }            
        else {
            //return asin(2.0 * (ori.w() * ori.y() - ori.z() * ori.x()));
            pitchresult = asin(2.0 * (ori.w() * ori.y() - ori.z() * ori.x()));
        }
        return pitchresult;
    }

    struct Desired_State_t{
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Vector3d a;
        Eigen::Vector3d j;
        Eigen::Quaterniond q;
        Eigen::Vector3d omg;
        double yaw;
        double yaw_rate;

        Desired_State_t(){};

        Desired_State_t(ctrl_node::Odom_Data_t &odom):
            p(odom.p),
            v(Eigen::Vector3d::Zero()),
            a(Eigen::Vector3d::Zero()),
            j(Eigen::Vector3d::Zero()),
            q(odom.q),
            omg(Eigen::Vector3d::Zero()),
            yaw(q2yaw(odom.q)),
            yaw_rate(0){};
    };

    struct Controller_Output_t
    {
        /* position and velocity controller output */
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d pos_last;
        Eigen::Vector3d vel_last;
        double yaw, yaw_last;

        /* attitude and rate controller output */
        Eigen::Quaterniond q; // Orientation of the body frame with respect to the world frame
        Eigen::Vector3d bodyrates; // Body rates in body frame, [rad/s]
        double thrust; // Collective mass normalized thrust
    };

    class Position_Control{
        private:
        ctrl_node::Parameter_t param_;
        quadrotor_msgs::Px4ctrlDebug debug_msg_;

        public:
        void init(ctrl_node::Parameter_t &param);
        quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des, 
                                                      const ctrl_node::Odom_Data_t &odom, 
                                                      Controller_Output_t &u);
    };

    class Velocity_Control{
        private:
        ctrl_node::Parameter_t param_;
        quadrotor_msgs::Px4ctrlDebug debug_msg_;
        ctrl_node::Odom_Data_t odom_last_;
        Controller_Output_t input;

        double get_vel_err(const Desired_State_t &des, const ctrl_node::Odom_Data_t &odom);

        public:
        void init(ctrl_node::Parameter_t &param);
        quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des, 
                                                      const ctrl_node::Odom_Data_t &odom, 
                                                      Controller_Output_t &u);
    };

    class Attitude_Angular_Control{
        private:
        ctrl_node::Parameter_t param_;
        quadrotor_msgs::Px4ctrlDebug debug_msg_;
        std::queue<std::pair<ros::Time, double>> timed_thrust_;
        static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;
        Eigen::Vector3d last_eul_err;
        // Thrust-accel mapping params
        const double rho2_ = 0.998; // do not change
        double thr2acc_;
        double P_;

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

        double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d& des_acc);

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
        ros::Subscriber current_position_sub_;
        ros::Subscriber current_velocity_sub_;
        ros::Subscriber time_future_sub_;
        ros::Subscriber future_position_sub_;
        ros::Subscriber future_velocity_sub_;
        ros::Subscriber initial_velocity_sub_;

        double time_diff_;
        double time_future_;
        Eigen::Vector3d current_position_;
        Eigen::Vector3d current_velocity_;
        Eigen::Vector3d future_position_;
        Eigen::Vector3d future_velocity_;
        Eigen::Vector3d initial_velocity_;
        // Yaw Angle Calculation

        public:
        void init(ctrl_node::Parameter_t& param, const ros::NodeHandle& nh);
        quadrotor_msgs::Px4ctrlDebug calculateControlCMD(const Desired_State_t& des,
                                                        const ctrl_node::Mission_Triger_t& mission,
                                                        const ctrl_node::Odom_Data_t& odom,
                                                        const ctrl_node::Imu_Data_t &imu, 
                                                        Controller_Output_t &u,
                                                        ros::Time &t);
        quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
                                                        const ctrl_node::Mission_Triger_t& mission,
                                                        const ctrl_node::Odom_Data_t& odom,
                                                        const ctrl_node::Imu_Data_t &imu, 
                                                        Controller_Output_t &u,
                                                        ros::Time &t);
        bool estimateThrustModel(const Eigen::Vector3d &est_v, const ctrl_node::Parameter_t &param);
        void resetThrustMapping();

        // Callback function
        void timeDiffCallback(const std_msgs::Float64::ConstPtr& msg);
        void currentPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void currentvelocityCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void timefutureCallback(const std_msgs::Float64::ConstPtr& msg);
        void futurePositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void futurevelocityCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void initialvelocityCallback(const geometry_msgs::PointStamped::ConstPtr & msg);

        double getTimeDiff() const;
        double getTimefuture() const;
        Eigen::Vector3d getCurrentPosition() const;
        Eigen::Vector3d getCurrentvelocity() const;
        Eigen::Vector3d getfuturePosition() const;
        Eigen::Vector3d getfuturevelocity() const;
        Eigen::Vector3d getinitialvelocity() const;
        // Callback function

        double calculate_yaw_velo(double t_cur,
            Eigen::Vector3d& current_velo,Eigen::Vector3d &initial_velo);

        double gettimevaryingT_z(double const t,
            double const t_min,
            double const t_max,
            double const Tude_min,
            double const Tude_max);
        double gettimevaryingT_z_dot(double const t,
            double const t_min,
            double const t_max,
            double const Tude_min,
            double const Tude_max);

          
        Eigen::Vector3d gettimevaryingT(double const t,
            double const t_min,
            double const t_max,
            Eigen::Vector3d const Tude_min,
            Eigen::Vector3d const Tude_max);
        Eigen::Vector3d gettimevaryingTdot(double const t,
            double const t_min,
            double const t_max,
            Eigen::Vector3d const Tude_min,
            Eigen::Vector3d const Tude_max);

// TTV-UDE Trigger Mechanism
        bool is_first_in_control = true;
        bool is_first_in_att_control = true;
        bool is_first_in_pubtriger = true;
        bool is_first_in_att_pubtriger = true;


        std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);
        std::thread initThread_;
        ros::Subscriber triger_sub_;
        ros::NodeHandle nh;

        void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr)
        {
          triger_received_ = true; // for static platfrom landing
          ROS_INFO("\033[32m[planning]:plan triger received!\033[32m");
        }    


        ros::Time last_time;
        ros::Time last_att_time;
        Eigen::Vector3d u0_integral_pos; //Interference estimation value
        Eigen::Vector3d u0_integral_att;
        Eigen::Vector3d pos_err_integral_; //Interference estimation value
        Eigen::Vector3d eul_integral_;
        Eigen::Vector3d init_state;
        Eigen::Vector3d init_taj_state;
        Eigen::Quaterniond init_att_state;
        Eigen::Quaterniond init_taj_att_state;
        ros::Time init_t;
        ros::Time init_TTV_t;
        ros::Time init_att_t;
        ros::Time init_att_TTV_t;


        // use in postion ESO est
        Eigen::Vector3d pos_x_est;
        Eigen::Vector3d pos_x_est_dot;
        Eigen::Vector3d pos_y_est;
        Eigen::Vector3d pos_y_est_dot;
        Eigen::Vector3d pos_z_est;
        Eigen::Vector3d pos_z_est_dot;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    
}

#endif