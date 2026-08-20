#include <mutex>
#include <memory>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/bind/bind.hpp>

#include "polyfit.hpp"
#include "ekf.hpp"

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace odomRemap
{

    class odomRemap : public nodelet::Nodelet
    {
    private:
        polyfit::Fit fit;

        std::recursive_mutex mtx;

        ros::Timer odom_timer;

        nav_msgs::Odometry gtruth;

        geometry_msgs::Vector3 gtruth_pos_bias;
        geometry_msgs::Vector3 gtruth_rpy_bias;
        geometry_msgs::Vector3 gtruth_rpy;

        Eigen::Quaterniond gtruth_qua_bias;
        tf::Quaternion gtruth_Q2T;

        sensor_msgs::Imu imu;

        // ============================================================
        // 普通 subscriber
        // ============================================================
        ros::Subscriber gtruthSub; // mocap / simulation 分支使用
        ros::Subscriber imuSub;

        // ============================================================
        // GPS 实飞模式：odom + velocity_local 时间同步
        // ============================================================
        using GpsVelSyncPolicy =
            message_filters::sync_policies::ApproximateTime<
                nav_msgs::Odometry,
                geometry_msgs::TwistStamped>;

        message_filters::Subscriber<nav_msgs::Odometry>
            gpsOdomSyncSub;

        message_filters::Subscriber<geometry_msgs::TwistStamped>
            gpsVelSyncSub;

        std::shared_ptr<
            message_filters::Synchronizer<GpsVelSyncPolicy>>
            gpsVelSynchronizer;

        // 同步器内部队列长度
        int gps_vel_sync_queue_size = 20;

        // odom 与 velocity_local 最大允许时间差
        double gps_vel_max_skew = 0.02;

        // use for simulation
        bool issimulation;

        std::string gtruthTopic;
        std::string imuTopic; // mocap 模式 IMU 来源(mctruth 分支时间对齐/速度积分用)
        std::string modelName; // /gazebo/model_states 中本机模型名(= vehicle_ID, 如 iris_liftwing_0)

        ros::Publisher odomPub;

        std::vector<double> imuvel_x; // imuvel store the unfit data
        std::vector<double> imuvel_y;
        std::vector<double> imuvel_z;
        std::vector<double> imu_t;

        double imu_t0; // coefficient for imu_t return to zero
        double gtruth_time_delay = 0;
        // double gtruth_time_l;

        bool imusubTri = false;
        // bool uavvelsubTri = false;
        bool gtruthsubTri = false;

        // Mocap 路径也必须按“新帧一次性消费 + 接收时间看门狗”工作，
        // 否则 timer 会重复发布缓存 pose/IMU，掩盖上游断流。
        double mocap_input_timeout = 0.20;
        ros::Time last_mocap_rx_time;
        ros::Time last_mocap_imu_rx_time;
        bool new_mocap_frame = false;
        bool new_mocap_imu_frame = false;

        int seq = 0;
        int bias_c = 0;
        int fit_size;

        // ============================================================
        // GPS/RTK odom fresh-frame handling
        // ============================================================
        // 真正用于零点标定的新 odom 样本数
        int gps_bias_samples = 300;
        // /mavros/local_position/odom 多久没有收到新 callback 后认为断流
        double gps_odom_timeout = 0.20;
        // 最后一个已经接受的 MAVROS odom 消息时间戳
        ros::Time last_gtruth_stamp;
        // 最近一次收到真正新 odom 的本机 ROS 时间
        ros::Time last_gtruth_rx_time;
        // 当前缓存的 gtruth 是否包含一帧尚未处理的新 odom
        bool new_gtruth_frame = false;
        // 是否已经完成 GPS/RTK 零点标定
        bool gps_bias_ready = false;

        void mctruthCallback(const geometry_msgs::PoseStamped::ConstPtr &gtruthMsg)
        {
            // MT 回调队列: 本回调与 mc_odom_pub(timer) 跑在 manager 不同 worker 线程,
            // gtruth/ready flag 必须持锁访问(GPS 路径同把 mtx), 否则撕裂帧/数据竞争。
            std::lock_guard<std::recursive_mutex> lock(mtx);

            gtruth.header = gtruthMsg->header;
            gtruth.pose.pose = gtruthMsg->pose;
            last_mocap_rx_time = ros::Time::now();
            new_mocap_frame = true;
            gtruthsubTri = true;
        }

        void gpsOdomVelCallback(
            const nav_msgs::Odometry::ConstPtr &odomMsg,
            const geometry_msgs::TwistStamped::ConstPtr &velMsg)
        {
            std::lock_guard<std::recursive_mutex> lock(mtx);

            // ============================================================
            // 1. 两路消息都必须有有效时间戳
            // ============================================================
            if (odomMsg->header.stamp.isZero())
            {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[odom_remap]: Received local odom with zero timestamp.");

                return;
            }

            if (velMsg->header.stamp.isZero())
            {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[odom_remap]: Received velocity_local with zero timestamp.");

                return;
            }

            // ============================================================
            // 2. 二次检查 odom / velocity_local 时间差
            //
            // message_filters 已经做第一次同步过滤；
            // 这里再检查一次，作为实飞安全防线。
            // ============================================================
            const double time_skew =
                std::fabs(
                    (odomMsg->header.stamp -
                     velMsg->header.stamp)
                        .toSec());

            if (!std::isfinite(time_skew))
            {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[odom_remap]: Invalid odom/velocity timestamp difference.");

                return;
            }

            if (time_skew > gps_vel_max_skew)
            {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[odom_remap]: odom/velocity timestamp mismatch: "
                    "|dt|=%.6f s > %.6f s.",
                    time_skew,
                    gps_vel_max_skew);

                return;
            }

            // ============================================================
            // 3. 只允许新的 odom 时间戳
            // ============================================================
            if (!last_gtruth_stamp.isZero() &&
                odomMsg->header.stamp <= last_gtruth_stamp)
            {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[odom_remap]: Duplicate or out-of-order "
                    "synchronized odom ignored.");

                return;
            }

            // ============================================================
            // 4. 数值合法性检查
            // ============================================================
            const auto &p = odomMsg->pose.pose.position;
            const auto &q = odomMsg->pose.pose.orientation;
            const auto &v = velMsg->twist.linear;
            const auto &w = velMsg->twist.angular;

            if (!std::isfinite(p.x) ||
                !std::isfinite(p.y) ||
                !std::isfinite(p.z) ||
                !std::isfinite(q.w) ||
                !std::isfinite(q.x) ||
                !std::isfinite(q.y) ||
                !std::isfinite(q.z) ||
                !std::isfinite(v.x) ||
                !std::isfinite(v.y) ||
                !std::isfinite(v.z) ||
                !std::isfinite(w.x) ||
                !std::isfinite(w.y) ||
                !std::isfinite(w.z))
            {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[odom_remap]: Non-finite synchronized odom/velocity data.");

                return;
            }

            // ============================================================
            // 5. 构造一帧时间一致的 gtruth
            //
            // pose:
            //   来自 /mavros/local_position/odom
            //
            // twist:
            //   来自时间匹配后的
            //   /mavros/local_position/velocity_local
            // ============================================================
            gtruth.header =
                odomMsg->header;

            gtruth.child_frame_id =
                odomMsg->child_frame_id;

            gtruth.pose =
                odomMsg->pose;

            gtruth.twist.twist =
                velMsg->twist;

            // twist covariance 没有来自 TwistStamped，
            // 因此不能伪造。
            //
            // 如果后续控制器根本不用 covariance，可以保持为零。
            gtruth.twist.covariance.fill(0.0);

            // ============================================================
            // 6. 标记真正得到一帧新的同步定位数据
            // ============================================================
            last_gtruth_stamp =
                odomMsg->header.stamp;

            last_gtruth_rx_time =
                ros::Time::now();

            new_gtruth_frame = true;
            gtruthsubTri = true;

            ROS_DEBUG_THROTTLE(
                1.0,
                "[odom_remap]: synchronized odom/velocity accepted, "
                "|dt|=%.6f s.",
                time_skew);
        }
        void simtruthCallback(const gazebo_msgs::ModelStates::ConstPtr &modelMsg)
        {
            std::lock_guard<std::recursive_mutex> lock(mtx);

            for (int i = 0; i < modelMsg->name.size(); i++)
            {
                if (modelMsg->name[i] == modelName)
                {
                    const ros::Time now = ros::Time::now();

                    gtruth.header.stamp = now;
                    gtruth.pose.pose = modelMsg->pose[i];
                    gtruth.twist.twist = modelMsg->twist[i];

                    last_gtruth_stamp = now;
                    last_gtruth_rx_time = now;

                    new_gtruth_frame = true;
                    gtruthsubTri = true;

                    break;
                }
            }
        }

        void gtruth_const_bias_cal(const double &gtruth_t)
        {
            double current_t = imu.header.stamp.toSec();

            double flash_dur;
            if (bias_c > 0)
            {
                flash_dur = abs(abs(gtruth_t - current_t) - gtruth_time_delay / bias_c);
            }
            else
            {
                flash_dur = abs(abs(gtruth_t - current_t) - gtruth_time_delay);
                gtruth_time_delay = 0; // delete first gtruth_time_delay for total time bias cal
            }

            if (flash_dur < 0.5)
            {
                bias_c += 1;

                gtruth_time_delay = gtruth_time_delay + abs(gtruth_t - current_t);

                gtruth_pos_bias.x = gtruth_pos_bias.x + gtruth.pose.pose.position.x;
                gtruth_pos_bias.y = gtruth_pos_bias.y + gtruth.pose.pose.position.y;
                gtruth_pos_bias.z = gtruth_pos_bias.z + gtruth.pose.pose.position.z;

                tf::quaternionMsgToTF(gtruth.pose.pose.orientation, gtruth_Q2T);
                tf::Matrix3x3(gtruth_Q2T).getRPY(gtruth_rpy.x, gtruth_rpy.y, gtruth_rpy.z);

                gtruth_rpy_bias.x = gtruth_rpy_bias.x + gtruth_rpy.x;
                gtruth_rpy_bias.y = gtruth_rpy_bias.y + gtruth_rpy.y;
                gtruth_rpy_bias.z = gtruth_rpy_bias.z + gtruth_rpy.z;
            }
            else
            {
                gtruth_time_delay = abs(gtruth_t - current_t);
                seq = 0;
                bias_c = 0;
                gtruth_pos_bias = geometry_msgs::Vector3();
                gtruth_rpy_bias = geometry_msgs::Vector3();
                std::cout << "time delay haven't stable" << std::endl;
            }
        }

        // GPS-mode bias calibration (car-related branches removed).
        void gtruth_const_bias_cal()
        {
            bias_c += 1;

            gtruth_pos_bias.x = gtruth_pos_bias.x + gtruth.pose.pose.position.x;
            gtruth_pos_bias.y = gtruth_pos_bias.y + gtruth.pose.pose.position.y;
            gtruth_pos_bias.z = gtruth_pos_bias.z + gtruth.pose.pose.position.z;

            Eigen::Quaterniond q_new(gtruth.pose.pose.orientation.w,
                                     gtruth.pose.pose.orientation.x,
                                     gtruth.pose.pose.orientation.y,
                                     gtruth.pose.pose.orientation.z);

            if (bias_c == 1)
            {
                // 第一帧直接覆盖, 兼修 gtruth_qua_bias 未初始化(Eigen 默认构造为垃圾值)/
                // 标定 reset 未清零的残留, 同时建立后续 hemisphere 对齐的参考方向。
                gtruth_qua_bias = q_new;
            }
            else
            {
                // hemisphere alignment: q 与 -q 表同一姿态, 累加前对齐到当前累加方向,
                // 避免符号翻转的样本逐分量抵消(逐分量平均的经典缺陷)。
                const double dot = gtruth_qua_bias.w() * q_new.w() +
                                   gtruth_qua_bias.x() * q_new.x() +
                                   gtruth_qua_bias.y() * q_new.y() +
                                   gtruth_qua_bias.z() * q_new.z();
                if (dot < 0.0)
                {
                    q_new.w() = -q_new.w();
                    q_new.x() = -q_new.x();
                    q_new.y() = -q_new.y();
                    q_new.z() = -q_new.z();
                }
                gtruth_qua_bias.w() += q_new.w();
                gtruth_qua_bias.x() += q_new.x();
                gtruth_qua_bias.y() += q_new.y();
                gtruth_qua_bias.z() += q_new.z();
            }
        }

        void imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
        {
            // 同 mctruthCallback: imu 整体拷贝非原子, 与 mc_odom_pub 并发须持锁。
            std::lock_guard<std::recursive_mutex> lock(mtx);

            imu = *imuMsg;
            last_mocap_imu_rx_time = ros::Time::now();
            new_mocap_imu_frame = true;
            imusubTri = true;
        }

        void imuvel_cal(geometry_msgs::Vector3 &vel)
        {
            int data_c = imu_t.size();
            int order = 2;

            if (imu_t.size() > (fit_size - 1))
            {
                imu_t.erase(imu_t.begin());
                imuvel_x.erase(imuvel_x.begin());
                imuvel_y.erase(imuvel_y.begin());
                imuvel_z.erase(imuvel_z.begin());
            }

            if (imu_t.size() < 1)
            {
                imu_t0 = imu.header.stamp.toSec();
            }

            imu_t.emplace_back(imu.header.stamp.toSec() - imu_t0);

            if (data_c > order)
            {
                double imu_dur;

                imu_dur = imu_t[data_c - 1] - imu_t[data_c - 2];
                vel.x = imuvel_x[data_c - 1] + imu.linear_acceleration.x * imu_dur;
                vel.y = imuvel_y[data_c - 1] + imu.linear_acceleration.y * imu_dur;
                vel.z = imuvel_z[data_c - 1] + imu.linear_acceleration.z * imu_dur;

                imuvel_x.emplace_back(vel.x);
                imuvel_y.emplace_back(vel.y);
                imuvel_z.emplace_back(vel.z);

                fit.polyfit(imu_t, imuvel_x, order, false);
                vel.x = imuvel_x[data_c - 1] - fit.getY(imu_t[data_c - 1]);

                fit.polyfit(imu_t, imuvel_y, order, false);
                vel.y = imuvel_y[data_c - 1] - fit.getY(imu_t[data_c - 1]);

                fit.polyfit(imu_t, imuvel_z, order, false);
                vel.z = imuvel_z[data_c - 1] - fit.getY(imu_t[data_c - 1]);
            }
            else
            {
                imuvel_x.resize(order + 1, 0);
                imuvel_y.resize(order + 1, 0);
                imuvel_z.resize(order + 1, 0);
            }
        }

        void motion_cal(Eigen::Quaterniond &qua)
        {
            Eigen::Quaterniond gtruth_orientation;
            gtruth_orientation.w() = gtruth.pose.pose.orientation.w;
            gtruth_orientation.x() = gtruth.pose.pose.orientation.x;
            gtruth_orientation.y() = gtruth.pose.pose.orientation.y;
            gtruth_orientation.z() = gtruth.pose.pose.orientation.z;

            qua = gtruth_qua_bias.inverse() * gtruth_orientation;
        }

        void motion_cal(Eigen::Quaterniond &qua, Eigen::Vector3d &pos, Eigen::Vector3d &lin_vel, Eigen::Vector3d &)
        {
            Eigen::Quaterniond gtruth_orientation;
            Eigen::Vector3d p(gtruth.pose.pose.position.x, gtruth.pose.pose.position.y, gtruth.pose.pose.position.z);
            Eigen::Vector3d p_bias(gtruth_pos_bias.x, gtruth_pos_bias.y, gtruth_pos_bias.z);
            Eigen::Vector3d l_v(gtruth.twist.twist.linear.x, gtruth.twist.twist.linear.y, gtruth.twist.twist.linear.z);
            Eigen::Vector3d a_v(gtruth.twist.twist.angular.x, gtruth.twist.twist.angular.y, gtruth.twist.twist.angular.z);

            gtruth_orientation.w() = gtruth.pose.pose.orientation.w;
            gtruth_orientation.x() = gtruth.pose.pose.orientation.x;
            gtruth_orientation.y() = gtruth.pose.pose.orientation.y;
            gtruth_orientation.z() = gtruth.pose.pose.orientation.z;

            qua = gtruth_qua_bias.inverse() * gtruth_orientation;
            pos = gtruth_qua_bias.inverse() * (p - p_bias);
            lin_vel = gtruth_qua_bias.inverse() * l_v;
        }

        void mc_odom_pub(const ros::TimerEvent &)
        {
            // timer 与 mctruthCallback/imuCallback 并发(MT 队列不同 worker),
            // 全程持锁: gtruth/imu/bias 累计量的读写均在锁内(与 gps_odom_pub 同构)。
            std::lock_guard<std::recursive_mutex> lock(mtx);

            if (imusubTri && gtruthsubTri)
            {
                const ros::Time now = ros::Time::now();
                const double pose_age = (now - last_mocap_rx_time).toSec();
                const double imu_age = (now - last_mocap_imu_rx_time).toSec();
                if (!std::isfinite(pose_age) || pose_age < 0.0 ||
                    pose_age > mocap_input_timeout ||
                    !std::isfinite(imu_age) || imu_age < 0.0 ||
                    imu_age > mocap_input_timeout)
                {
                    ROS_ERROR_THROTTLE(
                        1.0,
                        "[odom_remap]: Mocap/IMU input stale (pose=%.3fs, imu=%.3fs); cached odom not published.",
                        pose_age, imu_age);
                    return;
                }

                if (!new_mocap_frame || !new_mocap_imu_frame)
                {
                    return;
                }
                new_mocap_frame = false;
                new_mocap_imu_frame = false;

                seq += 1;
                double gtruth_t = gtruth.header.stamp.toSec();

                if (seq < 300)
                {
                    gtruth_const_bias_cal(gtruth_t);
                }
                else if (seq == 300)
                {
                    gtruth_time_delay = gtruth_time_delay / bias_c;
                    gtruth_pos_bias.x = gtruth_pos_bias.x / bias_c;
                    gtruth_pos_bias.y = gtruth_pos_bias.y / bias_c;
                    gtruth_pos_bias.z = gtruth_pos_bias.z / bias_c;
                    gtruth_rpy_bias.x = gtruth_rpy_bias.x / bias_c;
                    gtruth_rpy_bias.y = gtruth_rpy_bias.y / bias_c;
                    gtruth_rpy_bias.z = gtruth_rpy_bias.z / bias_c;

                    gtruth_qua_bias = Eigen::AngleAxisd(gtruth_rpy_bias.z, Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(gtruth_rpy_bias.y, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(gtruth_rpy_bias.x, Eigen::Vector3d::UnitX());

                    ROS_INFO("[odom_remap]:Odom const bias cal succeed, ready to flight!");

                    std::cout << "delta_r = " << gtruth_rpy_bias.x * 180 / 3.14159265 << " delta_p = " << gtruth_rpy_bias.y * 180 / 3.14159265 << " delta_y = " << gtruth_rpy_bias.z * 180 / 3.14159265 << std::endl;
                }
                else
                {
                    if ((abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec()) < 0.05))
                    {
                        geometry_msgs::Vector3 vel;
                        nav_msgs::OdometryPtr odomMsg(new nav_msgs::Odometry);
                        Eigen::Quaterniond gtruth_qua;

                        imuvel_cal(vel);
                        motion_cal(gtruth_qua);

                        odomMsg->header.stamp = ros::Time().fromSec(gtruth_t - gtruth_time_delay);
                        odomMsg->pose.pose.position.x = gtruth.pose.pose.position.y - gtruth_pos_bias.y;
                        odomMsg->pose.pose.position.y = -(gtruth.pose.pose.position.x - gtruth_pos_bias.x);
                        odomMsg->pose.pose.position.z = gtruth.pose.pose.position.z - gtruth_pos_bias.z;
                        odomMsg->pose.pose.orientation.w = gtruth_qua.w();
                        odomMsg->pose.pose.orientation.x = gtruth_qua.x();
                        odomMsg->pose.pose.orientation.y = gtruth_qua.y();
                        odomMsg->pose.pose.orientation.z = gtruth_qua.z();
                        odomMsg->twist.twist.linear.x = vel.x;
                        odomMsg->twist.twist.linear.y = vel.y;
                        odomMsg->twist.twist.linear.z = vel.z;
                        odomMsg->twist.twist.angular.x = 0;
                        odomMsg->twist.twist.angular.y = 0;
                        odomMsg->twist.twist.angular.z = 0;

                        odomPub.publish(odomMsg);
                    }
                    else
                    {
                        ROS_WARN("[odom_remap]:Truth data delay too high!");
                        std::cout << "truth_data_delay= " << abs(gtruth_t - gtruth_time_delay - imu.header.stamp.toSec()) << std::endl;
                    }
                }
            }
            else
            {
                // 数据未就绪: 只告警不阻塞。原 while-sleep 会永久占用 nodelet manager
                // 的 worker 线程(pose/imu 断流期间整个回调队列少一个线程)。
                if (!gtruthsubTri)
                {
                    ROS_WARN_THROTTLE(
                        2.0,
                        "[odom_remap]: Waiting for pose on '%s' ...",
                        gtruthTopic.c_str());
                }
                if (!imusubTri)
                {
                    ROS_WARN_THROTTLE(
                        2.0,
                        "[odom_remap]: Waiting for imu on '%s' ...",
                        imuTopic.c_str());
                }
            }
        }

        void gps_odom_pub(const ros::TimerEvent &)
        {
            std::lock_guard<std::recursive_mutex> lock(mtx);

            // ============================================================
            // 1. 尚未收到 UAV odom
            // ============================================================
            if (!gtruthsubTri)
            {
                ROS_WARN_THROTTLE(
                    2.0,
                    "[odom_remap]: Waiting for UAV odom on '%s' ...",
                    gtruthTopic.c_str());

                return;
            }

            const ros::Time now = ros::Time::now();

            // ============================================================
            // 2. 检查 ROS 端多久没有真正收到新 MAVROS odom
            //
            // 不能再比较：
            //
            // current_header_stamp - previous_header_stamp
            //
            // 因为旧帧重复时这个值正好为 0。
            // ============================================================
            if (last_gtruth_rx_time.isZero())
            {
                return;
            }

            const double odom_age =
                (now - last_gtruth_rx_time).toSec();

            if (odom_age > gps_odom_timeout)
            {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[odom_remap]: MAVROS local odom stale, "
                    "no fresh frame for %.3f s.",
                    odom_age);

                // 绝对不能重复发布旧定位
                return;
            }

            // ============================================================
            // 3. timer 可以继续保持 200 Hz，
            //    但没有新的输入 odom 就什么都不做
            // ============================================================
            if (!new_gtruth_frame)
            {
                return;
            }

            // 当前这帧只允许消费一次
            new_gtruth_frame = false;

            // ============================================================
            // 4. GPS/RTK 零点标定
            //
            // 现在：
            //
            // seq == 真正的新 /mavros/local_position/odom 样本数
            //
            // 而不是 timer 次数
            // ============================================================
            if (!gps_bias_ready)
            {
                // 函数内部 bias_c += 1，并累加 position/quaternion
                gtruth_const_bias_cal();

                seq += 1;

                if (seq % 50 == 0 || seq == 1)
                {
                    ROS_INFO(
                        "[odom_remap]: GPS bias calibration %d/%d "
                        "fresh odom samples.",
                        seq,
                        gps_bias_samples);
                }

                // 还没有真正收满 N 个新样本
                if (seq < gps_bias_samples)
                {
                    return;
                }

                // ========================================================
                // 此处应严格满足：
                //
                // seq == gps_bias_samples
                // bias_c == gps_bias_samples
                // ========================================================
                if (bias_c <= 0)
                {
                    ROS_ERROR(
                        "[odom_remap]: Invalid GPS bias sample count.");

                    return;
                }

                if (bias_c != seq)
                {
                    ROS_ERROR(
                        "[odom_remap]: GPS bias counter mismatch: "
                        "seq=%d, bias_c=%d.",
                        seq,
                        bias_c);

                    return;
                }

                // ----------------------------
                // position average
                // ----------------------------
                gtruth_pos_bias.x /= bias_c;
                gtruth_pos_bias.y /= bias_c;
                gtruth_pos_bias.z /= bias_c;

                // ----------------------------
                // quaternion arithmetic average
                // ----------------------------
                gtruth_qua_bias.w() /= bias_c;
                gtruth_qua_bias.x() /= bias_c;
                gtruth_qua_bias.y() /= bias_c;
                gtruth_qua_bias.z() /= bias_c;

                // 防止异常四元数
                const double qua_norm = gtruth_qua_bias.norm();

                if (qua_norm < 1e-6)
                {
                    ROS_ERROR(
                        "[odom_remap]: Invalid averaged quaternion, "
                        "norm=%.6e.",
                        qua_norm);

                    return;
                }

                gtruth_qua_bias.normalize();

                gps_bias_ready = true;

                ROS_INFO(
                    "[odom_remap]: GPS bias calibration succeeded "
                    "with %d fresh odom samples.",
                    bias_c);

                ROS_INFO(
                    "[odom_remap]: position bias = "
                    "[%.4f, %.4f, %.4f]",
                    gtruth_pos_bias.x,
                    gtruth_pos_bias.y,
                    gtruth_pos_bias.z);

                Eigen::Matrix3d R_bias =
                    gtruth_qua_bias.toRotationMatrix();

                Eigen::Vector3d euler_bias =
                    R_bias.eulerAngles(2, 1, 0);

                ROS_INFO(
                    "[odom_remap]: attitude bias [deg] = "
                    "[%.3f, %.3f, %.3f]",
                    euler_bias[2] * 180.0 / M_PI,
                    euler_bias[1] * 180.0 / M_PI,
                    euler_bias[0] * 180.0 / M_PI);

                // 标定完成这一帧不作为控制输出。
                // 从下一帧开始正式发布 /odom/remap。
                return;
            }

            // ============================================================
            // 5. 正常 GPS/RTK odom 输出
            //
            // 到这里必然满足：
            //
            // - 当前 MAVROS odom 没有超时
            // - 当前输入是一帧真正的新 odom
            // - 零点标定已经完成
            //
            // 因此：
            //
            // 一帧输入 → 最多发布一帧 /odom/remap
            // ============================================================
            nav_msgs::Odometry odomMsg;

            Eigen::Quaterniond gtruth_qua;
            Eigen::Vector3d position;
            Eigen::Vector3d lin_vel;
            Eigen::Vector3d ang_vel;

            motion_cal(
                gtruth_qua,
                position,
                lin_vel,
                ang_vel);

            odomMsg.header = gtruth.header;
            odomMsg.child_frame_id =
                gtruth.child_frame_id;

            odomMsg.pose.pose.position.x =
                position.x();

            odomMsg.pose.pose.position.y =
                position.y();

            odomMsg.pose.pose.position.z =
                position.z();

            odomMsg.pose.covariance =
                gtruth.pose.covariance;

            odomMsg.pose.pose.orientation.w =
                gtruth_qua.w();

            odomMsg.pose.pose.orientation.x =
                gtruth_qua.x();

            odomMsg.pose.pose.orientation.y =
                gtruth_qua.y();

            odomMsg.pose.pose.orientation.z =
                gtruth_qua.z();

            odomMsg.twist.twist.linear.x =
                lin_vel.x();

            odomMsg.twist.twist.linear.y =
                lin_vel.y();

            odomMsg.twist.twist.linear.z =
                lin_vel.z();

            // 暂时保留现有 angular velocity 处理
            odomMsg.twist.twist.angular =
                gtruth.twist.twist.angular;

            odomPub.publish(odomMsg);
        }

        void init(ros::NodeHandle &nh)
        {

            int localization_source;

            nh.getParam(
                "gtruthTopic",
                gtruthTopic);

            nh.param<std::string>(
                "modelName",
                modelName,
                "iris_0");

            nh.param(
                "fit_size",
                fit_size,
                100);

            // ============================================================
            // 定位来源选择
            //
            // 0 : Mocap / 外部位姿
            // 1 : GPS/RTK 实飞或 Gazebo 仿真
            // ============================================================
            nh.param(
                "localization_source",
                localization_source,
                0);

            nh.param(
                "simulation",
                issimulation,
                false);

            nh.param(
                "gps_bias_samples",
                gps_bias_samples,
                300);

            nh.param(
                "gps_odom_timeout",
                gps_odom_timeout,
                0.20);

            nh.param(
                "mocap_input_timeout",
                mocap_input_timeout,
                0.20);

            nh.param(
                "gps_vel_sync_queue_size",
                gps_vel_sync_queue_size,
                20);

            nh.param(
                "gps_vel_max_skew",
                gps_vel_max_skew,
                0.02);

            if (!std::isfinite(mocap_input_timeout) || mocap_input_timeout <= 0.0 ||
                !std::isfinite(gps_odom_timeout) || gps_odom_timeout <= 0.0)
            {
                ROS_FATAL("[odom_remap]: localization input timeout must be finite and > 0.");
                return;
            }

            odomPub = nh.advertise<nav_msgs::Odometry>(
                "/odom/remap",
                1);

            // ============================================================
            // 根据 localization_source 选择定位链路
            //
            // 0 : Mocap / 外部位姿
            // 1 : GPS/RTK 实飞或 Gazebo 仿真
            // ============================================================
            switch (localization_source)
            {
            case 0:
            {
                gtruthSub = nh.subscribe(
                    gtruthTopic,
                    1,
                    &odomRemap::mctruthCallback,
                    this,
                    ros::TransportHints().tcpNoDelay());

                // mocap 模式必须同时有 IMU(时间对齐 gtruth_time_delay + 速度积分);
                // 原实现只声明 imuSub 从未订阅, imusubTri 恒 false → 分支永远走不到发布。
                nh.param<std::string>(
                    "imuTopic",
                    imuTopic,
                    "/mavros/imu/data");

                imuSub = nh.subscribe(
                    imuTopic,
                    1,
                    &odomRemap::imuCallback,
                    this,
                    ros::TransportHints().tcpNoDelay());

                odom_timer = nh.createTimer(
                    ros::Duration(0.005),
                    &odomRemap::mc_odom_pub,
                    this);

                ROS_INFO(
                    "[odom_remap]: Using Mocap localization "
                    "(pose='%s', imu='%s').",
                    gtruthTopic.c_str(),
                    imuTopic.c_str());

                break;
            }

            case 1:
            {
                if (!issimulation)
                {
                    gpsOdomSyncSub.subscribe(
                        nh,
                        gtruthTopic,
                        gps_vel_sync_queue_size,
                        ros::TransportHints().tcpNoDelay());

                    gpsVelSyncSub.subscribe(
                        nh,
                        "/mavros/local_position/velocity_local",
                        gps_vel_sync_queue_size,
                        ros::TransportHints().tcpNoDelay());

                    gpsVelSynchronizer.reset(
                        new message_filters::Synchronizer<GpsVelSyncPolicy>(
                            GpsVelSyncPolicy(gps_vel_sync_queue_size),
                            gpsOdomSyncSub,
                            gpsVelSyncSub));

                    gpsVelSynchronizer->setMaxIntervalDuration(
                        ros::Duration(gps_vel_max_skew));

                    gpsVelSynchronizer->registerCallback(
                        boost::bind(
                            &odomRemap::gpsOdomVelCallback,
                            this,
                            boost::placeholders::_1,
                            boost::placeholders::_2));

                    ROS_INFO(
                        "[odom_remap]: Using time-synchronized "
                        "GPS/RTK odom + velocity_local "
                        "(queue=%d, max_skew=%.3fs).",
                        gps_vel_sync_queue_size,
                        gps_vel_max_skew);
                }
                else
                {
                    gtruthSub = nh.subscribe(
                        gtruthTopic,
                        1,
                        &odomRemap::simtruthCallback,
                        this,
                        ros::TransportHints().tcpNoDelay());

                    ROS_INFO(
                        "[odom_remap]: Using simulation model_states "
                        "('%s').",
                        modelName.c_str());
                }

                odom_timer = nh.createTimer(
                    ros::Duration(0.005),
                    &odomRemap::gps_odom_pub,
                    this);

                break;
            }

            default:
            {
                ROS_FATAL(
                    "[odom_remap]: Invalid localization_source=%d. "
                    "Valid values: 0=Mocap, 1=GPS/RTK.",
                    localization_source);

                // nodelet 与控制器共享 manager；全局 shutdown 会把同进程内的控制链一并终止。
                // 保持本 nodelet 未初始化并直接返回，让 manager 和其他 nodelet 继续运行。
                return;
            }
            }
        }

    public:
        void onInit() override
        {
            ros::NodeHandle nh(getMTPrivateNodeHandle());
            init(nh);
        }
    };
} // namespace odomRemap
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(odomRemap::odomRemap, nodelet::Nodelet);
