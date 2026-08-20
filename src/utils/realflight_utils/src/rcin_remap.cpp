#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "mavros_msgs/RCIn.h"
#include "std_msgs/String.h"

#include <quadrotor_msgs/RcinRemap.h>
#include <quadrotor_msgs/MotorlockTrigger.h>
namespace rcinRemap
{

    class rcRemap : public nodelet::Nodelet
    {
    private:
        ros::Publisher rcInPub;
        ros::Subscriber rcInSub;
        ros::Subscriber landtriSub;

        ros::Timer sim_timer;

        std::vector<uint16_t, std::allocator<uint16_t>> chn; // 通道中间变量
        std::vector<uint16_t, std::allocator<uint16_t>> chn_last;

        bool ifchange = false;
        bool locktrigger = false;
        bool takeoff_sim = false;

        void rcInCallback(const mavros_msgs::RCIn::ConstPtr &msg)
        {
            quadrotor_msgs::RcinRemapPtr rc_ref(new quadrotor_msgs::RcinRemap);

            rc_ref->header = msg->header;
            rc_ref->rssi = msg->rssi;
            rc_ref->channels = msg->channels;

            // 下方重映射/locktrigger 逻辑固定访问第 14/15/16 通道(下标 13~15)。
            // 实际通道数 <16 时(常见 8/12 通道接收机), chn/chn_last/rc_ref->channels
            // 均为源消息尺寸, 直接下标访问是数组越界(UB)。该场景原样透传并告警,
            // 禁止 13~15 重映射与状态切换; locktrigger 保持挂起, 待 16 通道帧到来再生效。
            const bool remap_ok = msg->channels.size() >= 16;
            if (!remap_ok)
            {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[rcin_remap]: RCIn only has %zu channels (<16); "
                    "pass through without ch14-16 remap.",
                    msg->channels.size());
            }

            if (remap_ok)
            {
                chn.resize(msg->channels.size());

                // 判断遥控器14/15/16通道上升沿；使用原始 PWM 阈值，避免整数除法截断。
                constexpr int kSwitchDeltaUs = 500;
                constexpr std::uint16_t kSwitchHighUs = 1500U;
                for (int i = 13; i < 16; i++)
                {
                    const int delta = std::abs(static_cast<int>(chn_last[i]) -
                                               static_cast<int>(rc_ref->channels[i]));
                    if (delta > kSwitchDeltaUs && rc_ref->channels[i] > kSwitchHighUs)
                    {
                        if (chn[i] > kSwitchHighUs) // 若此时 chn 对应通道为高位
                            chn[i] = 1000U;

                        else
                            chn[i] = 2000U;

                        ifchange = true;
                    }
                    rc_ref->channels[i] = chn[i];
                }

                if (locktrigger) // 若接收到motorlock信号，则切回手控模式
                {
                    chn[14] = 1000;
                    rc_ref->channels[14] = chn[14];
                    locktrigger = false;
                    ifchange = true;
                }

                if (ifchange)
                {
                    std::cout << "FN1(" << rc_ref->channels[14] << ") ," << "FN2(" << rc_ref->channels[13] << ") ," << "FN3(" << rc_ref->channels[15] << ")" << std::endl;
                    ifchange = false;
                }

                // chn_last 仅在重映射分支内被读取, 也只在接受 >=16 通道帧时更新,
                // 避免短帧把 chn_last 缩到 <16 后, 下一帧 16 通道时越界读取。
                chn_last = msg->channels;
            }

            rcInPub.publish(rc_ref);
        }

        void rcinSim(const ros::TimerEvent &)
        {
            quadrotor_msgs::RcinRemapPtr rc_ref(new quadrotor_msgs::RcinRemap);
            rc_ref->channels.resize(chn.size());

            if (locktrigger) // 若接收到motorlock信号，则切回悬停
            {
                chn[13] = 1000;
                locktrigger = false;
                ifchange = true;
            }

            // if(takeoff_sim)
            // {
            //     if(chn[14] == 1000)
            //     {
            //         chn[14] = 2000;
            //     }
            //     else
            //     {
            //         chn[13] = 2000;
            //         takeoff_sim = false;
            //     }
            // }

            rc_ref->header.stamp = ros::Time::now();
            rc_ref->rssi = 100;
            rc_ref->channels = chn;

            if (ifchange)
            {
                std::cout << "FN1(" << rc_ref->channels[14] << ") ," << "FN2(" << rc_ref->channels[13] << ") ," << "FN3(" << rc_ref->channels[15] << ")" << std::endl;
                ifchange = false;
            }

            rcInPub.publish(rc_ref);
        }

        void lockCallback(const quadrotor_msgs::MotorlockTrigger::ConstPtr &msg)
        {
            if (msg->trigger)
            {
                locktrigger = true;
            }
            else if (!msg->trigger)
            {
                takeoff_sim = true;
            }
        }

        void init(ros::NodeHandle &nh)
        {
            int flag;
            nh.param("flag", flag, 0); // 0 for real, 1 for sim
            chn.resize(16);
            for (int i = 0; i < 16; i++)
            {
                chn[i] = 1000;
            }

            rcInPub = nh.advertise<mavros_msgs::RCIn>("/mavros/rc/in/remap", 10);
            landtriSub = nh.subscribe<quadrotor_msgs::MotorlockTrigger>("/locktrigger", 1, &rcRemap::lockCallback, this,
                                                                       ros::TransportHints().tcpNoDelay());

            if (flag == 0)
            {
                chn_last = chn;
                rcInSub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, &rcRemap::rcInCallback, this,
                                                          ros::TransportHints().tcpNoDelay());

                ROS_INFO("[rcin_remap]:Waiting for rcIn");
            }
            else if (flag == 1)
            {
                ifchange = true;
                chn[13] = 2000;
                chn[14] = 2000;
                chn[0] = 1500;
                chn[1] = 1500;
                chn[2] = 1500;
                chn[3] = 1500;
                sim_timer = nh.createTimer(ros::Duration(0.05), &rcRemap::rcinSim, this);

                ROS_INFO("[rcin_remap]:rcin ready to publish");
            }
        }

    public:
        void onInit(void) override
        {
            ros::NodeHandle nh(getMTPrivateNodeHandle());
            // init() 只创建 publisher/subscriber/timer，不包含阻塞循环。
            init(nh);
        }
    };

} // namespace rcinRemap

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rcinRemap::rcRemap, nodelet::Nodelet);
