#include "ros/ros.h"

#include "chcnav/hc_sentence.h"
#include "chcnav/hcinspvatzcb.h"
#include "chcnav/hcrawimub.h"
#include "hc_cgi_protocol.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

static ros::Publisher gs_devpvt_pub;
static ros::Publisher gs_devimu_pub;

static unsigned int g_leaps = 18;

template <typename T, typename Byte>
static T read_little_endian(const std::vector<Byte> &data,
                            std::size_t offset)
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "wire value must be trivially copyable");
    T value{};
    if (offset > data.size() || sizeof(T) > data.size() - offset)
    {
        return value;
    }

    const std::uint16_t endian_probe = 1U;
    const bool host_is_little_endian =
        *reinterpret_cast<const std::uint8_t *>(&endian_probe) == 1U;
    if (host_is_little_endian)
    {
        std::memcpy(&value, data.data() + offset, sizeof(T));
    }
    else
    {
        std::array<std::uint8_t, sizeof(T)> bytes{};
        std::reverse_copy(data.begin() + offset,
                          data.begin() + offset + sizeof(T),
                          bytes.begin());
        std::memcpy(&value, bytes.data(), sizeof(T));
    }
    return value;
}

/**
 * @brief 处理华测协议的回调函数
 *
 * @param msg 接收到的数据
 * */
static void hc_sentence_callback(const chcnav::hc_sentence::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hc_cgi_protocol_process_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber serial_suber = nh.subscribe("hc_sentence", 1000, hc_sentence_callback);

    gs_devpvt_pub = nh.advertise<chcnav::hcinspvatzcb>("devpvt", 1000);
    gs_devimu_pub = nh.advertise<chcnav::hcrawimub>("devimu", 1000);

    ros::spin();

    return 0;
}

// 处理各个协议的函数（仅保留本节点实际定义并使用的两类协议处理函数）
static void msg_deal__hcrawimuib(const chcnav::hc_sentence::ConstPtr &msg);
static void msg_deal__hcinspvatzcb(const chcnav::hc_sentence::ConstPtr &msg);

/**
 * @brief 处理华测协议的回调函数
 *
 * @param msg 接收到的数据
 * */
static void hc_sentence_callback(const chcnav::hc_sentence::ConstPtr &msg)
{
    if (msg->data.empty())
    {
        ROS_WARN_THROTTLE(1.0, "empty CHCNAV frame ignored");
        return;
    }

    if (hc__cgi_check_crc32(
            reinterpret_cast<const unsigned char *>(msg->data.data()),
            static_cast<unsigned int>(msg->data.size())) != 0)
    {
        fprintf(stderr, "crc32 check failed!\n");
        return;
    }

    switch (msg->msg_id)
    {
        case INSPVATZCB:
            msg_deal__hcinspvatzcb(msg);
            break;
        case RAWIMUIB:
            msg_deal__hcrawimuib(msg);
            break;
        default:
            break;
    }

    return;
}

static void msg_deal__hcinspvatzcb(const chcnav::hc_sentence::ConstPtr &msg)
{
    chcnav::hcinspvatzcb devpvt;

    // devpvt的header使用原msg的header
    devpvt.header = msg->header;

    // 如果长度不对，不解析发布
    if (msg->data.size() == 296)
    {
        // 润秒
        devpvt.leaps = read_little_endian<std::uint16_t>(msg->data, 162U);
        g_leaps = devpvt.leaps;

        // gps 周 周内秒
        devpvt.week = read_little_endian<std::uint16_t>(msg->data, 22U);
        devpvt.second = read_little_endian<double>(msg->data, 24U);
        devpvt.header.stamp = ros::Time(devpvt.week * 7.0 * 24.0 * 3600.0 + devpvt.second + 315964800.0 - g_leaps);
        
        // 经纬高
        devpvt.latitude = read_little_endian<double>(msg->data, 32U);
        devpvt.longitude = read_little_endian<double>(msg->data, 40U);
        devpvt.altitude = read_little_endian<float>(msg->data, 48U);

        devpvt.position_stdev[0] = read_little_endian<float>(msg->data, 80U);
        devpvt.position_stdev[1] = read_little_endian<float>(msg->data, 84U);
        devpvt.position_stdev[2] = read_little_endian<float>(msg->data, 88U);

        devpvt.undulation = read_little_endian<float>(msg->data, 52U);

        // 姿态角
        devpvt.roll = read_little_endian<float>(msg->data, 72U);
        devpvt.pitch = read_little_endian<float>(msg->data, 68U);
        devpvt.yaw = read_little_endian<float>(msg->data, 76U);

        devpvt.euler_stdev[0] = read_little_endian<float>(msg->data, 108U); // std_roll
        devpvt.euler_stdev[1] = read_little_endian<float>(msg->data, 104U); // std_pitch
        devpvt.euler_stdev[2] = read_little_endian<float>(msg->data, 112U); // std_yaw

        devpvt.speed = read_little_endian<float>(msg->data, 140U);
        devpvt.heading = read_little_endian<float>(msg->data, 144U);
        devpvt.heading2 = read_little_endian<float>(msg->data, 148U);

        // vehicle velocity and acceleration
        devpvt.vehicle_angular_velocity.x = read_little_endian<float>(msg->data, 116U);
        devpvt.vehicle_angular_velocity.y = read_little_endian<float>(msg->data, 120U);
        devpvt.vehicle_angular_velocity.z = read_little_endian<float>(msg->data, 124U);

        devpvt.vehicle_linear_velocity.x = read_little_endian<float>(msg->data, 208U);
        devpvt.vehicle_linear_velocity.y = read_little_endian<float>(msg->data, 212U);
        devpvt.vehicle_linear_velocity.z = read_little_endian<float>(msg->data, 216U);

        devpvt.vehicle_linear_acceleration.x = read_little_endian<float>(msg->data, 196U);
        devpvt.vehicle_linear_acceleration.y = read_little_endian<float>(msg->data, 200U);
        devpvt.vehicle_linear_acceleration.z = read_little_endian<float>(msg->data, 204U);

        devpvt.vehicle_linear_acceleration_without_g.x = read_little_endian<float>(msg->data, 128U);
        devpvt.vehicle_linear_acceleration_without_g.y = read_little_endian<float>(msg->data, 132U);
        devpvt.vehicle_linear_acceleration_without_g.z = read_little_endian<float>(msg->data, 136U);

        devpvt.enu_velocity.x = read_little_endian<float>(msg->data, 56U);
        devpvt.enu_velocity.y = read_little_endian<float>(msg->data, 60U);
        devpvt.enu_velocity.z = read_little_endian<float>(msg->data, 64U);

        devpvt.enu_velocity_stdev[0] = read_little_endian<float>(msg->data, 92U);
        devpvt.enu_velocity_stdev[1] = read_little_endian<float>(msg->data, 96U);
        devpvt.enu_velocity_stdev[2] = read_little_endian<float>(msg->data, 100U);

        // 原始imu数据
        devpvt.raw_angular_velocity.x = read_little_endian<float>(msg->data, 172U);
        devpvt.raw_angular_velocity.y = read_little_endian<float>(msg->data, 176U);
        devpvt.raw_angular_velocity.z = read_little_endian<float>(msg->data, 180U);

        devpvt.raw_acceleration.x = read_little_endian<float>(msg->data, 184U);
        devpvt.raw_acceleration.y = read_little_endian<float>(msg->data, 188U);
        devpvt.raw_acceleration.z = read_little_endian<float>(msg->data, 192U);

        // stat, warning and flags
        devpvt.stat[0] = msg->data[152] & 0x0f;
        devpvt.stat[1] = (msg->data[152] >> 4) & 0x0f;

        devpvt.age = read_little_endian<float>(msg->data, 154U);

        devpvt.ns = read_little_endian<std::uint16_t>(msg->data, 158U);
        devpvt.ns2 = read_little_endian<std::uint16_t>(msg->data, 160U);

        // dop
        devpvt.hdop = read_little_endian<float>(msg->data, 164U);
        devpvt.pdop = read_little_endian<float>(msg->data, 220U);
        devpvt.vdop = read_little_endian<float>(msg->data, 224U);
        devpvt.tdop = read_little_endian<float>(msg->data, 228U);
        devpvt.gdop = read_little_endian<float>(msg->data, 232U);

        // warning
        devpvt.warning = read_little_endian<std::uint16_t>(msg->data, 168U);
        devpvt.sensor_used = read_little_endian<std::uint16_t>(msg->data, 170U);

        // body
        devpvt.ins2gnss_vector.x = read_little_endian<float>(msg->data, 236U);
        devpvt.ins2gnss_vector.y = read_little_endian<float>(msg->data, 240U);
        devpvt.ins2gnss_vector.z = read_little_endian<float>(msg->data, 244U);

        devpvt.ins2body_angle.x = read_little_endian<float>(msg->data, 248U);
        devpvt.ins2body_angle.y = read_little_endian<float>(msg->data, 252U);
        devpvt.ins2body_angle.z = read_little_endian<float>(msg->data, 256U);

        devpvt.gnss2body_vector.x = read_little_endian<float>(msg->data, 260U);
        devpvt.gnss2body_vector.y = read_little_endian<float>(msg->data, 264U);
        devpvt.gnss2body_vector.z = read_little_endian<float>(msg->data, 268U);

        devpvt.gnss2body_angle_z = read_little_endian<float>(msg->data, 272U);

        for (int index = 0; index < 16; index++)
            devpvt.receiver[index] = msg->data[276U + static_cast<std::size_t>(index)];

        gs_devpvt_pub.publish(devpvt);
    }
}

static void msg_deal__hcrawimuib(const chcnav::hc_sentence::ConstPtr &msg)
{
    chcnav::hcrawimub devimu;

    // devpvt的header使用原msg的header
    devimu.header = msg->header;

    // 如果长度不对，不解析发布
    if (msg->data.size() == 68)
    {
        // header的时间设置为gps时间
        devimu.week = read_little_endian<std::uint16_t>(msg->data, 22U);
        devimu.second = read_little_endian<double>(msg->data, 24U);
        devimu.header.stamp = ros::Time(devimu.week * 7.0 * 24.0 * 3600.0 + devimu.second + 315964800.0 - g_leaps);

        // xyz角速度
        devimu.angular_velocity.x = read_little_endian<float>(msg->data, 32U);
        devimu.angular_velocity.y = read_little_endian<float>(msg->data, 36U);
        devimu.angular_velocity.z = read_little_endian<float>(msg->data, 40U);

        // xyz角角速度 g
        devimu.angular_acceleration.x = read_little_endian<float>(msg->data, 44U);
        devimu.angular_acceleration.y = read_little_endian<float>(msg->data, 48U);
        devimu.angular_acceleration.z = read_little_endian<float>(msg->data, 52U);

        // 温度
        devimu.temp = read_little_endian<float>(msg->data, 56U);

        // 异常表示
        devimu.err_status = msg->data[60U];

        // Z轴陀螺积分航向, 180~180 系数 0.01
        devimu.yaw = read_little_endian<std::int16_t>(msg->data, 61U);

        // 预留
        devimu.receiver = msg->data[63U];

        gs_devimu_pub.publish(devimu);
    }
    //short 
    if (msg->data.size() == 34)
    {
        // header的时间设置为gps时间
        devimu.week = read_little_endian<std::uint16_t>(msg->data, 8U);
        devimu.second = static_cast<double>(
            read_little_endian<std::uint32_t>(msg->data, 10U)) / 1000.0;
        devimu.header.stamp = ros::Time(devimu.week * 7.0 * 24.0 * 3600.0 + devimu.second + 315964800.0 - g_leaps);

        // xyz角速度
        devimu.angular_velocity.x = static_cast<float>(read_little_endian<std::int16_t>(msg->data, 14U)) / 80.0F;
        devimu.angular_velocity.y = static_cast<float>(read_little_endian<std::int16_t>(msg->data, 16U)) / 80.0F;
        devimu.angular_velocity.z = static_cast<float>(read_little_endian<std::int16_t>(msg->data, 18U)) / 80.0F;

        // xyz角角速度 g
        devimu.angular_acceleration.x = static_cast<float>(read_little_endian<std::int16_t>(msg->data, 20U)) / 5000.0F;
        devimu.angular_acceleration.y = static_cast<float>(read_little_endian<std::int16_t>(msg->data, 22U)) / 5000.0F;
        devimu.angular_acceleration.z = static_cast<float>(read_little_endian<std::int16_t>(msg->data, 24U)) / 5000.0F;

        // 温度
        devimu.temp = static_cast<float>(read_little_endian<std::int16_t>(msg->data, 26U)) / 100.0F;

        // 异常表示
        devimu.err_status = msg->data[28U];

        // 预留
        devimu.receiver = msg->data[29U];

        gs_devimu_pub.publish(devimu);
    }
}
