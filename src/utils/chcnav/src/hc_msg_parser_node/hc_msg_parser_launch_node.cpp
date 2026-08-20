#include "ros/ros.h"

// msgs insclude
#include "chcnav/hc_sentence.h"
#include "chcnav/int8_array.h"
#include "chcnav/string.h"
#include "device_connector.hpp"
#include "hc_msg_parser.hpp"
#include "serial_common.hpp"
#include "std_msgs/String.h"
#include "tcp_common.hpp"
#include "udp_common.hpp"

#include <memory>
#include <string.h>
#include <string>

#include "file_common.hpp"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hc_msg_parser_node");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    std::unique_ptr<hc__device_connector> device_connector;

    /*********************************************************
     * params parse
     *********************************************************/
    string type = ""; // 参数配置类型
    if (private_nh.getParam("type", type) == false)
    {
        ROS_ERROR("chcnav: type is not set.");
        return -1;
    }

    int rate = 0; // 获取处理频率
    private_nh.param<int>("rate", rate, 1000);
    if (rate < 0)
    {
        ROS_ERROR("rate must be >= 0");
        return -1;
    }
    if (0 == rate)
        ROS_INFO("rate [0], No message pub deal subscribe only");

    if (type.compare("serial") == 0)
    {
        // 如果参数配置为串口
        string port = "", parity = "";
        int baudrate = -1, databits = -1, stopbits = -1;

        if (private_nh.getParam("port", port) == false)
        {
            ROS_ERROR("chcnav: port is not set.");
            return -1;
        }
        private_nh.param<int>("baudrate", baudrate, 115200);
        private_nh.param<int>("databits", databits, 8);
        private_nh.param<int>("stopbits", stopbits, 1);
        private_nh.param<std::string>("parity", parity, "None");

        ROS_INFO("serial config port[%s] baudrate[%d] databits[%d] stopbits[%d] parity[%s] rate [%d]",
                 port.c_str(), baudrate, databits, stopbits, parity.c_str(), rate);

        device_connector.reset(
            new serial_common(port, baudrate, databits, stopbits, parity));
    }
    else if (type.compare("tcp") == 0)
    {
        string host = "";
        int port = 0;

        if (private_nh.getParam("host", host) == false)
        {
            ROS_ERROR("chcnav: host is not set.");
            return -1;
        }

        if (private_nh.getParam("port", port) == false)
        {
            ROS_ERROR("chcnav: port is not set.");
            return -1;
        }

        ROS_INFO("tcp config host[%s] port[%d]", host.c_str(), port);

        device_connector.reset(new tcp_common(host, port));
    }
    else if (type.compare("file") == 0)
    {
        string path = "";
        if (private_nh.getParam("path", path) == false)
        {
            ROS_ERROR("chcnav: path is not set.");
            return -1;
        }

        ROS_INFO("file path[%s]", path.c_str());

        usleep(500 * 1000); // 很重要，等待其他节点都启动完毕再解析文件，不然会造成先解析文件，导致一些在别的节点启动前就被解析出的数据未处理。
        device_connector.reset(new file_common(path));
    }
    else if (type.compare("udp") == 0)
    {
        int port = 0;

        if (private_nh.getParam("port", port) == false)
        {
            ROS_ERROR("chcnav: port is not set.");
            return -1;
        }

        ROS_INFO("udp config port[%d]", port);

        device_connector.reset(new udp_common(port));
    }
    else
    {
        ROS_ERROR("chcnav: unsupported connector type [%s].", type.c_str());
        return -1;
    }

    // 获取节点名
    string node_path = ros::this_node::getName();
    string node_name = node_path.substr(node_path.find_last_of("/") + 1);

    // 初始化信息解析节点
    hc__msg_parser_node parser_node(
        static_cast<unsigned int>(rate), device_connector.get(),
        &nh, &private_nh, node_name);

    if (parser_node.device_connector->connect() != 0)
    {
        ROS_ERROR("chcnav: initial device connection failed.");
        return -1;
    }

    parser_node.node_start();

    return 0;
}
