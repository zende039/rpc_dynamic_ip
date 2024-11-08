#include "telecarla_rpc_client.h"

#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <thread>  // for std::this_thread::sleep_for
#include <chrono>  // for std::chrono::seconds
#include <memory>  // for std::unique_ptr

#include "bool.h"
#include "ros_msg_callback.h"
#include "vehicle_status.h"

using namespace lmt;

std::string getServerIP(const std::string& filename) {
    std::ifstream file(filename);
    std::string ip;
    if (file.is_open() && std::getline(file, ip) && !ip.empty()) {
        return ip;
    }
    return "";  // Return an empty string if the file doesn't contain a valid IP
}

TeleCarlaRpcClient::TeleCarlaRpcClient(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : port_(pnh.param("rpc_port", 2002))
{
    std::string ip_filename = "/home/rushikesh98/telecarla_ws/ip_address.txt";  // Path to the IP file

    // Continuously check for IP address until it's available
    while (true) {
        host_ = getServerIP(ip_filename);
        if (!host_.empty()) {
            ROS_INFO_STREAM("Found IP address: " << host_);
            break;
        } else {
            ROS_WARN_STREAM("No IP address found in " << ip_filename << ". Retrying in 5 seconds...");
            std::this_thread::sleep_for(std::chrono::seconds(5));  // Wait for 5 seconds before retrying
        }
    }

    // Initialize client_ with host_ and port_ once the IP is available
    client_ = std::make_unique<rpc::client>(host_, port_);
    
    // Connection status check
    while (client_->get_connection_state() == rpc::client::connection_state::initial)
    {
        ROS_INFO_STREAM("Connecting to " << host_ << ":" << port_ << "...");
    }
    if (client_->get_connection_state() == rpc::client::connection_state::connected)
    {
        ROS_INFO_STREAM("Connected to " << host_ << ":" << port_);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to connect to " << host_ << ":" << port_);
    }

    const auto prefix = "/carla/" + pnh.param("role_name", std::string("ego_vehicle"));

    // Subscribe to topics and send RPC commands
    subscribers_.push_back(nh.subscribe<std_msgs::Bool>(
        prefix + "/enable_autopilot",
        1,
        [this](const std_msgs::BoolConstPtr& msg) {
            ros::Time send_time = ros::Time::now();
            client_->call("set_enable_autopilot", msg->data, send_time.toSec());
            ROS_INFO_STREAM("Sent enable_autopilot with timestamp: " << send_time.toSec());
            clientEnableAutopilotPublisher_.publish(msg);
        }));
    subscribers_.push_back(nh.subscribe<std_msgs::Bool>(
        prefix + "/vehicle_control_manual_override",
        1,
        [this](const std_msgs::BoolConstPtr& msg) {
            ros::Time send_time = ros::Time::now();
            client_->call("set_vehicle_control_manual_override", msg->data, send_time.toSec());
            ROS_INFO_STREAM("Sent vehicle_control_manual_override with timestamp: " << send_time.toSec());
            clientManualOverridePublisher_.publish(msg);
        }));
    subscribers_.push_back(nh.subscribe<carla_msgs::CarlaEgoVehicleControl>(
        prefix + "/vehicle_control_cmd_manual",
        1,
        [this](const carla_msgs::CarlaEgoVehicleControlConstPtr& msg) {
            ros::Time send_time = ros::Time::now();
            client_->call("set_vehicle_control_cmd_manual", msg->throttle, msg->steer, msg->brake, send_time.toSec());
            ROS_INFO_STREAM("Sent vehicle_control_cmd_manual with timestamp: " << send_time.toSec());
            clientVehicleControlCmdPublisher_.publish(msg);
        }));

    vehicleStatusPublisher_ = nh.advertise<carla_msgs::CarlaEgoVehicleStatus>(prefix + "/vehicle_status", 1);

    // Publishers for client commands to the server
    clientEnableAutopilotPublisher_ = nh.advertise<std_msgs::Bool>(prefix + "/client_enable_autopilot", 1);
    clientManualOverridePublisher_ = nh.advertise<std_msgs::Bool>(prefix + "/client_vehicle_control_manual_override", 1);
    clientVehicleControlCmdPublisher_ = nh.advertise<carla_msgs::CarlaEgoVehicleControl>(prefix + "/client_vehicle_control_cmd_manual", 1);
}

void TeleCarlaRpcClient::update()
{
    auto status = client_->call("get_vehicle_status").as<data::VehicleStatus>();
    vehicleStatusPublisher_.publish(status.toROSMessage());
}

