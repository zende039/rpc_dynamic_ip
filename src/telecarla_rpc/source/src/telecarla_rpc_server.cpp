#include "telecarla_rpc_server.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <fstream>  // For logging

#include "bool.h"
#include "rpc_msg_callback.h"
#include "vehicle_status.h"

using namespace lmt;

TeleCarlaRpcServer::TeleCarlaRpcServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : server_("10.131.222.209", pnh.param("rpc_port", 2002)),  // Bind to server IP and port
      egoVehicleStatus_(boost::make_shared<carla_msgs::CarlaEgoVehicleStatus>())
{
    ROS_INFO_STREAM("Opening RPC Server at IP 10.131.222.209 and port " << pnh.param("rpc_port", 2002));

    const auto prefix = "/carla/" + pnh.param("role_name", std::string("ego_vehicle"));

    // Binding RPC functions for commands
    server_.bind("set_enable_autopilot", [this, &nh, prefix](const bool& data, double send_time_sec) {
        ros::Time receive_time = ros::Time::now();
        double latency_ms = (receive_time.toSec() - send_time_sec) * 1000.0;
        ROS_INFO_STREAM("Received enable_autopilot with latency: " << latency_ms << " ms");
        logLatency("enable_autopilot", send_time_sec, receive_time.toSec(), latency_ms);

        std_msgs::Bool msg;
        msg.data = data;
        autopilotPublisher_.publish(msg);
    });
    server_.bind("set_vehicle_control_manual_override", [this, &nh, prefix](const bool& data, double send_time_sec) {
        ros::Time receive_time = ros::Time::now();
        double latency_ms = (receive_time.toSec() - send_time_sec) * 1000.0;
        ROS_INFO_STREAM("Received vehicle_control_manual_override with latency: " << latency_ms << " ms");
        logLatency("vehicle_control_manual_override", send_time_sec, receive_time.toSec(), latency_ms);

        std_msgs::Bool msg;
        msg.data = data;
        manualOverridePublisher_.publish(msg);
    });
    server_.bind("set_vehicle_control_cmd_manual", [this, &nh, prefix](float throttle, float steer, float brake, double send_time_sec) {
        ros::Time receive_time = ros::Time::now();
        double latency_ms = (receive_time.toSec() - send_time_sec) * 1000.0;
        ROS_INFO_STREAM("Received vehicle_control_cmd_manual with latency: " << latency_ms << " ms");
        logLatency("vehicle_control_cmd_manual", send_time_sec, receive_time.toSec(), latency_ms);

        carla_msgs::CarlaEgoVehicleControl msg;
        msg.throttle = throttle;
        msg.steer = steer;
        msg.brake = brake;
        vehicleControlPublisher_.publish(msg);
    });
    server_.bind("get_vehicle_status", [this]() { return data::VehicleStatus(egoVehicleStatus_); });

    // Advertise and subscribe to ROS topics
    autopilotPublisher_ = nh.advertise<std_msgs::Bool>(prefix + "/enable_autopilot", 1);
    manualOverridePublisher_ = nh.advertise<std_msgs::Bool>(prefix + "/vehicle_control_manual_override", 1);
    vehicleControlPublisher_ = nh.advertise<carla_msgs::CarlaEgoVehicleControl>(prefix + "/vehicle_control_cmd_manual", 1);
    vehicleStatusPublisher_ = nh.advertise<carla_msgs::CarlaEgoVehicleStatus>(prefix + "/vehicle_status", 1);

    vehicleStatusSubscriber_ = nh.subscribe<carla_msgs::CarlaEgoVehicleStatus>(
        prefix + "/vehicle_status", 1, &TeleCarlaRpcServer::vehicleStatusCallback, this);

    // Subscribers for client commands
    clientEnableAutopilotSubscriber_ = nh.subscribe<std_msgs::Bool>(
        prefix + "/client_enable_autopilot", 1, [this](const std_msgs::BoolConstPtr& msg) {
            ROS_INFO_STREAM("Server received enable_autopilot command from client.");
            autopilotPublisher_.publish(msg);
        });
    clientManualOverrideSubscriber_ = nh.subscribe<std_msgs::Bool>(
        prefix + "/client_vehicle_control_manual_override", 1, [this](const std_msgs::BoolConstPtr& msg) {
            ROS_INFO_STREAM("Server received vehicle_control_manual_override command from client.");
            manualOverridePublisher_.publish(msg);
        });
    clientVehicleControlCmdSubscriber_ = nh.subscribe<carla_msgs::CarlaEgoVehicleControl>(
        prefix + "/client_vehicle_control_cmd_manual", 1, [this](const carla_msgs::CarlaEgoVehicleControlConstPtr& msg) {
            ROS_INFO_STREAM("Server received vehicle_control_cmd_manual command from client.");
            vehicleControlPublisher_.publish(msg);
        });
}

void TeleCarlaRpcServer::run()
{
    server_.async_run();
}

void TeleCarlaRpcServer::stop()
{
    server_.close_sessions();
    server_.stop();
}

void TeleCarlaRpcServer::vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr& vehicleStatus)
{
    egoVehicleStatus_ = vehicleStatus;
}

void TeleCarlaRpcServer::logLatency(const std::string& command, double send_time, double receive_time, double latency)
{
    std::ofstream log_file("latency_log.csv", std::ios::app);
    log_file << command << "," << send_time << "," << receive_time << "," << latency << "\n";
    log_file.close();
}

