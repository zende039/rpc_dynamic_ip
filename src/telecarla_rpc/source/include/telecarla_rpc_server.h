#ifndef TELECARLA_RPC_TELECARLA_RPC_SERVER_H
#define TELECARLA_RPC_TELECARLA_RPC_SERVER_H

#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <std_msgs/Bool.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <rpc/server.h>

#include <fstream>
#include <string>

namespace lmt
{
class TeleCarlaRpcServer
{
  public:
    TeleCarlaRpcServer(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void run();
    void stop();

  private:
    void vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr& vehicleStatus);
    void logLatency(const std::string& command, double send_time, double receive_time, double latency);

    rpc::server server_;
    ros::Subscriber vehicleStatusSubscriber_;
    ros::Subscriber clientEnableAutopilotSubscriber_;
    ros::Subscriber clientManualOverrideSubscriber_;
    ros::Subscriber clientVehicleControlCmdSubscriber_;

    ros::Publisher autopilotPublisher_;
    ros::Publisher manualOverridePublisher_;
    ros::Publisher vehicleControlPublisher_;
    ros::Publisher vehicleStatusPublisher_;

    carla_msgs::CarlaEgoVehicleStatusConstPtr egoVehicleStatus_;
};
}  // namespace lmt

#endif  // TELECARLA_RPC_TELECARLA_RPC_SERVER_H

