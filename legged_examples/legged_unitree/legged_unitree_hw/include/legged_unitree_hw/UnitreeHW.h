
//
// Created by qiayuan on 1/24/22.
//

#pragma once
#include "std_msgs/Float64MultiArray.h"
#include <legged_hw/LeggedHW.h>



#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include "geometry_msgs/Twist.h"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
using namespace unitree::common;
using namespace unitree::robot;

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);




namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct UnitreeMotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};


struct UnitreeMotorTempData {
  int8_t temperature_;
};



struct UnitreeImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};





class UnitreeHW : public LeggedHW {
 public:
  UnitreeHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

  void updateJoystick(const ros::Time& time);
  void updateTemp(const ros::Time& time);
  
  UnitreeMotorTempData jointTempData_[12]{};
  
 private:
  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  void InitLowCmd();

  // std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
  // std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  // UNITREE_LEGGED_SDK::LowState lowState_{};
  // UNITREE_LEGGED_SDK::LowCmd lowCmd_{};
  
  unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
  unitree_go::msg::dds_::LowState_ low_state{};  // default init

    /*publisher*/
  ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
  ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
  ThreadPtr lowCmdWriteThreadPtr;

  UnitreeMotorData jointData_[12]{};  // NOLINT(modernize-avoid-c-arrays)
    // NOLINT(modernize-avoid-c-arrays)


  UnitreeImuData imuData_{};
  bool contactState_[4]{};  // NOLINT(modernize-avoid-c-arrays)

  int powerLimit_{};
  int contactThreshold_{};
  int contactoriThreshold_{};


  uint32_t crc32_core(uint32_t* ptr, uint32_t len);
  
  void LowStateMessageHandler(const void* message);


  ros::Publisher joyPublisher_;
  ros::Publisher tempPublisher_;
  ros::Publisher kp_pub;

  ros::Subscriber cmd_vel_sub;
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);


  ros::Time lastPub_;
};

}  // namespace legged
