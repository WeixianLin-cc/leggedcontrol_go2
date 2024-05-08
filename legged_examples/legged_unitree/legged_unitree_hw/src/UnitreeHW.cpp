
//
// Created by qiayuan on 1/24/22.
//

#include "legged_unitree_hw/UnitreeHW.h"
#include <sensor_msgs/Joy.h>

// #ifdef UNITREE_SDK_3_3_1
// #include "unitree_legged_sdk_3_3_1/unitree_joystick.h"
// #elif UNITREE_SDK_3_8_0
// #include "unitree_legged_sdk_3_8_0/joystick.h"
// #endif




namespace legged {
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }
  robot_hw_nh.getParam("power_limit", powerLimit_);

  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);
  
  InitLowCmd();

//导入数据

  std::string robot_type;
  root_nh.getParam("robot_type", robot_type);

//设置安全性

// #ifdef UNITREE_SDK_3_3_1
//   if (robot_type == "a1") {
//     safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);


  joyPublisher_ = root_nh.advertise<sensor_msgs::Joy>("/joy", 10);
  tempPublisher_ = root_nh.advertise<std_msgs::Float64MultiArray>("motor_temperatures", 10);
  kp_pub = root_nh.advertise<std_msgs::Float64MultiArray>("kp_cmd", 10);
  
  cmd_vel_sub = root_nh.subscribe("/cmd_vel", 1, &UnitreeHW::cmdVelCallback, this);


  
  return true;
}


void UnitreeHW::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received /cmd_vel message");
    ros::Duration(2.0).sleep();  // 等待2秒

    // 在这里修改自定义变量 contact
    contactThreshold_ = contactoriThreshold_;

    // 打印修改后的 contact 值
    ROS_INFO("Modified contact: %d", contactThreshold_);

    // 取消订阅 /cmd_vel 话题
    cmd_vel_sub.shutdown();
    }

void UnitreeHW::InitLowCmd(){

    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }

    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&UnitreeHW::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    //lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &UnitreeHW::write, this);
}






uint32_t UnitreeHW::crc32_core(uint32_t* ptr, uint32_t len)
{
  unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void UnitreeHW::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}


void UnitreeHW::read(const ros::Time& time, const ros::Duration& /*period*/) {

  for (int i = 0; i < 12; ++i) {
    jointData_[i].pos_ = low_state.motor_state()[i].q();
    jointData_[i].vel_ = low_state.motor_state()[i].dq();
    jointData_[i].tau_ = low_state.motor_state()[i].tau_est();
  }
  //温度读到了
  for (int i = 0; i < 12; ++i) {
    jointTempData_[i].temperature_= low_state.motor_state()[i].temperature();
  }
  imuData_.ori_[0] = low_state.imu_state().quaternion()[1];
  imuData_.ori_[1] = low_state.imu_state().quaternion()[2];
  imuData_.ori_[2] = low_state.imu_state().quaternion()[3];
  imuData_.ori_[3] = low_state.imu_state().quaternion()[0];
  imuData_.angularVel_[0] = low_state.imu_state().gyroscope()[0];
  imuData_.angularVel_[1] = low_state.imu_state().gyroscope()[1];
  imuData_.angularVel_[2] = low_state.imu_state().gyroscope()[2];
  imuData_.linearAcc_[0] = low_state.imu_state().accelerometer()[0];
  imuData_.linearAcc_[1] = low_state.imu_state().accelerometer()[1];
  imuData_.linearAcc_[2] = low_state.imu_state().accelerometer()[2];



  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactState_[i] = low_state.foot_force()[i] > contactThreshold_;
  }

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names) {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }
  
  //updateJoystick(time);
  //updateTemp(time);
}

void UnitreeHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  std_msgs::Float64MultiArray kp_msg;
  for (int i = 0; i < 12; ++i) {
    low_cmd.motor_cmd()[i].q() = static_cast<float>(jointData_[i].posDes_);
    low_cmd.motor_cmd()[i].dq() = static_cast<float>(jointData_[i].velDes_);
    low_cmd.motor_cmd()[i].kp() = static_cast<float>(jointData_[i].kp_);
    
    low_cmd.motor_cmd()[i].kd() = static_cast<float>(jointData_[i].kd_);
    low_cmd.motor_cmd()[i].tau() = static_cast<float>(jointData_[i].ff_);
    // kp_msg.data.push_back(low_cmd.motor_cmd()[i].kp());
    // kp_msg.data.push_back(low_cmd.motor_cmd()[i].kd());
    // kp_msg.data.push_back(low_cmd.motor_cmd()[i].q());
    // kp_msg.data.push_back(low_cmd.motor_cmd()[i].dq());
    // kp_msg.data.push_back(low_cmd.motor_cmd()[i].tau());
    //kp_msg.data = {low_cmd.motor_cmd()[2].kp(),low_cmd.motor_cmd()[2].kd(),low_cmd.motor_cmd()[2].q(),low_cmd.motor_cmd()[2].dq(),low_cmd.motor_cmd()[2].tau()};
  }

  for (int i=0;i<12;++i){
    if (low_cmd.motor_cmd()[i].tau()>35)
    {
      low_cmd.motor_cmd()[i].tau() = 35;
    }
  }
  //safety_->PositionLimit(low_cmd);
  //safety_->PowerProtect(low_cmd, low_state, powerLimit_);
  low_cmd.crc() = UnitreeHW::crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    
  lowcmd_publisher->Write(low_cmd);
  // kp_pub.publish(kp_msg);


  // udp_->SetSend(low_cmd);
  // udp_->Send();
}

bool UnitreeHW::setupJoints() {
  for (const auto& joint : urdfModel_->joints_) {
    int leg_index = 0;
    int joint_index = 0;
    if (joint.first.find("RF") != std::string::npos) {
      leg_index = 0;
    } else if (joint.first.find("LF") != std::string::npos) {
      leg_index = 1;
    } else if (joint.first.find("RH") != std::string::npos) {
      leg_index = 2;
    } else if (joint.first.find("LH") != std::string::npos) {
      leg_index = 3;
    } else {
      continue;
    }


    if (joint.first.find("HAA") != std::string::npos) {
      joint_index = 0;
    } else if (joint.first.find("HFE") != std::string::npos) {
      joint_index = 1;
    } else if (joint.first.find("KFE") != std::string::npos) {
      joint_index = 2;
    } else {
      continue;
    }

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                           &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool UnitreeHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("unitree_imu", "unitree_imu", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("contact_threshold", contactThreshold_);
  contactoriThreshold_=contactThreshold_;
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  contactThreshold_=-100;//先设置为-100，回调函数时再修改
  return true;
}
void UnitreeHW::updateTemp(const ros::Time& time) {
  if ((time - lastPub_).toSec() < 1 / 50.) {
    return;
  }
  lastPub_ = time;
  std_msgs::Float64MultiArray motor_temps;
  motor_temps.data.resize(12);
  for (int i = 0; i < 12; ++i) {
    motor_temps.data[i] = jointTempData_[i].temperature_;
  }
  tempPublisher_.publish(motor_temps);
}

}


// void UnitreeHW::updateJoystick(const ros::Time& time) {
//   if ((time - lastPub_).toSec() < 1 / 50.) {
//     return;
//   }
//   lastPub_ = time;
//   xRockerBtnDataStruct keyData;
//   memcpy(&keyData, &lowState_.wirelessRemote[0], 40);
//   sensor_msgs::Joy joyMsg;  // Pack as same as Logitech F710
//   joyMsg.axes.push_back(-keyData.lx);
//   joyMsg.axes.push_back(keyData.ly);
//   joyMsg.axes.push_back(-keyData.rx);
//   joyMsg.axes.push_back(keyData.ry);
//   joyMsg.buttons.push_back(keyData.btn.components.X);
//   joyMsg.buttons.push_back(keyData.btn.components.A);
//   joyMsg.buttons.push_back(keyData.btn.components.B);
//   joyMsg.buttons.push_back(keyData.btn.components.Y);
//   joyMsg.buttons.push_back(keyData.btn.components.L1);
//   joyMsg.buttons.push_back(keyData.btn.components.R1);
//   joyMsg.buttons.push_back(keyData.btn.components.L2);
//   joyMsg.buttons.push_back(keyData.btn.components.R2);
//   joyMsg.buttons.push_back(keyData.btn.components.select);
//   joyMsg.buttons.push_back(keyData.btn.components.start);
//   joyPublisher_.publish(joyMsg);
// }

  // namespace legged
