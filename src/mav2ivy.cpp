/*
 *  mav_tools ros node to ivy bus
 *  Copyright (C) 2010, Gautier Hattenberger (ENAC / Paparazzi)
 *
 *  based on:
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mav2ivy/mav2ivy.h"

#include <string>
#include <sstream>
#include <iostream>

// Templated function to convert from a string
// return 0 on fail
template <class T> bool from_string(T& t, const std::string& s)
{
  std::istringstream iss(s);
  return !(iss >> t).fail();
}

namespace mav2ivy
{

Mav2Ivy::Mav2Ivy(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO("Starting Mav2Ivy"); 

  // NodeHandle for mav data
  ros::NodeHandle nh_data (nh_, mav::ROS_NAMESPACE);

  // **** get parameters

  initializeParams();

  // **** initialize vaiables

  //motors_on_ = false;
  //engaging_ = false;
  // TODO handle motors on
  motors_on_ = true;
  engaging_ = false;

  ctrl_roll_ = 0;
  ctrl_pitch_ = 0;
  ctrl_yaw_ = 0;
  ctrl_thrust_ = 0;

  // *** register publishers

  imu_publisher_  = nh_data.advertise<sensor_msgs::Imu>(mav::IMU_TOPIC, 10);
  height_publisher_ = nh_data.advertise<mav_msgs::Height>(mav::P_HEIGHT_TOPIC, 10);

  // **** register subscribers

  // Replace by IVY stuff
  IvyBindMsg(ivyFPCallback,this, "^(\\S*) ROTORCRAF_FP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*");
	IvyBindMsg(ivyImuCallback, this, "^(\\S*) AHRS_LKF (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(ivyStatusCallback, this, "^(\\S*) ROTORCRAFT_STATUS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");

  //imu_calcdata_subscriber_ = nh_rawdata.subscribe(
  //  asctec::IMU_CALCDATA_TOPIC, 10, &Mav2Ivy::imuCalcDataCallback, this);
  //ll_status_subscriber_ = nh_rawdata.subscribe(
  //  asctec::LL_STATUS_TOPIC, 5, &Mav2Ivy::llStatusCallback, this);

  cmd_thrust_subscriber_ = nh_data.subscribe(
      mav::CMD_THRUST_TOPIC, 1, &Mav2Ivy::cmdThrustCallback, this);
  cmd_roll_subscriber_ = nh_data.subscribe(
      mav::CMD_ROLL_TOPIC, 1, &Mav2Ivy::cmdRollCallback, this);
  cmd_pitch_subscriber_ = nh_data.subscribe(
      mav::CMD_PITCH_TOPIC, 1, &Mav2Ivy::cmdPitchCallback, this);
  cmd_yaw_subscriber_ = nh_data.subscribe(
      mav::CMD_YAW_RATE_TOPIC, 5, &Mav2Ivy::cmdYawCallback, this);

  // **** services

  //set_motors_on_off_srv_ = nh_data.advertiseService(
  //    "setMotorsOnOff", &Mav2Ivy::setMotorsOnOff, this);
  get_motors_on_off_srv_ = nh_data.advertiseService(
    "getMotorsOnOff", &Mav2Ivy::getMotorsOnOff, this);


}

Mav2Ivy::~Mav2Ivy()
{
  ROS_INFO("Destroying Mav2Ivy"); 
}

void Mav2Ivy::initializeParams()
{
  //TODO Adapt to correct values for PPRZ interface
  // Not sure how it works anyway...
  if (!nh_private_.getParam ("max_ctrl_thrust", max_ctrl_thrust_))
    max_ctrl_thrust_ = 200;
  if (!nh_private_.getParam ("max_ctrl_roll", max_ctrl_roll_))
    max_ctrl_roll_ = 100;
  if (!nh_private_.getParam ("max_ctrl_pitch", max_ctrl_pitch_))
    max_ctrl_pitch_ = 100;
  if (!nh_private_.getParam ("max_ctrl_yaw", max_ctrl_yaw_))
    max_ctrl_yaw_ = 100;
}

/*
bool Mav2Ivy::setMotorsOnOff(mav_msgs::SetMotorsOnOff::Request  &req,
                                mav_msgs::SetMotorsOnOff::Response &res)
{
  state_mutex_.lock();
  engaging_ = true;

  if (req.on && !motors_on_)
  {
    ctrl_roll_ = 0;
    ctrl_pitch_ = 0;
    ctrl_yaw_ = 0;
    ctrl_thrust_ = 0;
    startMotors();
  }
  else
  {
    stopMotors();
  }

  engaging_ = false;
  state_mutex_.unlock();

  return (req.on == motors_on_);
}
*/

void Mav2Ivy::setMotorsOnOff(const bool motors_on)
{
  state_mutex_.lock();
  motors_on_ = motors_on;
  state_mutex_.unlock();
}

bool Mav2Ivy::getMotorsOnOff(mav_msgs::GetMotorsOnOff::Request  &req,
                                mav_msgs::GetMotorsOnOff::Response &res)
{
  state_mutex_.lock();
  res.on = motors_on_;
  state_mutex_.unlock();

  return true;
}

void Mav2Ivy::cmdRollCallback(const std_msgs::Float64ConstPtr& cmd_roll_msg)
{
  if (!motors_on_ || engaging_) return;

  state_mutex_.lock();
 
  // translate from cmd_roll [-1.0 to 1.0] to ctrl_roll PPRZ TODO
  ctrl_roll_ = (int)(cmd_roll_msg->data * 100);

  ROS_INFO ("cmd_roll received: %f (%d)", cmd_roll_msg->data, ctrl_roll_);

  // limit min/max output
  if (ctrl_roll_ > max_ctrl_roll_)
  {
    ROS_WARN("ctrl_roll of %d too big, clamping to %d!", ctrl_roll_, max_ctrl_roll_);
    ctrl_roll_ = max_ctrl_roll_;
  }
  else if (ctrl_roll_ < -max_ctrl_roll_)
  {
    ROS_WARN("ctrl_roll of %d too small, clamping to -%d!", ctrl_roll_, -max_ctrl_roll_);
    ctrl_roll_ = -max_ctrl_roll_;
  }

  publishCtrlInputMsg();

  state_mutex_.unlock();
}

void Mav2Ivy::cmdPitchCallback(const std_msgs::Float64ConstPtr& cmd_pitch_msg)
{
  if (!motors_on_ || engaging_) return;

  state_mutex_.lock();
 
  // translate from cmd_pitch [-1.0 to 1.0] to ctrl_pitch PPRZ TODO,
  ctrl_pitch_ = (int)(cmd_pitch_msg->data * 100);

  ROS_DEBUG ("cmd_pitch received: %f (%d)", cmd_pitch_msg->data, ctrl_pitch_);

  // limit min/max output
  if (ctrl_pitch_ > max_ctrl_pitch_)
  {
    ROS_WARN("ctrl_pitch of %d too big, clamping to %d!", ctrl_pitch_, max_ctrl_pitch_);
    ctrl_pitch_ = max_ctrl_pitch_;
  }
  else if (ctrl_pitch_ < -max_ctrl_pitch_)
  {
    ROS_WARN("ctrl_pitch of %d too small, clamping to -%d!", ctrl_pitch_, -max_ctrl_pitch_);
    ctrl_pitch_ = -max_ctrl_pitch_;
  }

  publishCtrlInputMsg();

  state_mutex_.unlock();
}

void Mav2Ivy::cmdYawCallback(const std_msgs::Float64ConstPtr& cmd_yaw_rate_msg)
{
  if (!motors_on_ || engaging_) return;

  state_mutex_.lock();

  // translate from cmd_yaw [rad/s] to ctrl_yaw PPRZ TODO,
  ctrl_yaw_ = (int)(cmd_yaw_rate_msg->data * 1);

  ROS_DEBUG ("cmd_yaw received: %f (%d)", cmd_yaw_rate_msg->data, ctrl_yaw_);

  // limit min/max output
  if (ctrl_yaw_ > max_ctrl_yaw_)
  {
    ROS_WARN("ctrl_yaw of %d too big, clamping to %d!", ctrl_yaw_, max_ctrl_yaw_);
    ctrl_yaw_ = max_ctrl_yaw_;
  }
  else if (ctrl_yaw_ < -max_ctrl_yaw_)
  {
    ROS_WARN("ctrl_yaw of %d too small, clamping to -%d!", ctrl_yaw_, -max_ctrl_yaw_);
    ctrl_yaw_ = -max_ctrl_yaw_;
  }

  publishCtrlInputMsg();

  state_mutex_.unlock();
}

void Mav2Ivy::cmdThrustCallback(const std_msgs::Float64ConstPtr& cmd_thrust_msg)
{
  if (!motors_on_ || engaging_) return;

  state_mutex_.lock();

  // translate from cmd_thrust [0.0 to 1.0] to ctrl_thrust PPRZ TODO,
  ctrl_thrust_ = (int)(cmd_thrust_msg->data * 200);

  ROS_DEBUG ("cmd_thrust received: %f (%d)", cmd_thrust_msg->data, ctrl_thrust_);

  // limit min-max output
  if (ctrl_thrust_ > max_ctrl_thrust_)
  {
    ROS_WARN("ctrl_thrust of %d too big, clamping to %d!", ctrl_thrust_, max_ctrl_thrust_);
    ctrl_thrust_ = max_ctrl_thrust_;
  }
  else if (ctrl_thrust_ < 0)
  {
    ROS_WARN("ctrl_thrust of %d too small, clamping to 0!", ctrl_thrust_);
    ctrl_thrust_ = 0;
  }

  publishCtrlInputMsg();

  state_mutex_.unlock();
}

void Mav2Ivy::publishImuMsg(const sensor_msgs::ImuPtr& imu_msg)
{
  state_mutex_.lock();

  imu_publisher_.publish(imu_msg);

  state_mutex_.unlock();
}

void Mav2Ivy::publishHeightMsg(const mav_msgs::HeightPtr& height_msg)
{
  state_mutex_.lock();

  height_publisher_.publish(height_msg);

  state_mutex_.unlock();
}

// Ivy IMU callback
void ivyImuCallback(IvyClientPtr app, void *data, int argc, char **argv)
{
  Mav2Ivy *m2i = static_cast<Mav2Ivy *>(data);

  // *** Create Imu message
  sensor_msgs::ImuPtr imu_msg;
  imu_msg = boost::make_shared<sensor_msgs::Imu>();

  // set header info
  //imu_msg->header.stamp    = 0;
  imu_msg->header.frame_id = "imu";

  // *** Parse Ivy message (description in PPRZ_HOME/conf/messages.xml)

  // copy orientation quaternion
  from_string<double>(imu_msg->orientation.x, argv[4]);
  from_string<double>(imu_msg->orientation.y, argv[5]);
  from_string<double>(imu_msg->orientation.z, argv[6]);
  from_string<double>(imu_msg->orientation.w, argv[7]);
  // copy angular_velocity
  from_string<double>(imu_msg->angular_velocity.x, argv[8]);
  from_string<double>(imu_msg->angular_velocity.y, argv[9]);
  from_string<double>(imu_msg->angular_velocity.z, argv[10]);
  // copy linear acceleration
  from_string<double>(imu_msg->linear_acceleration.x, argv[11]);
  from_string<double>(imu_msg->linear_acceleration.y, argv[12]);
  from_string<double>(imu_msg->linear_acceleration.z, argv[13]);

  // *** publish imu message
  m2i->publishImuMsg(imu_msg);
}

void ivyStatusCallback(IvyClientPtr app, void *data, int argc, char **argv)
{
  Mav2Ivy *m2i = static_cast<Mav2Ivy *>(data);

  bool motors_on;
  from_string<bool>(motors_on, argv[8]);
  m2i->setMotorsOnOff(motors_on);
}

void ivyFPCallback(IvyClientPtr app, void *data, int argc, char **argv)
{
  Mav2Ivy *m2i = static_cast<Mav2Ivy *>(data);

  mav_msgs::HeightPtr height_msg;
  height_msg = boost::make_shared<mav_msgs::Height>();

  // set header info
  //height_msg->header.stamp    = 0;
  height_msg->header.frame_id = "imu"; // the frame seems arbitrary here

  // TODO convert to correct scale and frame
  from_string<double>(height_msg->height, argv[3]);
  from_string<double>(height_msg->climb, argv[6]);

  // *** publish height message
  m2i->publishHeightMsg(height_msg);
}

void Mav2Ivy::startMotors()
{
  ROS_INFO ("Starting motors...");

  //TODO send resurect message with Ivy

  ROS_INFO("Motors are ON");
}

void Mav2Ivy::stopMotors()
{
  ROS_INFO ("Stopping motors...");

  //TODO send kill message with Ivy

  ROS_INFO("Motors are OFF");
}

void Mav2Ivy::publishCtrlInputMsg()
{
  ROS_DEBUG("Publishing ctrl with Ivy");

  // **** TODO Send with Ivy

}

} // end namespace mav2ivy
