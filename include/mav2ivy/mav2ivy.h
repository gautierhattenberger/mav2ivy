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

#ifndef MAV2IVY_H
#define MAV2IVY_H

#include <ros/ros.h>
#include <ros/timer.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mav_msgs/common.h>
#include <mav_msgs/State.h>
#include <mav_msgs/Height.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread/mutex.hpp>
//#include <tf/transform_datatypes.h>

//#include <mav_msgs/SetMotorsOnOff.h>
#include <mav_msgs/GetMotorsOnOff.h>

// Ivy
#include <Ivy/ivy.h>

namespace mav2ivy
{

class Mav2Ivy
{
  private:

    // **** ROS-related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber cmd_thrust_subscriber_;
    ros::Subscriber cmd_roll_subscriber_;
    ros::Subscriber cmd_pitch_subscriber_;
    ros::Subscriber cmd_yaw_subscriber_;
    ros::Subscriber state_subscriber_;

    ros::Publisher imu_publisher_;
    ros::Publisher height_publisher_;

    //ros::ServiceServer set_motors_on_off_srv_;
    ros::ServiceServer get_motors_on_off_srv_;

    // **** state variables

    boost::mutex state_mutex_;

    int ctrl_roll_;
    int ctrl_pitch_;
    int ctrl_yaw_;
    int ctrl_thrust_;

    int max_ctrl_thrust_;   // max output - in PPRZ units
    int max_ctrl_roll_; 
    int max_ctrl_pitch_;
    int max_ctrl_yaw_;
  
    bool motors_on_;
    bool engaging_;

    // **** member functions

    void initializeParams();

    void cmdThrustCallback(const std_msgs::Float64ConstPtr& cmd_thrust_msg);
    void cmdRollCallback  (const std_msgs::Float64ConstPtr& cmd_roll_msg);
    void cmdPitchCallback (const std_msgs::Float64ConstPtr& cmd_pitch_msg);
    void cmdYawCallback   (const std_msgs::Float64ConstPtr& cmd_yaw_rate_msg);
    void stateCallback    (const mav_msgs::StatePtr&        state_msg);

    // TODO Implement Ivy message
    void startMotors();
    void stopMotors();
    void publishCtrlInputMsg();

    //bool setMotorsOnOff(mav_msgs::SetMotorsOnOff::Request  &req,
    //                    mav_msgs::SetMotorsOnOff::Response &res);
    bool getMotorsOnOff(mav_msgs::GetMotorsOnOff::Request  &req,
                        mav_msgs::GetMotorsOnOff::Response &res);

  public:

    Mav2Ivy(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~Mav2Ivy();

    void publishImuMsg(const sensor_msgs::ImuPtr& imu_msg);
    void publishHeightMsg(const mav_msgs::HeightPtr& height_msg);

    // Change motor status
    void setMotorsOnOff(const bool motors_on);
};

// Ivy Callback on FLIGHT_PARAM message
void ivyFPCallback(IvyClientPtr app, void *data, int argc, char **argv);
// Ivy Callback on IMU message
void ivyImuCallback(IvyClientPtr app, void *data, int argc, char **argv);
// Ivy Callback on STATUS message
void ivyStatusCallback(IvyClientPtr app, void *data, int argc, char **argv);

} // end namespace mav2ivy

#endif // MAV2IVY_H
