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

// Ivy main loop
#include <Ivy/ivyloop.h>

//#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
  // *** Init ros
  ros::init(argc, argv, "mav2ivy");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // ***  Init Ivy
  IvyInit ("mav2ivy", "mav2ivy READY", NULL, NULL, NULL, NULL);

//  // Ivy callback queue and node
//  ros::CallbackQueue ivy_queue;
//  ros::NodeHandle nh_ivy;
//  nh_ivy.setCallbackQueue(&ivy_queue);
//  // Asynchronous spinner for Ivy
//  ros::AsyncSpinner ivy_spinner(1, &ivy_queue);
//  // Create Mav2Ivy object
//  mav2ivy::Mav2Ivy m2i(nh, nh_private, nh_ivy);
//  // Start spinners
//  ivy_spinner.start();
//  ros::spin();

  // *** Start ros (asynchronous multi-threaded calls)
  mav2ivy::Mav2Ivy m2i(nh, nh_private);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  // *** Start Ivy
  IvyStart("127.255.255.255");
  // *** Blocking Ivy main loop
  IvyMainLoop();
  return 0;
}

