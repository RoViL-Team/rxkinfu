/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Dimitrios Kanoulas (dkanoulas@gmail.com)
 */

#include <iostream>

#include "kinfu_app_ros.h"

// ROS headers
#include <ros/ros.h>

// PCL headers
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/exceptions.h>

using namespace pcl;
using namespace rxkinfu;
namespace pc = pcl::console;

int main (int argc, char **argv)
{
  // initialize ROS
  ros::init (argc, argv, "rxkinfu_ros");
  ros::NodeHandle node;
  
  if (pc::find_switch(argc, argv, "-h"))
  {
    KinfuAppRos::usageHelp();
    
    return 0;
  }
  else
  {
    pc::print_highlight("run rxkinfu -h for command line help, "
                        "hit h for online help\n");
  }
  
  try
  {
    KinfuAppRos (argc, argv, node).mainLoop();
  }
  catch (const PCLException &e) {
    std::cerr << "PCLException: " << e.what() << std::endl;
  } catch (const std::bad_alloc &e) {
    std::cerr << "Bad alloc: " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  return 0;
}