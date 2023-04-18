/*
 # Copyright (c) 2011, Georgia Tech Research Corporation
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #     * Redistributions of source code must retain the above copyright
 #       notice, this list of conditions and the following disclaimer.
 #     * Redistributions in binary form must reproduce the above copyright
 #       notice, this list of conditions and the following disclaimer in the
 #       documentation and/or other materials provided with the distribution.
 #     * Neither the name of the Georgia Tech Research Corporation nor the
 #       names of its contributors may be used to endorse or promote products
 #       derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
 # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT,
 INDIRECT,
 # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 # OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 # LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE
 # OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 # ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 #

 ## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
 ## author Chih-Hung Aaron King (Healthcare Robotics Lab, Georgia Tech.)
 */

// This application listens for a rigid body named 'Tracker' on a remote
// machine.
// The raw data is input to a Extended Kalman Filter (EKF) based estimator
// estimating
// the target position, velocity, orientation and angular velocity. The
// estimated
// quantities and raw data are then published through ROS.

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include "Mocap.h"

using namespace std;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "vrpn_client", ros::init_options::AnonymousName);
  ros::NodeHandle nh("");
  ros::NodeHandle private_nh("~");
  ros::Publisher measured_target_transform_pub_ = 
    nh.advertise<geometry_msgs::TransformStamped>("natnet_client/raw_transform", 1);
  
  int bodyId;
  private_nh.param<int>("body_id", bodyId, 1);

  std::vector<int> idBody{bodyId};
  Mocap* pMocap = new Mocap(idBody);

  ros::Rate loop_rate(1000);  // TODO(gohlp): fix this
  while (ros::ok()) {
    pMocap->mMutexPose.lock();
    std::map<int, bool>& valid = pMocap->GetVaild();
    std::map<int, Sophus::SE3f>& pose = pMocap->GetPose();
    double& time = pMocap->GetFrameTime();
    int& id = pMocap->GetFrameId();
    for(auto &v:valid){
      if(v.second){
        v.second = false;
        Sophus::SE3f p = pose[v.first];
        geometry_msgs::TransformStamped measured_transform;
        measured_transform.header.stamp = ros::Time(time);
        measured_transform.header.seq = id;
        measured_transform.header.frame_id = "optitrack";
        measured_transform.transform.rotation.w = p.unit_quaternion().w();
        measured_transform.transform.rotation.x = p.unit_quaternion().x();
        measured_transform.transform.rotation.y = p.unit_quaternion().y();
        measured_transform.transform.rotation.z = p.unit_quaternion().z();
        measured_transform.transform.translation.x = p.translation().x();
        measured_transform.transform.translation.y = p.translation().y();
        measured_transform.transform.translation.z = p.translation().z();
        measured_target_transform_pub_.publish(measured_transform);
      }
    }
    pMocap->mMutexPose.unlock();

    loop_rate.sleep();
  }
  return 0;
}
