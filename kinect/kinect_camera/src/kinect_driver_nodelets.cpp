/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010
 *    Radu Bogdan Rusu <rusu@willowgarage.com>
 *
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
 */
#include <pluginlib/class_list_macros.h>
#include "kinect_camera/kinect_nodelets.h"

typedef kinect_camera::KinectDriverNodelet KinectDriverNodelet;

PLUGINLIB_DECLARE_CLASS (kinect_camera, KinectDriverNodelet, KinectDriverNodelet, nodelet::Nodelet);

void
  kinect_camera::KinectDriverNodelet::onInit ()
{
  ros::NodeHandle pnh = getMTPrivateNodeHandle ();
  k = new KinectDriver (pnh);

  int device_id = 0;
  pnh.getParam ("device_id", device_id);

  if (!k->init (device_id))
    return;
  k->start ();

  ros::Duration r (0.0001);
  while (pnh.ok () && k->ok ())
  {
    ros::spinOnce ();
    r.sleep ();
  }
}

