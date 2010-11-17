/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010
 *    Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *    William Morris <morris@ee.ccny.cuny.edu>
 *    Stéphane Magnenat <stephane.magnenat@mavt.ethz.ch>
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

#ifndef KINECT_NODE_KINECT_H_
#define KINECT_NODE_KINECT_H_

#include <libusb.h>
#include <algorithm>

#include <camera_info_manager/camera_info_manager.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread/mutex.hpp>
#include <image_transport/image_transport.h>

extern "C" 
{
  #include "libfreenect.h"
}

namespace kinect_camera
{
  class KinectDriver
  {
    public:
      std::string cam_name_;
      std::string cam_info_url_;

      /** \brief Camera info data. Holds camera parameters. */
      sensor_msgs::CameraInfo cam_info_;
      /** \brief Camera info manager object. */
      CameraInfoManager *cam_info_manager_;

      boost::mutex rgbMutex_;
      boost::mutex depthMutex_;

      /** \brief Constructor */
      KinectDriver (const ros::NodeHandle &nh);
      virtual ~KinectDriver ();

      /** \brief Depth callback. Virtual. 
        * \param dev the Freenect device
        * \param buf the resultant output buffer
        * \param timestamp the time when the data was acquired
        */
      virtual void depthCb (freenect_device *dev, freenect_depth *buf, uint32_t timestamp);
      /** \brief RGB callback. Virtual.
        * \param dev the Freenect device
        * \param buf the resultant output buffer
        * \param timestamp the time when the data was acquired
        */
      virtual void rgbCb   (freenect_device *dev, freenect_pixel *rgb, uint32_t timestamp);

      void start ();
      void stop ();
      bool init (int index);
      void publish ();

      /** \brief Check the current state of Freenect. */
      inline bool ok ()
      {
        return (freenect_process_events (f_ctx_) >= 0);
      }

    private:
      boost::mutex bufferMutex_;

      /** \brief Internal node handle copy. */
      ros::NodeHandle nh_;

      /** \brief Buffer that holds the depth values. */
      uint16_t *depth_buf_;
      /** \brief Buffer that holds the RGB values. */
      uint8_t  *rgb_buf_;

      bool depthSent_;
      bool rgbSent_; 

      /** \brief ROS publishers. */
      image_transport::CameraPublisher pub_image_;
      ros::Publisher pub_points_, pub_points2_;

      int width_;
      int height_;
      double max_range_;
      std::string kinect_RGB_frame_;
      std::string kinect_depth_frame_;

      /** \brief The horizontal field of view (in radians). */
      double horizontal_fov_;
      /** \brief The vertical field of view (in radians). */
      double vertical_fov_;
      
      /** \brief Freenect context structure. */
      freenect_context *f_ctx_;

      /** \brief Freenect device structure. */
      freenect_device *f_dev_;
  };

  KinectDriver* kinect_driver_global;
   
  inline void 
    globalDepthCb (freenect_device *dev, freenect_depth *buf, uint32_t timestamp)
  {
    if (kinect_driver_global)
      kinect_driver_global->depthCb (dev, buf, timestamp);
    else
      ROS_ERROR ("[globalDepthCb] KinectDriver not initialized!");
  }

  inline void 
    globalrgbCb (freenect_device *dev, freenect_pixel *buf, uint32_t timestamp)
  {
    if (kinect_driver_global)
      kinect_driver_global->rgbCb (dev, buf, timestamp);
    else
      ROS_ERROR ("[globalDepthCb] KinectDriver not initialized!");
  }
}

#endif //KINECT_NODE_KINECT_H_
