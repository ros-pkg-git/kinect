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
#include <boost/thread/mutex.hpp>

// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <kinect_camera/KinectConfig.h>

//@todo: warnings about deprecated header? 
#include <image_geometry/pinhole_camera_model.h>

#include <Eigen/Core>

extern "C" 
{
  #include "libfreenect.h"
}

namespace kinect_camera
{
  class KinectDriver
  {
    public:
      /** \brief Constructor */
      KinectDriver (ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
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

      /** \brief IR callback. Virtual.
        * \param dev the Freenect device
        * \param buf the resultant output buffer
        * \param timestamp the time when the data was acquired
        */
      virtual void irCb    (freenect_device *dev, freenect_pixel_ir *rgb, uint32_t timestamp);

      /// @todo Replace explicit stop/start with subscriber callbacks
      /** \brief Start (resume) the data acquisition process. */
      void start ();
      /** \brief Stop (pause) the data acquisition process. */
      void stop ();

      /** \brief Initialize a Kinect device, given an index.
        * \param index the index of the device to initialize
        */
      bool init (int index);

      /** \brief Check whether it's time to exit. 
        * \return true if we're still OK, false if it's time to exit
        */
      inline bool 
        ok ()
      {
        freenect_raw_device_state *state;
        freenect_update_device_state (f_dev_);
        state = freenect_get_device_state (f_dev_);
        freenect_get_mks_accel (state, &accel_x_, &accel_y_, &accel_z_);
        tilt_angle_ = freenect_get_tilt_degs(state);
        //ROS_INFO("tilt angle: %g", tilt_angle_);
        publishImu();
        return (freenect_process_events (f_ctx_) >= 0);
      }

      void processRgbAndDepth();

    protected:
      /** \brief Send the data over the network. */
      void publish ();

      void publishImu();

    private:
      /** \brief Internal mutex. */
      boost::mutex buffer_mutex_;

      /** \brief ROS publishers. */
      image_transport::CameraPublisher pub_rgb_, pub_depth_, pub_ir_;
      image_transport::Publisher pub_rgb_rect_;
      ros::Publisher pub_depth_points_, pub_depth_points2_;
      ros::Publisher pub_rgb_points_, pub_rgb_points2_;
      ros::Publisher pub_imu_;

      /** \brief Camera info manager objects. */
      boost::shared_ptr<CameraInfoManager> rgb_info_manager_, depth_info_manager_;
      image_geometry::PinholeCameraModel rgb_model_, depth_model_;

      /** \brief Dynamic reconfigure. */
      typedef kinect_camera::KinectConfig Config;
      typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
      ReconfigureServer reconfigure_server_;
      Config config_;

      /** \brief Camera parameters. */
      int width_;
      int height_;
      double shift_offset_;
      double baseline_; // between IR projector and depth camera
      Eigen::Matrix<double, 3, 4> depth_to_rgb_;
      Eigen::Matrix4d S;
      static const double SHIFT_SCALE;
      
      /** \brief Freenect context structure. */
      freenect_context *f_ctx_;

      /** \brief Freenect device structure. */
      freenect_device *f_dev_;

      /** \brief True if we're acquiring images. */
      bool started_;

      /// @todo Get rid of member messages since we allocate a shared_ptr each time
      /** \brief Image data. */
      sensor_msgs::Image rgb_image_, depth_image_;
      /** \brief PointCloud data. */
      sensor_msgs::PointCloud cloud_, cloud_rgb_;
      /** \brief PointCloud2 data. */
      sensor_msgs::PointCloud2 cloud2_, cloud2_rgb_;
      /** \brief Camera info data. */
      sensor_msgs::CameraInfo rgb_info_, depth_info_;
      /** \brief Accelerometer data. */
      sensor_msgs::Imu imu_msg_;
      cv::Mat rgb_rect_;

      bool depth_sent_;
      bool rgb_sent_;
      freenect_depth *depth_buf_;
      freenect_pixel *rgb_buf_;

      /** \brief Accelerometer data */
      double accel_x_, accel_y_, accel_z_;

      /** \brief Tilt sensor */
      double tilt_angle_; // [deg]

      /** \brief Callback for dynamic_reconfigure */
      void configCb (Config &config, uint32_t level);

      void updateDeviceSettings();
    
      static void depthCbInternal (freenect_device *dev, void *buf, uint32_t timestamp);

      static void rgbCbInternal (freenect_device *dev, freenect_pixel *buf, uint32_t timestamp);

      static void irCbInternal (freenect_device *dev, freenect_pixel_ir *buf, uint32_t timestamp);

      /** \brief Fills in the depth_image_ data with color from the depth buffer.
        * The color is linear with the z-depth of the pixel, scaling up to max_range_
        * \param buf the depth buffer
        */
      void depthBufferTo8BitImage(const freenect_depth * buf);
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

} // namespace kinect_camera

#endif //KINECT_NODE_KINECT_H_
