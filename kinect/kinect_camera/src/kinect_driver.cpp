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

#include "kinect_camera/kinect.h"

namespace kinect_camera {

/** \brief Constructor */
KinectDriver::KinectDriver (const ros::NodeHandle &nh) : nh_ (nh), width_ (640), height_ (480), max_range_ (5.0), depth_sent_ (false), rgb_sent_ (false)
{
  cam_info_manager_ = new CameraInfoManager (nh_);

  nh_.getParam ("max_range", max_range_);
  nh_.getParam ("width", width_);
  nh_.getParam ("height", height_);

  // Assemble the point cloud data
  std::string kinect_depth_frame ("/kinect_depth");
  nh_.getParam ("kinect_depth_frame", kinect_depth_frame);
  cloud_.header.frame_id = cloud2_.header.frame_id = kinect_depth_frame;
  cloud_.channels.resize (1);
  cloud_.channels[0].name = "rgb";
  cloud_.channels[0].values.resize (0);

  cloud2_.height = height_; cloud2_.width = width_;
  cloud2_.fields.resize (4);
  cloud2_.fields[0].name = "x";
  cloud2_.fields[1].name = "y";
  cloud2_.fields[2].name = "z";
  cloud2_.fields[3].name = "rgb";

  // Set all the fields types accordingly
  int offset = 0;
  for (size_t s = 0; s < cloud2_.fields.size (); ++s, offset += 4)
  {
    cloud2_.fields[s].offset   = offset;
    cloud2_.fields[s].count    = 1;
    cloud2_.fields[s].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud2_.point_step = offset;
  cloud2_.row_step   = cloud2_.point_step * cloud2_.width;
  cloud2_.data.resize (cloud2_.row_step   * cloud2_.height);
  cloud2_.is_dense = true;


  // Assemble the image data
  std::string kinect_RGB_frame ("/kinect_rgb");
  nh_.getParam ("kinect_rgb_frame", kinect_RGB_frame);
  image_.header.frame_id = kinect_RGB_frame;

  image_.height = height_; image_.width = width_;
  image_.encoding = "rgb8";
  image_.step = width_ * 3;
  image_.data.resize (width_ * height_ * 3);
  cam_info_.height = image_.height; cam_info_.width = image_.width;
  cam_info_.header.frame_id = image_.header.frame_id; 

  // Read calibration parameters from disk
  std::string cam_name ("camera"), cam_info_url ("auto");
  nh_.getParam ("camera_name", cam_name);
  nh_.getParam ("camera_info_url", cam_info_url);

  if (cam_info_url.compare ("auto") == 0) 
    cam_info_url = std::string ("file://")+ros::package::getPath (ROS_PACKAGE_NAME)+std::string("/info/calibration.yaml");
  ROS_INFO ("[KinectDriver] Calibration URL: %s", cam_info_url.c_str ());

  if (cam_info_manager_->validateURL (cam_info_url)) 
    cam_info_manager_->loadCameraInfo (cam_info_url);
  else
  {
    ROS_ERROR ("[KinectDriver] Invalid Calibration URL");
    ROS_BREAK ();
  }
  cam_info_manager_->setCameraName (cam_name);
  cam_info_ = cam_info_manager_->getCameraInfo ();

  // Publishers and subscribers
  image_transport::ImageTransport it (nh_);
  pub_image_   = it.advertiseCamera ("image_raw", 1);
  pub_points_  = nh_.advertise<sensor_msgs::PointCloud>("points", 15);
  pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2>("points2", 15);
}

/** \brief Initialize a Kinect device, given an index.
  * \param index the index of the device to initialize
  */
bool
  KinectDriver::init (int index)
{
  // Initialize the USB 
  if (freenect_init (&f_ctx_, NULL) < 0)
  {
    ROS_ERROR ("[KinectDriver::init] Initialization failed!");
    return (false);
  }

  // Get the number of devices available
  int nr_devices = freenect_num_devices (f_ctx_);
  if (nr_devices < 1)
  {
    ROS_WARN ("[KinectDriver::init] No devices found!");
    return (false);
  }
  ROS_DEBUG ("[KinectDriver::init] Number of devices found: %d", nr_devices);
  if (nr_devices <= index)
  {
    ROS_WARN ("[KinectDriver::init] Desired device index (%d) out of bounds (%d)!", index, nr_devices);
    return (false);
  }

  // Open the device
  if (freenect_open_device (f_ctx_, &f_dev_, index) < 0)
  {
    ROS_ERROR ("[KinectDriver::init] Could not open device with index (%d)!", index);
    return (false);
  }

  // Set the appropriate data callbacks
  freenect_set_user(f_dev_, this);
  freenect_set_depth_callback (f_dev_, &KinectDriver::depthCbInternal);
  freenect_set_rgb_callback (f_dev_, &KinectDriver::rgbCbInternal);
  freenect_set_rgb_format (f_dev_, FREENECT_FORMAT_RGB);

  return (true);
}

void KinectDriver::depthCbInternal (freenect_device *dev, freenect_depth *buf, uint32_t timestamp)
{
  KinectDriver* driver = reinterpret_cast<KinectDriver*>(freenect_get_user(dev));
  driver->depthCb(dev, buf, timestamp);
}

void KinectDriver::rgbCbInternal (freenect_device *dev, freenect_pixel *buf, uint32_t timestamp)
{
  KinectDriver* driver = reinterpret_cast<KinectDriver*>(freenect_get_user(dev));
  driver->rgbCb(dev, buf, timestamp);
}

/** \brief Destructor */
KinectDriver::~KinectDriver ()
{
  freenect_close_device (f_dev_);
  freenect_shutdown (f_ctx_);
}

/** \brief Start (resume) the data acquisition process. */
void 
  KinectDriver::start ()
{
  freenect_start_depth (f_dev_);
  freenect_start_rgb (f_dev_);
}

/** \brief Stop (pause) the data acquisition process. */
void 
  KinectDriver::stop ()
{
  freenect_stop_depth (f_dev_);
  freenect_stop_rgb (f_dev_);
}

/** \brief Depth callback. Virtual. 
  * \param dev the Freenect device
  * \param buf the resultant output buffer
  * \param timestamp the time when the data was acquired
  */
void 
  KinectDriver::depthCb (freenect_device *dev, freenect_depth *buf, uint32_t timestamp)
{
  boost::mutex::scoped_lock lock (buffer_mutex_);

  depth_sent_ = false;

  // Convert the data to ROS format
  if (pub_points_.getNumSubscribers () > 0)
  {
    cloud_.points.resize (width_ * height_);
    int nrp = 0;
    // Assemble an ancient sensor_msgs/PointCloud message
    for (int u = 0; u < width_; u += 2) 
    {
      for (int v = 0; v < height_; v += 2)
      {
        if (!getPoint3D (buf, u, v, cloud_.points[nrp].x, cloud_.points[nrp].y, cloud_.points[nrp].z))
          continue;

        nrp++;
      }
    }
    // Resize to the correct number of points
    cloud_.points.resize (nrp);
  }
  if (pub_points2_.getNumSubscribers () > 0)
  {
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    // Assemble an awesome sensor_msgs/PointCloud2 message
    for (int u = 0; u < width_; u += 2) 
    {
      for (int v = 0; v < height_; v += 2)
      {
        float *pstep = (float*)&cloud2_.data[(v * width_ + u) * cloud2_.point_step];
        int d = 0;

        float x, y, z;
        if (getPoint3D (buf, u, v, x, y, z))
        {
          pstep[d++] = x;
          pstep[d++] = y;
          pstep[d++] = z;
          // Fill in RGB
          pstep[d++] = 0;
        }
        else
        {
          pstep[d++] = bad_point;
          pstep[d++] = bad_point;
          pstep[d++] = bad_point;
          // Fill in RGB
          pstep[d++] = 0;
        }
      }
    }
  }
  // Publish only if we have an rgb image too
  if (/*rgb_buf_ && */!rgb_sent_) 
    publish ();
}

/** \brief RGB callback. Virtual.
  * \param dev the Freenect device
  * \param buf the resultant output buffer
  * \param timestamp the time when the data was acquired
  */
void 
  KinectDriver::rgbCb (freenect_device *dev, freenect_pixel *rgb, uint32_t timestamp)
{
  boost::mutex::scoped_lock lock (buffer_mutex_);

  rgb_sent_ = false;

  if (pub_image_.getNumSubscribers () > 0)
  {
    // Copy the image data
    memcpy (&image_.data[0], &rgb[0], width_ * height_ * 3);

    // Check the camera info
    if (cam_info_.height != (size_t)image_.height || cam_info_.width != (size_t)image_.width)
      ROS_DEBUG_THROTTLE (60, "[KinectDriver::rgbCb] Uncalibrated Camera");
  }

  // Publish only if we have a depth image too
  if (/*depth_buf_ && */!depth_sent_) 
    publish ();
}

void 
  KinectDriver::publish ()
{
  /// @todo Do something with the timestamps from the device
  // Get the current time
  ros::Time time = ros::Time::now ();
	cloud_.header.stamp = cloud2_.header.stamp = time;
  image_.header.stamp = cam_info_.header.stamp = time;

  // Publish RGB Image
  if (pub_image_.getNumSubscribers () > 0)
    pub_image_.publish (image_, cam_info_); 

  // Publish the PointCloud messages
  if (pub_points_.getNumSubscribers () > 0)
    pub_points_.publish  (cloud_);
  if (pub_points2_.getNumSubscribers () > 0)
    pub_points2_.publish (cloud2_);

  rgb_sent_   = true;
  depth_sent_ = true;
}

} // namespace kinect_camera
