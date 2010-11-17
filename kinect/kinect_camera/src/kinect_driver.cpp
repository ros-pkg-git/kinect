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


/** \brief Constructor */
kinect_camera::KinectDriver::KinectDriver (const ros::NodeHandle &nh) : nh_ (nh), width_ (640), height_ (480), max_range_ (5.0), kinect_RGB_frame_ ("/kinect_rgb"), kinect_depth_frame_ ("/kinect_depth")
{
  cam_info_manager_ = new CameraInfoManager (nh_);

  // Kinect constants
  double horizontal_fov = 57.0;
	double vertical_fov   = 43.0;
	double deg2rad = M_PI / 180.0;

	horizontal_fov_ = horizontal_fov * deg2rad;
	vertical_fov_   = vertical_fov   * deg2rad;

  rgb_buf_   = NULL;
  depth_buf_ = NULL;

  depthSent_ = false;
  rgbSent_ = false; 

  kinect_driver_global = this;

  // Read in global parameters
  nh_.getParam ("kinect_rgb_frame", kinect_RGB_frame_);
  nh_.getParam ("kinect_depth_frame", kinect_depth_frame_);
  nh_.getParam ("max_range", max_range_);
  nh_.getParam ("width", width_);
  nh_.getParam ("height", height_);

  nh_.param ("camera_name", cam_name_, std::string("camera"));
  nh_.param ("camera_info_url",cam_info_url_,std::string("auto"));
  if (cam_info_url_.compare ("auto") == 0) 
    cam_info_url_ = std::string ("file://")+ros::package::getPath (ROS_PACKAGE_NAME)+std::string("/info/calibration.yaml");
  ROS_INFO ("Calibration URL: %s", cam_info_url_.c_str ());

  if (cam_info_manager_->validateURL (cam_info_url_)) 
    cam_info_manager_->loadCameraInfo (cam_info_url_);
  else
  {
    ROS_ERROR ("Invalid Calibration URL");
    ROS_BREAK ();
  }
  cam_info_manager_->setCameraName (cam_name_);

  // Publishers and subscribers
  image_transport::ImageTransport it (nh_);
  pub_image_   = it.advertiseCamera ("image_raw", 1);
  pub_points_  = nh_.advertise<sensor_msgs::PointCloud>("points", 15);
  pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2>("points2", 15);
}

bool
  kinect_camera::KinectDriver::init (int index)
{
  // Initilize the USB 
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
  freenect_set_depth_callback (f_dev_, kinect_camera::globalDepthCb);
  freenect_set_rgb_callback (f_dev_, kinect_camera::globalrgbCb);
  freenect_set_rgb_format (f_dev_, FREENECT_FORMAT_RGB);

  return (true);
}

kinect_camera::KinectDriver::~KinectDriver ()
{
  freenect_close_device (f_dev_);
  freenect_shutdown (f_ctx_);
  kinect_driver_global = NULL;
}

void kinect_camera::KinectDriver::start ()
{
  freenect_start_depth (f_dev_);
  freenect_start_rgb (f_dev_);
}

void kinect_camera::KinectDriver::stop ()
{
  freenect_stop_depth (f_dev_);
  freenect_stop_rgb (f_dev_);
}

void 
  kinect_camera::KinectDriver::depthCb (freenect_device *dev, freenect_depth *buf, uint32_t timestamp)
{
  bufferMutex_.lock ();

  depthSent_ = false;

  if (depth_buf_) delete depth_buf_;
  depth_buf_ = new uint16_t[width_*height_];
  memcpy(depth_buf_, buf, width_*height_*sizeof(uint16_t));

  if (rgb_buf_ && !rgbSent_) publish();

  bufferMutex_.unlock();
}

void 
  kinect_camera::KinectDriver::rgbCb (freenect_device *dev, freenect_pixel *rgb, uint32_t timestamp)
{
  bufferMutex_.lock();

  rgbSent_ = false;
  if (rgb_buf_) delete rgb_buf_;
  rgb_buf_ = new uint8_t[width_*height_*3];
  memcpy(rgb_buf_, rgb, width_*height_*3);

  if (depth_buf_ && !depthSent_) publish();

  bufferMutex_.unlock();
}

void kinect_camera::KinectDriver::publish ()
{
  ros::Time time = ros::Time::now();

  // **** publish point cloud

	sensor_msgs::PointCloud cloud;
	sensor_msgs::PointCloud2 cloud2;
	cloud.header.stamp = cloud2.header.stamp = time;
	cloud.header.frame_id = cloud2.header.frame_id = kinect_depth_frame_;

	for (int x=0; x<width_; x+=2) 
	for (int y=0; y<height_; y+=2)
  {
    int index(y*width_ + x);
    int reading = depth_buf_[index];

    if (reading  >= 2048 || reading <= 0) continue;

    int px = x - width_/2;
    int py = y - height_/2;
  
    double wx = px * (horizontal_fov_ / (double)width_);
    double wy = py * (vertical_fov_ / (double)height_);
    double wz = 1.0;
  
    double range = -325.616 / ((double)reading + -1084.61);
    
    if (range > max_range_ || range <= 0) 
      continue;

    wx *= range;
    wy *= range;
    wz *= range;

	  geometry_msgs::Point32 point;
    point.x = wx;
    point.y = wy;
    point.z = wz;
  	cloud.points.push_back(point);

    /*pcl::PointXYZ pt;
    pt.x = wx; pt.y = wy; pt.z = wz;
  	pcl_cloud.points.push_back (pt);*/
  }
  //pcl::toROSMsg (pcl_cloud, cloud2);

  // **** publish RGB Image

	sensor_msgs::Image image;
	image.header.stamp = time;
	image.header.frame_id = kinect_RGB_frame_;
	image.height = height_;
	image.width = width_;
	image.encoding = "rgb8";
	image.step = width_ * 3;
	image.data.reserve(width_*height_*3);
  
	copy(rgb_buf_, rgb_buf_ + width_*height_*3, back_inserter(image.data));

  // **** publish Camera Info
  cam_info_ = cam_info_manager_->getCameraInfo ();
  if (cam_info_.height != (unsigned int)image.height || cam_info_.width != (unsigned int)image.width)
    ROS_DEBUG_THROTTLE (60, "Uncalibrated Camera");
  cam_info_.header.stamp = image.header.stamp;
  cam_info_.height = image.height;
  cam_info_.width = image.width;
  cam_info_.header.frame_id = image.header.frame_id;

  // **** publish the messages

	pub_points_.publish  (cloud);
	pub_points2_.publish (cloud2);
	pub_image_.publish   (image, cam_info_); 

  rgbSent_   = true;
  depthSent_ = true;
}
