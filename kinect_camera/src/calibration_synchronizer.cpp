#include <ros/ros.h>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_synchronizer");
  
  ros::NodeHandle nh("camera");
  ros::Publisher pub_rgb = nh.advertise<sensor_msgs::Image>("rgb/image_raw_synch", 1);
  ros::Publisher pub_ir  = nh.advertise<sensor_msgs::Image>("ir/image_raw_synch", 1);
  
  while (ros::ok()) {
    sensor_msgs::ImageConstPtr rgb = ros::topic::waitForMessage<sensor_msgs::Image>("rgb/image_raw", nh);
    sensor_msgs::ImageConstPtr ir  = ros::topic::waitForMessage<sensor_msgs::Image>("ir/image_raw", nh);
    boost::const_pointer_cast<sensor_msgs::Image>(rgb)->header.stamp = ir->header.stamp;
    pub_rgb.publish(rgb);
    pub_ir.publish(ir);
    ros::spinOnce();
  }
}
