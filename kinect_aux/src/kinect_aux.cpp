#include <libusb.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

#define MS_MAGIC_VENDOR 0x45e
#define MS_MAGIC_MOTOR_PRODUCT 0x02b0
#define GRAVITY 9.80665
#define FREENECT_COUNTS_PER_G 819.

ros::Publisher pub_imu;

libusb_device_handle *openAux(int index = 0)
{
	libusb_device_handle* dev(0);
	libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	ssize_t cnt = libusb_get_device_list (0, &devs); //get the list of devices
	if (cnt < 0)
	{
		ROS_ERROR("No device on USB");
		return 0;
	}
	
	int nr_mot(0);
	for (int i = 0; i < cnt; ++i)
	{
		struct libusb_device_descriptor desc;
		const int r = libusb_get_device_descriptor (devs[i], &desc);
		if (r < 0)
			continue;

		// Search for the aux
		if (desc.idVendor == MS_MAGIC_VENDOR && desc.idProduct == MS_MAGIC_MOTOR_PRODUCT)
		{
			// If the index given by the user matches our camera index
			if (nr_mot == index)
			{
				if ((libusb_open (devs[i], &dev) != 0) || (dev == 0))
				{
					ROS_ERROR_STREAM("Cannot open aux " << index);
					return 0;
				}
				// Claim the aux
				libusb_claim_interface (dev, 0);
				break;
			}
			else
				nr_mot++;
		}
	}

	libusb_free_device_list (devs, 1);  // free the list, unref the devices in it

	return dev;
}


void publishState(libusb_device_handle *dev)
{
	uint8_t buf[10];
	int ret = libusb_control_transfer(dev, 0xC0, 0x32, 0x0, 0x0, buf, 10, 0);
	if (ret != 10)
	{
		ROS_ERROR_STREAM("Error in accelerometer reading, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}
	
	const uint16_t ux = ((uint16_t)buf[2] << 8) | buf[3];
	const uint16_t uy = ((uint16_t)buf[4] << 8) | buf[5];
	const uint16_t uz = ((uint16_t)buf[6] << 8) | buf[7];
	
	const int16_t accelerometer_x = (int16_t)ux;
	const int16_t accelerometer_y = (int16_t)uy;
	const int16_t accelerometer_z = (int16_t)uz;
	const int8_t tilt_angle = (int8_t)buf[8];
	const uint8_t tilt_status = buf[9];
	
	// publish IMU
	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp = ros::Time::now();
	imu_msg.linear_acceleration.x = (double(accelerometer_x)/FREENECT_COUNTS_PER_G)*GRAVITY;
	imu_msg.linear_acceleration.y = (double(accelerometer_y)/FREENECT_COUNTS_PER_G)*GRAVITY;
	imu_msg.linear_acceleration.z = (double(accelerometer_z)/FREENECT_COUNTS_PER_G)*GRAVITY;
	imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4]
		= imu_msg.linear_acceleration_covariance[8] = 0.01; // @todo - what should these be?
	imu_msg.angular_velocity_covariance[0] = -1; // indicates angular velocity not provided
	imu_msg.orientation_covariance[0] = -1; // indicates orientation not provided
	if (pub_imu.getNumSubscribers() > 0)
		pub_imu.publish(imu_msg);
	
	// publish tilt angle
	// TODO
}

/*
double freenect_get_tilt_degs(freenect_raw_device_state *state)
{
	return ((double)state->tilt_angle) / 2.;
}

int freenect_set_tilt_degs(libusb_device_handle *dev, double angle)
{
	int ret;
	uint8_t empty[0x1];

	angle = (angle<MIN_TILT_ANGLE) ? MIN_TILT_ANGLE : ((angle>MAX_TILT_ANGLE) ? MAX_TILT_ANGLE : angle);
	angle = angle * 2;

	ret = fnusb_control(&dev->usb_aux, 0x40, 0x31, (uint16_t)angle, 0x0, empty, 0x0);
	return ret;
}

int freenect_set_led(libusb_device_handle *dev, freenect_led_options option)
{
	int ret;
	uint8_t empty[0x1];
	ret = fnusb_control(&dev->usb_aux, 0x40, 0x06, (uint16_t)option, 0x0, empty, 0x0);
	return ret;
}
*/




int main(int argc, char* argv[])
{
	int ret = libusb_init(0);
	if (ret)
	{
		ROS_ERROR_STREAM("Cannot initialize libusb, error: " << ret);
		return 1;
	}
	
	libusb_device_handle *aux = openAux();
	if (!aux)
	{
		ROS_ERROR_STREAM("No valid aux device found");
		libusb_exit(0);
		return 2;
	}
	
	ros::init(argc, argv, "kinect_aux");
	ros::NodeHandle n;
	
	pub_imu = n.advertise<sensor_msgs::Imu>("imu", 15);
	while (ros::ok())
	{
		ros::spinOnce();
		publishState(aux);
	}
	
	libusb_exit(0);
	return 0;
}