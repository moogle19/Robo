#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	tf::Quaternion q;
	double roll,pitch,yaw;
	tf::quaternionMsgToTF(msg->orientation,q);
	tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
	ROS_INFO("Got IMU−Data:(%lf,%lf,%lf)",roll,pitch,yaw);
}
int main(int argc, char** argv)
{
	ros::init(argc,argv,"imu");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/imucorrected",1,imuCallback);
	ros::spin();
	return 0 ;
}

