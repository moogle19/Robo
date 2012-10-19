#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

int count = 0;
double rollsum, rollsumquad, pitchsum, pitchsumquad, yawsum, yawsumquad;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	tf::Quaternion q;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(msg->orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	rollsum += roll;
	pitchsum += pitch;
	yawsum += yaw;

	rollsumquad += roll * roll;
	pitchsumquad += pitch * pitch;
	yawsumquad += yaw * yaw;

	++count;

	if(count > 1)
	{
		roll = sqrt((1/(count-1))*(rollsumquad-((1/count)*(rollsum*rollsum))));
		pitch = sqrt((1/(count-1))*(pitchsumquad-((1/count)*(pitchsum*pitchsum))));
		yaw = sqrt((1/(count-1))*(yawsumquad-((1/count)*(yawsum*yawsum))));
	}
	ROS_INFO("Got IMU-Data: (%lf, %lf, %lf)", roll, pitch, yaw);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/imu_corrected", 1, imuCallback);
	
	ros::spin();
	return 0;
}

