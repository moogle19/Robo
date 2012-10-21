#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

ros::Publisher laser_pub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{

	sensor_msgs::LaserScan median(*scan);

	std::cout <<  "min: " << scan->angle_min << std::endl << "increment: " << scan->angle_increment << std::endl;
	// create a new point cloud
	sensor_msgs::PointCloud cloud;
	cloud.header = scan->header;
	cloud.points.resize(scan->ranges.size());
	// change first element of the cloud
	cloud.points[0].x = 2.0;
	cloud.points[0].y = 1.0;
	int i = 0;
	for(i = 0; i < scan->ranges.size(); i++)
	{
		cloud.points[i].x = scan->ranges[i] * cos(scan->angle_min + (scan->angle_increment)*i);
		cloud.points[i].y = scan->ranges[i] * sin(scan->angle_min + (scan->angle_increment)*i);
	}

	laser_pub.publish(cloud);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "median");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan", 1, laserCallback);
	laser_pub = n.advertise<sensor_msgs::PointCloud>("/cloud", 1);
	ros::spin();
	return 0;
}