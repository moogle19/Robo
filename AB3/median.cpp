#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher laser_pub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	static int count = 0;
	//ROS_INFO("Got a laser scan with %d values and the 5th value is %f", scan->ranges.size(), scan->ranges[5]);

	//copy scan so we can modify it
	sensor_msgs::LaserScan median(*scan);
  
	//set first measurement 2 meter
	median.ranges[0] = 2.0;
	//std::cout << median << std::endl;
	for(count = 0;count < median.ranges.size();count++)
	{
		if(count >= 1 && count < median.ranges.size()-1)
		{
			if(median.ranges[count - 1] < median.ranges[count])
			{
				if(median.ranges[count +1] < median.ranges[count - 1])
				{
					median.ranges[count] = median.ranges[count - 1];
				}
				else if(median.ranges[count + 1] < median.ranges[count])
				{
					median.ranges[count] = median.ranges[count + 1];
				}
			}
			else
			{
				if(median.ranges[count -1] < median.ranges[count + 1])
				{
					median.ranges[count] = median.ranges[count - 1];
				}
				else if(median.ranges[count] < median.ranges[count + 1])
				{
					median.ranges[count] = median.ranges[count + 1];
				}
			}
		}
	}
	
	std::cout << count << std::endl;
	laser_pub.publish(median);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "median");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan", 1, laserCallback);
	laser_pub = n.advertise<sensor_msgs::LaserScan>("/scan_median", 1);
	ros::spin();
	return 0;
}