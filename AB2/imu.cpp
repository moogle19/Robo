#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <cmath>

double cnt;
double sumroll, mroll, sdroll, sqsumroll;
double sumpitch, mpitch, sdpitch, sqsumpitch;
double sumyaw, myaw, sdyaw, sqsumyaw;


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ++cnt;
    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    sumroll  += roll;
    sumpitch += pitch;
    sumyaw   += yaw;

    sqsumroll  += (roll*roll);
    sqsumpitch += (pitch*pitch);
    sqsumyaw   += (yaw*yaw);

    //calculate mean
    mroll  = sumroll  / cnt;
    mpitch = sumpitch / cnt;
    myaw   = sumyaw   / cnt; 

    //calculate standard deviation
    sdpitch = sqrt(1.0/(cnt-1.0) * (sqsumpitch - (1.0/cnt) * pow(sumpitch, 2.0)));
    sdroll  = sqrt(1.0/(cnt-1.0) * (sqsumroll  - (1.0/cnt) * pow(sumroll,  2.0)));
    sdyaw   = sqrt(1.0/(cnt-1.0) * (sqsumyaw   - (1.0/cnt) * pow(sumyaw,   2.0)));

    ROS_INFO("Got IMU-Data: (%lf, %lf, %lf)", roll, pitch, yaw);
//    ROS_INFO("IMU-Data -    Mean: (%lf, %lf, %lf)", mroll, mpitch, myaw);
//    ROS_INFO("Standard Deviation: (%lf, %lf, %lf)", sdroll, sdpitch, sdyaw);
}

int main(int argc, char **argv)
{
    cnt = 0.0;
    mroll = mpitch = myaw = 0.0;
    sdroll = sdpitch = sdyaw = 0.0;
    sumroll = sumpitch = sumyaw = 0.0;  

    ros::init(argc, argv, "imu");
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("/imu", 1, imuCallback);
//    ros::Subscriber sub = n.subscribe("/android/imu", 1, imuCallback);
//    ros::Subscriber sub = n.subscribe("/imu_corrected", 1, imuCallback);
    
    ros::spin();
    return 0;
}
