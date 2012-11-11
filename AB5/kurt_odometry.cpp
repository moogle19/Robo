#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "Encoder.h"

//std::string turtle_name;
ros::Publisher pub;

void poseCallback(const kurt_msgs::EncoderConstPtr& msg){
    static tf::TransformBroadcaster br;

    //static variables to save old state
    static double lasttime = ros::Time::now().toSec();
    static double oldx = 0.0;
    static double oldy = 0.0;
    static double oldtheta = 0.0;
    static double thetasum = 0.0;

    tf::Transform transform;
    tf::Quaternion rot;

    ros::Time rostime = ros::Time::now();
    double nowtime = rostime.toSec(); //set nowtime to current time
    double deltatime = nowtime - lasttime;
    double theta = 0.0;
    double x = 0.0;
    double y = 0.0;


    //convert clicks into m/s 
    double left = msg->left * 0.379 / 21950; //converts to m/s
    double right = msg->right * 0.379 / 21950;

    //std::cout << "LEFT   : " << left << std::endl;
    //std::cout << "RIGHT  : " << right << std::endl <<std::endl;


    theta = (((right - left))/0.28) * deltatime * 0.69; //achslaenge 0.28m winkelkorrektur 0.69

    //calculate x-coord and y-coord
    x = oldx + (((right + left)/2) * deltatime * cos(oldtheta + ((theta/2) * deltatime)));
    y = oldy + (((right + left)/2) * deltatime * sin(oldtheta + ((theta/2) * deltatime)));
    
    //save old values
    lasttime = nowtime;
    oldtheta = theta;
    oldx = x;
    oldy = y;

    rot.setEuler(0.0, 0.0, theta);
    transform.setOrigin( tf::Vector3(x, y, 0.0) );
    transform.setRotation( rot );

    br.sendTransform(tf::StampedTransform(transform, rostime, "/trans", "/laser")); 
}

int main(int argc, char** argv){
    ros::init(argc, argv, "kurt_odometry");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/encoder", 10, &poseCallback);

    ros::spin();
    return 0;
};