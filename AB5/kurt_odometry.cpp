#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include "Encoder.h"

//std::string turtle_name;
ros::Publisher pub;

void poseCallback(const kurt_msgs::EncoderConstPtr& msg){
    static tf::TransformBroadcaster br;

    //static variables to save old state
    static double lasttime = 0;
    static double oldx = 0;
    static double oldy = 0;
    static double oldtheta = 0;

    tf::Transform transform;
    tf::Quaternion rot;
    double nowtime = ros::Time::now().toSec();
    double deltatime = nowtime - lasttime;
    double theta = 0, x = 0, y = 0;

    /*std::cout << (msg->left) << std::endl;
    std::cout << (msg->right) << std::endl;
    std::cout << (msg->header) << std::endl;*/
    double left = msg->left * 0.379 / 21950; //converts to m/s
    double right = msg->right * 0.379 / 21950;

    theta = ((left - right) * deltatime)/0.28; //achslaenge 0,28m
    x = oldx + (left + right)/2 * deltatime * sin(oldtheta + theta/2 * deltatime);
    y = oldy + (left + right)/2 * deltatime * cos(oldtheta + theta/2 * deltatime);

    std::cout << "OLD: " <<  "Theta: " << oldtheta << " x: " << oldx << " y: " << oldy << std::endl; 

    std::cout <<  "Theta: " << theta << " x: " << x << " y: " << y << std::endl; 
    //x = x(n-1) + v * t * sin( theta(n-1) + (theta/2) * t )
    //z = z(n-1) + v * t * cos( theta(n-1) + (theta/2) * t )

    lasttime = nowtime;
    oldtheta = theta;
    oldx = x;
    oldy = y;
    rot.setEuler(0.0, 0.0, theta);
    transform.setOrigin( tf::Vector3(x, y, 0.0) );
    transform.setRotation( rot );

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "encoder"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "kurt_odometry");
    /*if (argc != 2)
    {
        ROS_ERROR("need turtle name as argument"); 
        return -1;
    };
    turtle_name = argv[1];*/

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/encoder", 10, &poseCallback);

    //pub = node.advertise<kurt_msgs::TF>("/encoder", 1);

    ros::spin();
    return 0;
};