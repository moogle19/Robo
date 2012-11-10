#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
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

    tf::Transform transform;
    tf::Quaternion rot;
    double nowtime = ros::Time::now().toSec(); //set nowtime to current time
    double deltatime = nowtime - lasttime;
    double theta = 0.0;
    double x = 0.0;
    double y = 0.0;

    //convert clicks into m/s 
    double left = msg->left * 0.379 / 21950; //converts to m/s
    double right = msg->right * 0.379 / 21950;

    theta = (((left - right) * deltatime)/0.28) * 0.69; //achslaenge 0.28m winkelkorrektur 0.69

    //calculate x-coord and y-coord
    x = oldx + ((left + right)/2) * deltatime * sin(oldtheta + ((theta/2) * deltatime));
    y = oldy + ((left + right)/2) * deltatime * cos(oldtheta + ((theta/2) * deltatime));

    //std::cout << oldx << std::endl << x << std::endl<<std::endl;

    //debug output
    //std::cout << "Deltatime: " << deltatime << std::endl;
    //std::cout << "OLD: " <<  "Theta: " << oldtheta << " x: " << oldx << " y: " << oldy << std::endl; 
    //std::cout <<  "Theta: " << theta << " x: " << x << " y: " << y << std::endl; 
    

    //save old values
    lasttime = nowtime;
    oldtheta = theta;
    oldx = x;
    oldy = y;


    rot.setEuler(0.0, 0.0, theta);
    transform.setOrigin( tf::Vector3(x, y, 0.0) );
    transform.setRotation( rot );

    //std::cout << "x: " << x << " y: " << y << std::endl;

    //?????????????????? world laser richtig????????????????????????????
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/laser")); 
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