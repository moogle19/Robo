#include <Eigen/Core>
#include <Eigen/LU>
#include <iostream>
#include <ros/ros.h>
#include <kurt_msgs/Encoder.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>


float k = 0.0, sig = 0.0;

void encoderCallback(const kurt_msgs::Encoder::ConstPtr& encoder, const sensor_msgs::Imu::ConstPtr& imu)
{
	  std::cout << "tst" << std::endl;
	  tf::Quaternion q;
	  double roll, pitch, yaw;
	  tf::quaternionMsgToTF(imu->orientation, q);
	  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	  const int ticks_per_turn_of_wheel = 21950;
	  const double wheel_perimeter = 0.379;
	  const double axis_length = 0.28;
	  const double turning_adaptation = 0.69;

	  static tf::TransformBroadcaster br;

	  static double x = 0.0;
	  static double y = 0.0;
	  static double theta = 0.0;
	  static ros::Time last = encoder->header.stamp;

	  if(encoder->header.stamp < last) {
	    x = 0.0;
	    y = 0.0;
	    theta = 0.0;
	  }
	  last = encoder->header.stamp;

	  double wheel_L = wheel_perimeter * encoder->left / ticks_per_turn_of_wheel;
	  double wheel_R = wheel_perimeter * encoder->right / ticks_per_turn_of_wheel;

	  double dtheta = (wheel_R - wheel_L) / axis_length * turning_adaptation;
	  double hypothenuse = 0.5 * (wheel_L + wheel_R);


	  double deltax = hypothenuse * cos(theta + dtheta * 0.5);
	  double deltay = hypothenuse * sin(theta + dtheta * 0.5);
	  x += deltax;
	  y += deltay;

	  k = pow((dtheta+0.1/10), 2.0)/(pow((dtheta+0.1/10), 2.0)+pow(0.004,2.0));
	  sig = (1-k)*(sig+pow((dtheta+0.1/10), 2.0));
	  dtheta = (theta + dtheta) + k*(yaw - (theta + dtheta));

	  theta += dtheta;

	  //br.sendTransform(tf::StampedTransform(transform, encoder->header.stamp, "/odom_combined", encoder->header.frame_id));

	  //f() bzw x t+1
	  static Eigen::Vector3f dxt1;
	  dxt1 << 0, 0, 0;

	  Eigen::Matrix3f mat;
	  mat << cos(dxt1.z()), sin(dxt1.z()), 0, -sin(dxt1.z()), cos(dxt1.z()), 0, 0, 0, 1;

	  Eigen::Vector3f delta;
	  delta << deltax, deltay, dtheta;

	  dxt1 = dxt1 + (mat * delta);

	  std::cout << dxt1;

	  Eigen::Vector3f h(0, 0, 0);
	  static float xsum = 0, ysum = 0,zsum = 0;
	  static int count = 1;
	  xsum += dxt1.x();
	  ysum += dxt1.y();
	  zsum += dxt1.z();

	  //h()
	  h << xsum/count, ysum/count, zsum/count;
	  ++count;

	  //F Matrix
	  Eigen::Matrix3f upperf;
	  upperf << 1, 0, -sin(dxt1.z()) * deltax + deltay, cos(dxt1.z()),
	  			-cos(dxt1.z()) * deltax - deltay, sin(dxt1.z()),
	  			0, 0, 1;

	  //H Matrix
	  Eigen::Matrix3f upperh;
	  upperh << 1, 0, 0,
	  			0, 1, 0,
	  			0, 0, 1;  

	  static Eigen::Matrix3f sigma;
	  sigma << 0,0,0,0,0,0,0,0,0;
	  Eigen::Matrix3f errormat;
	  errormat << 0.01, 0, 0, 0, 0.04, 0, 0, 0, 0.64;

	  sigma = upperf * sigma * upperf.transpose() + errormat;

	  Eigen::Matrix3f kgain;
	  Eigen::Matrix3f matrix;
	  matrix << pow(0.3, 2), 0, 0, 0, pow(0.3, 2), 0, 0, 0, pow(0.4, 2);
	  kgain = sigma * (sigma + matrix).inverse();

	  Eigen::Vector3f ret;

	  ret = dxt1;// + kgain * (-dxt1)

	  tf::Vector3 trans(ret.x(), ret.y(), 0.0);
	  tf::Quaternion rot;
	  rot.setEuler(0.0, 0.0, ret.z());
	  tf::Transform transform;
	  transform.setOrigin(trans);
	  transform.setRotation(rot);

	  br.sendTransform(tf::StampedTransform(transform, encoder->header.stamp, "/odom_combined", encoder->header.frame_id));


}


int main(int argc, char **argv)
{
	/*Eigen::Matrix3d M;
	M = Eigen::Matrix3d::Identity();
	M(0, 0) = 10;
	Eigen::Vector3d v;
	v = M * v;
	M = (M * M).transpose();
	std::cout << M.inverse() << std::endl;*/

	ros::init(argc, argv, "ekf");
	ros::NodeHandle n;
	/*ros::Subscriber encoder_sub = n.subscribe("/encoder", 1, encoderCallback);
	  ros::Subscriber sub = n.subscribe("/imu", 1, encoderCallback);
	  typedef message_filters::sync_policies::ApproximateTime MySyncPolicy;
	*/
	message_filters::Subscriber<kurt_msgs::Encoder> encoder_sub(n, "/encoder", 1);
	message_filters::Subscriber<sensor_msgs::Imu> sub(n, "/imu", 1);
	typedef message_filters::sync_policies::ApproximateTime<kurt_msgs::Encoder, sensor_msgs::Imu> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), encoder_sub, sub);
	sync.registerCallback(boost::bind(&encoderCallback, _1, _2));
	//exit(0);

	ros::spin();

	return 0;
}
