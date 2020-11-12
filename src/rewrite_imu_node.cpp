#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "boost/bind.hpp"


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  sensor_msgs::Imu new_imu;
  //Update covariance matrices in order to pass the filter
  new_imu.angular_velocity_covariance[0] = 0.01;
  new_imu.linear_acceleration_covariance[0]= 0.01;
  new_imu.orientation_covariance[0] = -1;
  //Copy the measurements from the old imu to the new imu message
  new_imu.header = msg->header;
  new_imu.angular_velocity = msg->angular_velocity;
  new_imu.linear_acceleration = msg->linear_acceleration;

  pub.publish(new_imu);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "rewrite_imu");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu_ok", 100);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>("imu", 100, boost::bind(imuCallback, _1, pub));
  ros::spin();
}
