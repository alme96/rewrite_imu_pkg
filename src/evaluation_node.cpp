#include "evaluation.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv, "evaluation_node");
  ros::NodeHandle nh;
  fmat mat_init(1,5, fill::zeros);
  Evaluation evaluation(mat_init);
  ros::Subscriber ekfsub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 100, &Evaluation::ekfCallback, &evaluation);
  ros::Subscriber viconsub = nh.subscribe<nav_msgs::Odometry>("/radar/vrpn_client/estimated_odometry", 100, &Evaluation::viconCallback, &evaluation);
  ros::spin();
  evaluation.error_calculation();

  return 0;
}
