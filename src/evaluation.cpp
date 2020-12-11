#include "evaluation.h"

void Evaluation::ekfCallback(const nav_msgs::Odometry::ConstPtr& ekf)
{
  //Save pose and velocity data from ekf estimation and add it to the ekf data collector matrix
  actual_data(0,1) = ekf->pose.pose.position.x;
  actual_data(0,2) = ekf->pose.pose.position.y;
  actual_data(0,3) = ekf->pose.pose.position.z;
  actual_data(0,4) = 0;
  actual_data(0,5) = index_ekf;
  position_ekf = join_vert(position_ekf, actual_data);

  actual_data(0,1) = ekf->pose.pose.orientation.x;
  actual_data(0,2) = ekf->pose.pose.orientation.y;
  actual_data(0,3) = ekf->pose.pose.orientation.z;
  actual_data(0,4) = ekf->pose.pose.orientation.w;
  actual_data(0,5) = index_ekf;
  orientation_ekf = join_vert(orientation_ekf, actual_data);


  actual_data(0,1) = ekf->twist.twist.linear.x;
  actual_data(0,2) = ekf->twist.twist.linear.y;
  actual_data(0,3) = ekf->twist.twist.linear.z;
  actual_data(0,4) = 0;
  actual_data(0,5) = index_ekf;
  lin_vel_ekf = join_vert(lin_vel_ekf, actual_data);

  actual_data(0,1) = ekf->twist.twist.angular.x;
  actual_data(0,2) = ekf->twist.twist.angular.y;
  actual_data(0,3) = ekf->twist.twist.angular.z;
  actual_data(0,4) = 0;
  actual_data(0,5) = index_ekf;
  ang_vel_ekf = join_vert(ang_vel_ekf, actual_data);

  //Update ekf index
  index_ekf++;
}

void Evaluation::viconCallback(const nav_msgs::Odometry::ConstPtr& vicon)
{
  if (index_ekf == index_vicon+1) {
    //Save pose and velocity data from vicon estimation and add it to the vicon data collector matrix
    actual_data(0,1) = vicon->pose.pose.position.x;
    actual_data(0,2) = vicon->pose.pose.position.y;
    actual_data(0,3) = vicon->pose.pose.position.z;
    actual_data(0,4) = 0;
    actual_data(0,5) = index_vicon;
    position_vicon = join_vert(position_vicon, actual_data);

    actual_data(0,1) = vicon->pose.pose.orientation.x;
    actual_data(0,2) = vicon->pose.pose.orientation.y;
    actual_data(0,3) = vicon->pose.pose.orientation.z;
    actual_data(0,4) = vicon->pose.pose.orientation.w;
    actual_data(0,5) = index_vicon;
    orientation_vicon = join_vert(orientation_vicon, actual_data);


    actual_data(0,1) = vicon->twist.twist.linear.x;
    actual_data(0,2) = vicon->twist.twist.linear.y;
    actual_data(0,3) = vicon->twist.twist.linear.z;
    actual_data(0,4) = 0;
    actual_data(0,5) = index_vicon;
    lin_vel_vicon = join_vert(lin_vel_vicon, actual_data);

    actual_data(0,1) = vicon->twist.twist.angular.x;
    actual_data(0,2) = vicon->twist.twist.angular.y;
    actual_data(0,3) = vicon->twist.twist.angular.z;
    actual_data(0,4) = 0;
    actual_data(0,5) = index_vicon;
    ang_vel_vicon = join_vert(ang_vel_vicon, actual_data);

    //Update vicon index
    index_vicon++;
  }
  else if (index_ekf - index_vicon >= 2) {
    ROS_ERROR("Evaluation_node: vicon data updates are missing!");
  }
  else if (index_vicon > index_ekf) {
    ROS_ERROR("Evalaution_node: index_vicon greater than index_ekf!");
  }
}

void Evaluation::error_calculation()
{
  // This function computes the mean square error of position, orientation, linear and angular velocity.
  int N = index_ekf+1; // number of saved data
  // Calculate position error
  if (lin_vel_ekf.n_rows == lin_vel_vicon.n_rows) {
    error_matrix = sum((lin_vel_ekf-lin_vel_vicon)%(lin_vel_ekf-lin_vel_vicon))/N;
    bool ok = error_matrix.save("home/menichea/catkin_ws/src/rewrite_imu_pkg/evaluation_result/lin_vel_error.txt", arma_ascii);
    if (ok) {
      ROS_ERROR("error saved");
    }
    else {
      ROS_ERROR("error not saved");
    }
  }
  else {
    ROS_ERROR("Evaluation_node: lin_vel_ekf and lin_vel_vicon have different dimensions!");
  }
}
