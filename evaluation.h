#ifndef _EVALUATION_
#define _EVALUATION_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <armadillo>

using namespace arma;

class Evaluation {

  public:
    Evaluation(fmat mat_init) {
      position_ekf = mat_init;
      orientation_ekf = mat_init;
      lin_vel_ekf = mat_init;
      ang_vel_ekf = mat_init;

      position_vicon = mat_init;
      orientation_vicon = mat_init;
      lin_vel_vicon = mat_init;
      ang_vel_vicon = mat_init;

      actual_data = mat_init;
      error_matrix = mat_init;

      index_ekf = 0;
      index_vicon = 0;
    };

    void ekfCallback(const nav_msgs::Odometry::ConstPtr& ekf);
    void viconCallback(const nav_msgs::Odometry::ConstPtr& vicon);
    void error_calculation();

  private:
    fmat position_ekf;
    fmat orientation_ekf;
    fmat lin_vel_ekf;
    fmat ang_vel_ekf;

    fmat position_vicon;
    fmat orientation_vicon;
    fmat lin_vel_vicon;
    fmat ang_vel_vicon;

    fmat actual_data;
    fmat error_matrix;

    int index_ekf;
    int index_vicon;

};

#endif
