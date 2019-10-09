#pragma once

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

#include "mav_forces_and_moments.h"

namespace rosflight_sil
{

class Multirotor : public MAVForcesAndMoments {
private:
    ros::NodeHandle* nh_;
    Eigen::Vector3d wind_;

    double prev_time_;

    struct Rotor{
      double max;
      std::vector<double> F_poly;
      std::vector<double> T_poly;
      double tau_up; // time constants for response
      double tau_down;
    };

    struct Motor{
      Rotor rotor;
      Eigen::Vector3d position;
      Eigen::Vector3d normal;
      int direction; // 1 for CW -1 for CCW
    };

    int num_rotors_;
    std::vector<Motor> motors_;

    double linear_mu_;
    double angular_mu_;
    std::vector<double> ground_effect_;

    double mass_;

    // Container for an Actuator
    struct Actuator{
      double max;
      double tau_up;
      double tau_down;
    };

    // Struct of Actuators
    // This organizes the physical limitations of the abstract torques and Force
    struct Actuators{
      Actuator l;
      Actuator m;
      Actuator n;
      Actuator F;
    } actuators_;

    Eigen::MatrixXd rotor_position_;
    Eigen::MatrixXd rotor_plane_normal_;
    Eigen::VectorXd rotor_rotation_direction_;

    Eigen::MatrixXd force_allocation_matrix_;
    Eigen::MatrixXd torque_allocation_matrix_;
    Eigen::VectorXd desired_forces_;
    Eigen::VectorXd desired_torques_;
    Eigen::VectorXd actual_forces_;
    Eigen::VectorXd actual_torques_;

public:
    Multirotor(ros::NodeHandle* nh);
    ~Multirotor();

    Eigen::Matrix<double, 6, 1> updateForcesAndTorques(Current_State x, const int act_cmds[]);
    void set_wind(Eigen::Vector3d wind);
};

} // namespace rosflight_sil
