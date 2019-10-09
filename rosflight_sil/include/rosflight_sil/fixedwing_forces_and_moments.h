#pragma once

#include "mav_forces_and_moments.h"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

namespace rosflight_sil
{

class Fixedwing : public MAVForcesAndMoments
{
private:
    ros::NodeHandle* nh_;

    // physical parameters
    double mass_;
    double Jx_;
    double Jy_;
    double Jz_;
    double Jxz_;
    double rho_;

    // aerodynamic coefficients
    struct WingCoeff{
      double S;
      double b;
      double c;
      double M;
      double epsilon;
      double alpha0;
    } wing_;

    // Propeller Coefficients
    struct PropCoeff{
      double k_motor;
      double k_T_P;
      double k_Omega;
      double e;
      double S;
      double C;
    } prop_;

    // Lift Coefficients
    struct LiftCoeff{
      double O;
      double alpha;
      double beta;
      double p;
      double q;
      double r;
      double delta_a;
      double delta_e;
      double delta_r;
    };

    LiftCoeff CL_;
    LiftCoeff CD_;
    LiftCoeff Cm_;
    LiftCoeff CY_;
    LiftCoeff Cell_;
    LiftCoeff Cn_;

    // not constants
    // actuators
    struct Actuators{
      double e;
      double a;
      double r;
      double t;
    } delta_;

    // wind
    Eigen::Vector3d wind_;

public:
    Fixedwing(ros::NodeHandle* nh);
    ~Fixedwing();

    Eigen::Matrix<double, 6, 1> updateForcesAndTorques(Current_State x, const int act_cmds[]);
    void set_wind(Eigen::Vector3d wind);
};

} // namespace rosflight_sil
