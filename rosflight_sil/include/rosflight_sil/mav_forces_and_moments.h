#pragma once

#include <eigen3/Eigen/Core>

namespace rosflight_sil
{

class MAVForcesAndMoments {
protected:
    double sat(double x, double max, double min)
    {
      if(x > max)
        return max;
      else if(x < min)
        return min;
      else
        return x;
    }

    double max(double x, double y)
    {
      return (x > y) ? x : y;
    }

public:

    struct Current_State{
        Eigen::Vector3d pos; // Position of MAV in NED wrt initial position
        Eigen::Matrix3d rot; // Rotation of MAV in NED wrt initial position
        Eigen::Vector3d vel; // Body-fixed velocity of MAV wrt initial position (NED)
        Eigen::Vector3d omega; // Body-fixed angular velocity of MAV (NED)
        double t; // current time
    };

    virtual Eigen::Matrix<double, 6, 1> updateForcesAndTorques(Current_State x, const int act_cmds[]) = 0;
    virtual void set_wind(Eigen::Vector3d wind) = 0;
};

} // namespace rosflight_sil
