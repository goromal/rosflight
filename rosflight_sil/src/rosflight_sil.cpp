#pragma GCC diagnostic ignored "-Wwrite-strings"

#include <sstream>
#include <stdint.h>
#include <stdio.h>

#include <eigen3/Eigen/Core>

#include "rosflight_sil/rosflight_sil.h"
#include "rosflight_sil/sil_board.h"

namespace rosflight_sil {

ROSflightSIL::ROSflightSIL() : nh_(nullptr),
    nh_private_(nullptr), comm_(board_), firmware_(board_, comm_), truth_received_(false),
    start_time_sec_(0.0)
{
  memset(&motor_forces_msg_, 0.0, sizeof(motor_forces_msg_));
}

ROSflightSIL::~ROSflightSIL()
{
  if (nh_)
  {
    nh_->shutdown();
    delete nh_;
  }
  if (nh_private_)
  {
    nh_private_->shutdown();
    delete nh_private_;
  }
}

void ROSflightSIL::Initialize()
{
  nh_ = new ros::NodeHandle();
  nh_private_ = new ros::NodeHandle("~");
  timer_ = nh_->createTimer(ros::Duration(ros::Rate(1000)), &ROSflightSIL::OnUpdate, this);
  mav_type_ = nh_private_->param<std::string>("mav_type", "multirotor");

  if(mav_type_ == "multirotor")
    mav_dynamics_ = new Multirotor(nh_);
  else if(mav_type_ == "fixedwing")
    mav_dynamics_ = new Fixedwing(nh_);

  board_.ros_setup(nh_, nh_private_, mav_type_);

  truth_NED_sub_ = nh_->subscribe("uav_truth_NED", 1, &ROSflightSIL::truthCallback, this);
  motor_forces_pub_ = nh_->advertise<geometry_msgs::Wrench>("uav_motor_wrench", 1);
}

void ROSflightSIL::truthCallback(const rosflight_msgs::ROSflightSimState &msg)
{
  board_.set_truth(msg);
  if (!truth_received_)
  {
    firmware_.init();
    start_time_sec_ = msg.header.stamp.toSec();
    truth_received_ = true;
  }
  current_state_.pos = Vector3d(msg.pos.x, msg.pos.y, msg.pos.z);
  current_state_.rot = transforms::Quatd(
              (Vector4d() << msg.att.w, msg.att.x, msg.att.y, msg.att.z).finished()).R();
  current_state_.vel = Vector3d(msg.vel.x, msg.vel.y, msg.vel.z);
  current_state_.omega = Vector3d(msg.w.x, msg.w.y, msg.w.z);
}

void ROSflightSIL::OnUpdate(const ros::TimerEvent&)
{
  if (truth_received_) // Physics are turned on, so ready to start simulating ROSflight
  {
    // We run twice so that that functions that take place when we don't have new IMU data get run
    firmware_.run();
    firmware_.run();

//     std::cout << "SIL OUTPUTS: " << board_.get_outputs()[0] << " "
//                                  << board_.get_outputs()[1] << " "
//                                  << board_.get_outputs()[2] << " "
//                                  << board_.get_outputs()[3] << std::endl;
    current_state_.t = ros::Time::now().toSec();
//    std::cout << "SIL STATE: (" << current_state_.t << ")\t" /*<< current_state_.pos.transpose() << "\t"*/
//              << current_state_.rot(0,0) << "\t" << current_state_.rot(0,1) << "\t" << current_state_.rot(0,2) << "\t"
//              << current_state_.rot(1,0) << "\t" << current_state_.rot(1,1) << "\t" << current_state_.rot(1,2) << "\t"
//              << current_state_.rot(2,0) << "\t" << current_state_.rot(2,1) << "\t" << current_state_.rot(2,2) << "\t"
//              << std::endl;
//    std::cout << "SIL STATE: (" << current_state_.t << ")\t" << current_state_.vel.transpose() << "\t"
//              << current_state_.omega.transpose() << std::endl;
    motor_forces_ = mav_dynamics_->updateForcesAndTorques(current_state_, board_.get_outputs());
//    std::cout << "SIL F: " << (motor_forces_.block<3,1>(0,0)).transpose() << std::endl; // ----
//   std::cout << "SIL T: " << (motor_forces_.block<3,1>(3,0)).transpose() << std::endl << std::endl; // ----
    motor_forces_msg_.force.x = motor_forces_(0);  // should be 0
    motor_forces_msg_.force.y = motor_forces_(1);  // should be 0
    motor_forces_msg_.force.z = motor_forces_(2);  // thrust
    motor_forces_msg_.torque.x = motor_forces_(3); // tau_x (body-frame)
    motor_forces_msg_.torque.y = motor_forces_(4); // tau_y (body-frame)
    motor_forces_msg_.torque.z = motor_forces_(5); // tau_z (body-frame)
    motor_forces_pub_.publish(motor_forces_msg_);
  }
}

} // end namespace rosflight_sil
