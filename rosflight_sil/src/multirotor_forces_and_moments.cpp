#include <rosflight_sil/multirotor_forces_and_moments.h>

namespace rosflight_sil
{

Multirotor::Multirotor(ros::NodeHandle *nh)
{
  nh_ = nh;

  // Pull Parameters off of rosparam server
  num_rotors_ = 0;

  if (nh_->hasParam("ground_effect"))
  {
    nh_->getParam("ground_effect", ground_effect_);
  }
  else
  {
    ground_effect_.push_back(-55.3516);
    ground_effect_.push_back(181.8265);
    ground_effect_.push_back(-203.9874);
    ground_effect_.push_back(85.3735);
    ground_effect_.push_back(-7.6619);
  }
  mass_ = nh_->param<double>("uav_mass", 2.0);
//  linear_mu_ = nh_->param<double>("linear_mu", 0.05);
//  angular_mu_ = nh_->param<double>("angular_mu", 0.0005);
  num_rotors_ = nh_->param<int>("num_rotors", 4);

  std::vector<double> rotor_positions; //(3 * num_rotors_);
  std::vector<double> rotor_vector_normal; //(3 * num_rotors_);
  std::vector<int> rotor_rotation_directions; //(num_rotors_);

  // For now, just assume all rotors are the same
  Rotor rotor;

  if (nh_->hasParam("rotor_positions"))
  {
    nh_->getParam("rotor_positions", rotor_positions);
  }
  else
  {
    rotor_positions.push_back(0.1926);
    rotor_positions.push_back(0.230);
    rotor_positions.push_back(-0.0762);
    rotor_positions.push_back(-0.1907);
    rotor_positions.push_back(0.205);
    rotor_positions.push_back(-0.0762);
    rotor_positions.push_back(-0.1907);
    rotor_positions.push_back(-0.205);
    rotor_positions.push_back(-0.0762);
    rotor_positions.push_back(0.1926);
    rotor_positions.push_back(-0.230);
    rotor_positions.push_back(-0.0762);
  }
  if (nh_->hasParam("rotor_vector_normal"))
  {
    nh_->getParam("rotor_vector_normal", rotor_vector_normal);
  }
  else
  {
    rotor_vector_normal.push_back(-0.02674078);
    rotor_vector_normal.push_back(0.0223925);
    rotor_vector_normal.push_back(-0.99939157);
    rotor_vector_normal.push_back(0.02553726);
    rotor_vector_normal.push_back(0.02375588);
    rotor_vector_normal.push_back(-0.99939157);
    rotor_vector_normal.push_back(0.02553726);
    rotor_vector_normal.push_back(-0.02375588);
    rotor_vector_normal.push_back(-0.99939157);
    rotor_vector_normal.push_back(-0.02674078);
    rotor_vector_normal.push_back(-0.0223925);
    rotor_vector_normal.push_back(-0.99939157);
  }
  if (nh_->hasParam("rotor_rotation_directions"))
  {
    nh_->getParam("rotor_rotation_directions", rotor_rotation_directions);
  }
  else
  {
    rotor_rotation_directions.push_back(-1);
    rotor_rotation_directions.push_back(1);
    rotor_rotation_directions.push_back(-1);
    rotor_rotation_directions.push_back(1);
  }
  rotor.max = nh_->param<double>("rotor_max_thrust", 14.961);
  if (nh_->hasParam("rotor_F"))
  {
    nh_->getParam("rotor_F", rotor.F_poly);
  }
  else
  {
    rotor.F_poly.push_back(0.000015);
    rotor.F_poly.push_back(-0.024451);
    rotor.F_poly.push_back(9.00225);
  }
  if (nh_->hasParam("rotor_T"))
  {
    nh_->getParam("rotor_T", rotor.T_poly);
  }
  else
  {
    rotor.T_poly.push_back(0.000000222);
    rotor.T_poly.push_back(-0.000351);
    rotor.T_poly.push_back(0.12531);
  }
  rotor.tau_up = nh_->param<double>("rotor_tau_up", 0.2164);
  rotor.tau_down = nh_->param<double>("rotor_tau_down", 0.1644);

  /* Load Rotor Configuration */
  motors_.resize(num_rotors_);

  force_allocation_matrix_.resize(4,num_rotors_);
  torque_allocation_matrix_.resize(4,num_rotors_);
  for(int i = 0; i < num_rotors_; i++)
  {
    motors_[i].rotor = rotor;
    motors_[i].position.resize(3);
    motors_[i].normal.resize(3);
    for (int j = 0; j < 3; j++)
    {
      motors_[i].position(j) = rotor_positions[3*i + j];
      motors_[i].normal(j) = rotor_vector_normal[3*i + j];
    }
    motors_[i].normal.normalize();
    motors_[i].direction = rotor_rotation_directions[i];

    Eigen::Vector3d moment_from_thrust = motors_[i].position.cross(motors_[i].normal);
    Eigen::Vector3d moment_from_torque = motors_[i].direction * motors_[i].normal;

    // build allocation_matrices
    force_allocation_matrix_(0,i) = moment_from_thrust(0); // l
    force_allocation_matrix_(1,i) = moment_from_thrust(1); // m
    force_allocation_matrix_(2,i) = moment_from_thrust(2); // n
    force_allocation_matrix_(3,i) = motors_[i].normal(2); // F

    torque_allocation_matrix_(0,i) = moment_from_torque(0); // l
    torque_allocation_matrix_(1,i) = moment_from_torque(1); // m
    torque_allocation_matrix_(2,i) = moment_from_torque(2); // n
    torque_allocation_matrix_(3,i) = 0.0; // F
  }

  ROS_INFO_STREAM("allocation matrices:\nFORCE \n" << force_allocation_matrix_ << "\nTORQUE\n" << torque_allocation_matrix_ << "\n");

  // Initialize size of dynamic force and torque matrices
  desired_forces_.resize(num_rotors_);
  desired_torques_.resize(num_rotors_);
  actual_forces_.resize(num_rotors_);
  actual_torques_.resize(num_rotors_);

  for (int i = 0; i < num_rotors_; i++)
  {
    desired_forces_(i)=0.0;
    desired_torques_(i)=0.0;
    actual_forces_(i)=0.0;
    actual_torques_(i)=0.0;
  }

  wind_ = Eigen::Vector3d::Zero();
  prev_time_ = -1;
}

Eigen::Matrix<double, 6, 1> Multirotor::updateForcesAndTorques(Current_State x, const int act_cmds[])
{
  if (prev_time_ < 0)
  {
    prev_time_ = x.t;
    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  double dt = x.t - prev_time_;
  double pd = x.pos[2];

  // Get airspeed vector for drag force calculation (rotate wind into body frame and add to inertial velocity)
  Eigen::Vector3d Va = x.vel; // + x.rot.inverse()*wind_;

//  std::cout << "mixed sim: " << act_cmds[0] << " " << act_cmds[1] << " " << act_cmds[2] << " " << act_cmds[3] << std::endl; // ----

  // Calculate Forces
  for (int i = 0; i<num_rotors_; i++)
  {
    // First, figure out the desired force output from passing the signal into the quadratic approximation
    double signal = act_cmds[i];
    desired_forces_(i,0) = motors_[i].rotor.F_poly[0]*signal*signal + motors_[i].rotor.F_poly[1]*signal + motors_[i].rotor.F_poly[2];
    desired_torques_(i,0) = motors_[i].rotor.T_poly[0]*signal*signal + motors_[i].rotor.T_poly[1]*signal + motors_[i].rotor.T_poly[2];

    // Then, Calculate Actual force and torque for each rotor using first-order dynamics
    double tau = (desired_forces_(i,0) > actual_forces_(i,0)) ? motors_[i].rotor.tau_up : motors_[i].rotor.tau_down;
    double alpha = dt/(tau + dt);
    actual_forces_(i,0) = sat((1-alpha)*actual_forces_(i) + alpha*desired_forces_(i), motors_[i].rotor.max, 0.0);
    actual_torques_(i,0) = sat((1-alpha)*actual_torques_(i) + alpha*desired_torques_(i), motors_[i].rotor.max, 0.0);
  }

  // Use the allocation matrix to calculate the body-fixed force and torques
  Eigen::Vector4d output_forces = force_allocation_matrix_*actual_forces_;
  Eigen::Vector4d output_torques = torque_allocation_matrix_*actual_torques_;
  Eigen::Vector4d output_forces_and_torques = output_forces + output_torques;

  // Calculate Ground Effect
  double z = -pd;
  double ground_effect = max(ground_effect_[0]*z*z*z*z + ground_effect_[1]*z*z*z + ground_effect_[2]*z*z + ground_effect_[3]*z + ground_effect_[4], 0);

  Eigen::Matrix<double, 6,1> forces;
  // Apply other forces (drag) <- follows "Quadrotors and Accelerometers - State Estimation With an Improved Dynamic Model"
  // By Rob Leishman et al.
  forces.block<3,1>(0,0).setZero(); // -linear_mu_ * Va;
  forces.block<3,1>(3,0) = output_forces_and_torques.block<3,1>(0,0); //-angular_mu_ * x.omega + output_forces_and_torques.block<3,1>(0,0);

  // Apply ground effect and thrust
  forces(2) += output_forces_and_torques(3); // - ground_effect;

  return forces;
}

void Multirotor::set_wind(Eigen::Vector3d wind)
{
  wind_ = wind;
}

}
