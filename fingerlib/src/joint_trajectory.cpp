#include "fingerlib/joint_trajectory.hpp"
#include <armadillo>
#include <cmath>

JointTrajectory::JointTrajectory(
  const Transformer & transforms, int sampling_rate,
  double ground_height)
: _transforms(transforms),
  _sampling_rate(sampling_rate),
  _ground_height(ground_height)
{
}

std::vector<arma::vec> JointTrajectory::generate_sinusoid(
  int joint, double amp, double freq,
  double v_shift)
{
  int N = std::ceil((double)_sampling_rate / freq);

  std::vector<arma::vec> q_motor_list;
  q_motor_list.reserve(N);

  for(double t = 0; t < 1.0 / freq; t += 1.0 / _sampling_rate) {

    double sine_value = amp * std::sin(2 * M_PI * freq * t) + v_shift;

    arma::vec q_joint(3, arma::fill::zeros);

    if(joint >= 0 && joint <= 2) {
      q_joint(joint) = sine_value;
    }

    q_motor_list.push_back(_transforms.joint_to_motor(q_joint));
  }

  q_motor_list.pop_back();  // Remove the last point to ensure we don't repeat the first point of the next cycle

    //std::cout << "Generated " << q_motor_list.size() << " motor positions." << std::endl;

  return q_motor_list;
}

std::vector<arma::vec> JointTrajectory::generate_chirp(
  int joint, double amp, double freq_1, double freq_2, double time, double v_shift)
{
  int N = std::ceil((double)_sampling_rate * time);

  std::vector<arma::vec> q_motor_list;
  q_motor_list.reserve(N);

  double c = (freq_2 - freq_1) / time;  // Chirp rate

  for(double t = 0; t < time; t += 1.0 / _sampling_rate) {

    double sine_value = amp * std::sin(2 * M_PI * (freq_1 * t + c * t * t / 2)) + v_shift;

    arma::vec q_joint(3, arma::fill::zeros);

    if(joint >= 0 && joint <= 2) {
      q_joint(joint) = sine_value;
    }

    q_motor_list.push_back(_transforms.joint_to_motor(q_joint));
  }

    //std::cout << "Generated " << q_motor_list.size() << " motor positions." << std::endl;

  return q_motor_list;
}


std::vector<arma::vec> JointTrajectory::generate_step(
  std::vector<arma::vec> waypoints,
  double freq)
{
  int N = std::ceil((double)_sampling_rate / freq);

  int total_waypoints = waypoints.size();

  std::vector<arma::vec> q_motor_list;

  q_motor_list.reserve(N);

  for (int i = 0; i < N; i++) {
    int waypoint_idx = (i * total_waypoints) / N;
    q_motor_list.push_back(_transforms.joint_to_motor(waypoints.at(waypoint_idx)));
  }

  return q_motor_list;
}

std::vector<arma::vec> JointTrajectory::generate_linear(
  std::vector<arma::vec> waypoints,
  double v_max, double a_max)
{

  std::vector<arma::vec> trajectory;

  for(unsigned int i = 1; i < waypoints.size(); i++) {
    auto start = waypoints.at(i - 1);
    auto end = waypoints.at(i);

    double s = arma::norm(end - start);
    double Tf = s <
      (v_max * v_max) / a_max ? 2.0 * std::sqrt(s / a_max) : s / v_max + v_max / a_max;

    int N = std::ceil(Tf * (double)_sampling_rate);

    std::vector<arma::vec> q_motor_list;
    q_motor_list.reserve(N);

    for(double t = 0; t < Tf; t += 1.0 / _sampling_rate) {

      double st = TrapezoidalTimeScaling(Tf, t, v_max, a_max, s);
      auto q_joint = start * (1 - st) + end * st;

      if(_transforms.joint_to_end_effector(q_joint)(2, 3) < _ground_height) {
        std::cout << _transforms.joint_to_end_effector(q_joint) << std::endl;
        throw std::runtime_error("Joint Trajectory goes into the ground");
      }

      q_motor_list.push_back(_transforms.joint_to_motor(q_joint));
    }
    // q_motor_list.push_back(_transforms.joint_to_motor(end));

    trajectory.insert(trajectory.end(), q_motor_list.begin(), q_motor_list.end());
  }

  return trajectory;
}

std::vector<arma::vec> JointTrajectory::generate_cartesian(
  std::vector<arma::vec> waypoints,
  double v_max, double a_max)
{

  std::vector<arma::vec> temp_waypoints;

  for(unsigned int i = 0; i < waypoints.size(); i++) {
    temp_waypoints.push_back(_transforms.end_effector_to_joint(waypoints[i]));
  }

  std::vector<arma::vec> trajectory = generate_linear(temp_waypoints, v_max, a_max);

  return trajectory;
}

std::vector<arma::vec> JointTrajectory::generate_force_step(
  const arma::vec & q_joint,
  const arma::vec & force_low,
  const arma::vec & force_high,
  double freq)
{

  int N = std::ceil((double)_sampling_rate / freq);
  auto J = _transforms.get_jacobian_body(q_joint);

  // std::cout << "Jacobian at q_joint: " << std::endl << J << std::endl;

  std::vector<arma::vec> t_motor_list;
  t_motor_list.reserve(N);

  for(double t = 0; t < 1.0 / freq; t += 1.0 / _sampling_rate) {

    arma::vec force_value = (std::sin(2 * M_PI * freq * t) >= 0) ? force_high : force_low;

    arma::vec wrench (6, arma::fill::zeros);
    wrench.tail(3) = force_value;

    auto J_t = J.t();

    auto t_joint = J_t * wrench;  // Convert force to joint torques

    auto t_motor = _transforms.joint_to_motor_torque(t_joint);
    t_motor(0) /= 3.5;
    // t_motor(0) *= 0.0;

    t_motor_list.push_back(t_motor);

    // std::cout << "force_value: " << force_value.t() << std::endl;
    // std::cout << "t_joint: " << t_joint.t() << std::endl;
    // std::cout << "t_motor: " << t_motor_list.back().t() << std::endl;
  }

  return t_motor_list;
}

std::vector<arma::vec> JointTrajectory::generate_chirp_velocity(
  int joint, double amp, double freq_1, double freq_2, double time, double start_pos)
{
  int N = std::ceil((double)_sampling_rate * time);

  std::vector<arma::vec> q_motor_list;
  q_motor_list.reserve(N);

  double c = (freq_2 - freq_1) / time;  // Chirp rate
  double joint_angle = start_pos;

  for(double t = 0; t < time; t += 1.0 / _sampling_rate) {

    double sine_value = amp * std::sin(2 * M_PI * (freq_1 * t + c * t * t / 2));

    joint_angle += sine_value * (1.0 / _sampling_rate);  // integrate velocity to get position

    arma::vec q_joint(3, arma::fill::zeros);
    arma::vec q_pos_test(3, arma::fill::zeros);

    if(joint >= 0 && joint <= 2) {
      q_joint(joint) = sine_value;
      q_pos_test(joint) = joint_angle;
    }

    // test pos
    _transforms.joint_to_motor(q_pos_test);

    q_motor_list.push_back(_transforms.joint_to_motor(q_joint, false));
  }

    //std::cout << "Generated " << q_motor_list.size() << " motor positions." << std::endl;

  return q_motor_list;
}

double JointTrajectory::TrapezoidalTimeScaling(
  const double Tf, const double t,
  const double v_max, const double a_max, const double s)
{
  const bool is_triangular = ((v_max * v_max) / a_max > s);

  const double ta = is_triangular ? Tf / 2.0 :
    v_max / a_max;

  double st = 0.0;

  if (t <= ta) {
    st = 0.5 * a_max * t * t;
  } else if (t <= Tf - ta) {
    st = 0.5 * a_max * ta * ta + v_max * (t - ta);
  } else {
    const double dt = Tf - t;
    st = s - 0.5 * a_max * dt * dt;
  }

  return st / s;  // normalize to [0, 1]
}
