#ifndef JOINT_TRAJECTORY_HPP
#define JOINT_TRAJECTORY_HPP

#include "armadillo"
#include "fingerlib/transformer.hpp"
#include <vector>  // needed for std::vector

/// \brief Basic sinusoid generator for testing 1 joint at a time
class JointTrajectory
{
public:
    /// \brief Initializer for sinusoid generator
    /// \param transforms - Transformer object for converting between joint and motor space
    /// \param sampling_rate - The rate at which to sample the sinusoid (in Hz)
  JointTrajectory(const Transformer & transforms, int sampling_rate, double ground_height);

    /// \brief Generate a sinusoidal trajectory for a specified joint + sine parameters
    /// \param joint - The index of the joint to generate the sinusoid for (0, 1, or 2)
    /// \param amp - The amplitude of the sinusoid (in radians)
    /// \param freq - The frequency of the sinusoid (in Hz)
    /// \param v_shift - The vertical shift of the sinusoid (in radians)
    /// \return A vector of motor positions corresponding to the generated sinusoidal trajectory
  std::vector<arma::vec> generate_sinusoid(int joint, double amp, double freq, double v_shift);

    /// \brief Generate a chirp trajectory for a specified joint + parameters
    /// \param joint - The index of the joint to generate the chirp for (0, 1, or 2)
    /// \param amp - The amplitude of the chirp (in radians)
    /// \param freq_1 - The initial frequency of the chirp (in Hz)
    /// \param freq_2 - The final frequency of the chirp (in Hz)
    /// \param time - The duration of the chirp (in seconds)
    /// \param v_shift - The vertical shift of the chirp (in radians)
    /// \return A vector of motor positions corresponding to the generated chirp trajectory
  std::vector<arma::vec> generate_chirp(
    int joint, double amp, double freq_1, double freq_2,
    double time, double v_shift);

    /// \brief Generate a step trajectory for a specified joint + parameters
    /// \param waypoints - list of joint waypoints
    /// \param freq - The frequency of the step (in Hz) to change waypoint
    /// \return A vector of motor positions corresponding to the generated step trajectory
  std::vector<arma::vec> generate_step(std::vector<arma::vec> waypoints, double freq);

    /// \brief Generate a linear trajectory with trapezoidal time scaling
    /// \param waypoints - list of joint waypoints
    /// \param v_max - max speed during operation
    /// \param a_max - max acceleration during operation
    /// \param ground_height - relative height of the ground plane (should be negative)
    /// \return A vector of motor positions corresponding to the generated linear trajectory
  std::vector<arma::vec> generate_linear(
    std::vector<arma::vec> waypoints, double v_max,
    double a_max);

    /// \brief Generate a trajectory through joint space linear interpolation of cartesian waypoints
    /// \param waypoints - list of cartesian waypoints
    /// \param v_max - maximum speed during operation
    /// \param a_max - maximum acceleration during operation
    /// \return a vector of motor positions for the entire motion
  std::vector<arma::vec> generate_cartesian(
    std::vector<arma::vec> waypoints, double v_max,
    double a_max);

    /// \brief Generate a trajectory that applies a step force at the end-effector
    /// \param q_joint - the joint position at which to apply the force step (x,y,z)
    /// \param force_low - the lower bound of the force step (in Newtons)
    /// \param force_high - the upper bound of the force step (in Newtons)
    /// \param freq - the frequency of the force step (in Hz)
    /// \return a vector of motor positions corresponding to the generated force step trajectory
  std::vector<arma::vec> generate_force_step(
    const arma::vec & q_joint,
    const arma::vec & force_low,
    const arma::vec & force_high,
    double freq);

    /// \brief Generate a chirp trajectory for a specified joint + parameters
    /// \param joint - The index of the joint to generate the chirp for (0, 1, or 2)
    /// \param amp - The amplitude of the chirp (in radians)
    /// \param freq_1 - The initial frequency of the chirp (in Hz)
    /// \param freq_2 - The final frequency of the chirp (in Hz)
    /// \param time - The duration of the chirp (in seconds)
    /// \param start_pos - The starting position of the chirp (in radians)
    /// \return A vector of motor positions corresponding to the generated chirp trajectory
  std::vector<arma::vec> generate_chirp_velocity(
    int joint, double amp, double freq_1,
    double freq_2, double time, double start_pos);

private:
    /// \brief The transformer object for converting between joint and motor space
  Transformer _transforms;

    /// \brief The sampling rate for the sinusoid (in Hz)
  int _sampling_rate;

    /// \brief height of the ground relative to fully extended pose (should be negative)
  double _ground_height;

    /// @brief Trapezoidal time scaling returning normalized position s(t) in [0, 1].
    ///
    /// Matches a trapezoidal velocity profile with peak velocity v_max and
    /// acceleration a_max. Automatically falls back to a triangular profile
    /// if the distance (normalized to 1) is too short to reach v_max.
    ///
    /// @param Tf     Total motion time
    /// @param t      Current time in [0, Tf]
    /// @param v_max  Maximum velocity (in normalized distance / unscaled time units)
    /// @param a_max  Maximum acceleration (in normalized distance / unscaled time^2 units)
    /// @param s      total motion length
    /// @return       Normalized position s in [0, 1]
  double TrapezoidalTimeScaling(
    const double Tf, const double t,
    const double v_max, const double a_max, const double s);

};

#endif // JOINT_TRAJECTORY
