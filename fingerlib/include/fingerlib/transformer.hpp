#ifndef TRANSFORMS_HPP
#define TRANSFORMS_HPP

#include "armadillo"

// notes
// 0 is fully extended

/// \brief Class for handling transformations between joint space, motor space, and end-effector space for the robotic finger
class Transformer
{
public:
/// \brief Constructor for the Transformer class
/// \param Ra - The radius matrix for the finger joints
/// \param structure - The structure matrix for the finger joints (following Sairam's convention)
/// \param screw_axes - The screw axes for the finger joints (in space frame)
/// \param _M - The home configuration of the fingertip
/// \param four_bar_lengths - The lengths of the four bars in the finger's four-bar linkage
/// \param joint_min - The minimum angle for each joint
/// \param joint_max - The maximum angle for each joint
  Transformer(
    const arma::mat & Ra,
    const arma::mat & structure,
    const std::vector<arma::vec6> & screw_axes,
    const arma::mat44 & M,
    const std::vector<double> & four_bar_lengths,
    const arma::vec & joint_min,
    const arma::vec & joint_max
  );

/// \brief Convert joint angles to motor positions
/// \param q_joint The joint angles (in radians)
/// \return The motor positions corresponding to the given joint angles
  arma::vec joint_to_motor(const arma::vec & q_joint, bool enforce_limits = true);

/// \brief Convert motor positions to joint angles
/// \param q_motor The motor positions
/// \return The joint angles corresponding to the given motor positions
  arma::vec motor_to_joint(const arma::vec & q_motor, bool enforce_limits = true);

  /// @brief Convert joint torques to motor torques
  /// @param t_joint The joint torques
  /// @return The motor torques corresponding to the given joint torques
  arma::vec joint_to_motor_torque(const arma::vec & t_joint);

/// \brief Convert joint angles to end-effector position
/// \param q_joint The joint angles (in radians)
/// \return The end-effector transformation matrix corresponding to the given joint angles
  arma::mat44 joint_to_end_effector(const arma::vec & q_joint);

/// \brief Convert end-effector position to joint angles
/// \param q_end_effector The end-effector position (x,y,z no angle)
/// \param tolerance The allowable position error tolerance (in meters)
/// \return The joint angles corresponding to the given end-effector position
  arma::vec end_effector_to_joint(const arma::vec & q_end_effector, double tolerance = 1e-6);


/// \brief Get the Jacobian matrix in space frame (collapsed for pip/dip)
/// \param q_joint The joint angles (in radians)
/// \return The Jacobian matrix in space frame
  arma::mat get_jacobian_space(const arma::vec & q_joint);

/// \brief Get the Jacobian matrix in body frame (collapsed for pip/dip)
/// \param q_joint The joint angles (in radians)
/// \return The Jacobian matrix in body frame
  arma::mat get_jacobian_body(const arma::vec & q_joint);

/// \brief Calculate the 4-bar linkage ratios (dip angle and speed ratio) based on the pip angle
/// \param pip_angle The angle of the pip joint (in radians)
/// \param dip_angle Output parameter for the calculated dip angle (in radians)
/// \param speed_ratio Output parameter for the calculated speed ratio (dimensionless)
  void calculate_4bar_ratios(double pip_angle, double & dip_angle, double & speed_ratio);

private:
// constant things
/// \brief The radius matrix for the finger joints
  const arma::mat _Ra;
/// \brief The inverse of the radius matrix (precomputed for efficiency)
  const arma::mat _Ra_inv;

/// \brief The structure matrix for the finger joints (following Sairam's convention)
  const arma::mat _structure;
/// \brief The inverse of the structure matrix (precomputed for efficiency)
  const arma::mat _structure_inv;

/// \brief The screw axes for the finger joints (in space frame)
  const std::vector<arma::vec6> _screw_axes;
/// \brief The home configuration of the fingertip
  const arma::mat44 _M;
/// \brief  The lengths of the four bars in the finger's four-bar linkage (in meters)
  const std::vector<double> _4bar_lengths;

/// \brief minimum joint angle for each joint
  const arma::vec _joint_min;
/// \brief maximum joint angle for each joint
  const arma::vec _joint_max;


};

#endif
