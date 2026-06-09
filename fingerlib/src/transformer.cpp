#include "fingerlib/transformer.hpp"
#include "modern_robotics/velocity_kinematics_and_statics.hpp"
#include "modern_robotics/forward_kinematics.hpp"
#include "modern_robotics/rigid_body_motions.hpp"

Transformer::Transformer(
  const arma::mat & Ra,
  const arma::mat & structure,
  const std::vector<arma::vec6> & screw_axes,
  const arma::mat44 & M,
  const std::vector<double> & four_bar_lengths,
  const arma::vec & joint_min,
  const arma::vec & joint_max)
: _Ra(Ra), _Ra_inv(arma::pinv(_Ra)),
  _structure(structure), _structure_inv(arma::pinv(_structure)),
  _screw_axes(screw_axes), _M(M),
  _4bar_lengths(four_bar_lengths),
  _joint_min(joint_min), _joint_max(joint_max)
{
}

arma::vec Transformer::joint_to_motor(const arma::vec & q_joint, bool enforce_limits)
{
  if(enforce_limits) {
    bool inside =
      arma::all(q_joint >= _joint_min) &&
      arma::all(q_joint <= _joint_max);

    if(!inside) {
      throw std::runtime_error("Input outside of joint limits");
    }
  }

  arma::vec q_tendon = _structure.t() * q_joint;
  arma::vec q_motor = _Ra_inv * q_tendon;
  return q_motor;
}

arma::vec Transformer::motor_to_joint(const arma::vec & q_motor, bool enforce_limits)
{
  arma::vec q_tendon = _Ra * q_motor;
  arma::vec q_joint = _structure_inv.t() * q_tendon;

  if(enforce_limits) {
    bool inside =
      arma::all(q_joint >= _joint_min) &&
      arma::all(q_joint <= _joint_max);

    if(!inside) {
      std::cout << q_joint << std::endl;
      throw std::runtime_error("Output outside of joint limits");
    }
  }

  return q_joint;
}

arma::vec Transformer::joint_to_motor_torque(const arma::vec & t_joint)
{
  arma::vec f_tendon = _structure_inv * t_joint;
  arma::vec t_motor = _Ra * f_tendon;
  return t_motor;
}

arma::mat Transformer::get_jacobian_space(const arma::vec & q_joint)
{
    //TODO: come back to this and make sure its correct
  arma::mat J_full = mr::JacobianSpace(_screw_axes, q_joint);
  double dip_angle, speed_ratio;
  calculate_4bar_ratios(q_joint(2), dip_angle, speed_ratio);

  auto J = J_full.cols(0, 2);   // only the first 3 columns are relevant for the 3 joints
  J.col(2) = J_full.col(2) + J_full.col(3) * speed_ratio; // add the contribution from the 4-bar linkage
  return J;
}

arma::mat Transformer::get_jacobian_body(const arma::vec & q_joint)
{
  auto J_s = get_jacobian_space(q_joint);
  auto T_sb = joint_to_end_effector(q_joint);
  auto J_b = mr::Adjoint(arma::inv(T_sb)) * J_s;
  return J_b;
}

// the plan:
// Normally, IK uses a twist, but we will just use a position vector (x,y,z)
// Basically e = x_d - f(q) will be 3x3 instead of 6x6.
// This means that the Jacobian will also be 3x3 (using collapsed version), so it might be invertible?? will stick with pinv for now
// Will still use FK from modern robotics but just drop orientation
// might need to modify jacobian to make it coordinate not twist based..., should work fine using space frame though
// update: yeah had to switch the jacobian to be "coordinate" based (frame at e-e with space orientation)
arma::vec Transformer::end_effector_to_joint(const arma::vec & q_end_effector, double tolerance)
{
  constexpr int max_iter = 200;
  constexpr double max_step = 0.1;
  constexpr double lambda = 1e-3;

  const std::vector<arma::vec> seeds = {
    arma::vec({0.0, 0.5, 0.5}),
    arma::vec({0.0, 0.0, 0.0}),
    arma::vec({0.5, -0.5, 1.0}),
    arma::vec({-0.5, 1.0, -0.5}),
    arma::vec({1.0, 1.0, 1.0}),
    arma::vec({-1.0, -1.0, -1.0}),
  };

  auto try_ik = [&](const arma::vec & seed, arma::vec & thetalist) -> bool
    {
      thetalist = arma::min(arma::max(seed, _joint_min), _joint_max);

      for (int i = 0; i < max_iter; ++i) {
        const arma::vec pos_error = q_end_effector -
          joint_to_end_effector(thetalist).col(3).head(3);

        if (arma::norm(pos_error) <= tolerance) {
          return true;
        }

        arma::mat44 T(arma::fill::eye);
        T(arma::span(0, 2), 3) = -q_end_effector;

        const arma::mat Js = mr::Adjoint(T) * get_jacobian_space(thetalist);
        const arma::mat Js_sub = Js.submat(3, 0, 5, 2);

        arma::vec dtheta;
        if (arma::cond(Js_sub) > 1e6) {
          std::cout << "Warning: Jacobian near singular, using damped least squares" << std::endl;
          dtheta = Js_sub.t() * arma::solve(
          Js_sub * Js_sub.t() + lambda * arma::eye(3, 3), pos_error);
        } else {
          dtheta = arma::pinv(Js_sub) * pos_error;
        }

        const double step_norm = arma::norm(dtheta);
        if (step_norm > max_step) {
          dtheta *= max_step / step_norm;
        }

        thetalist += dtheta;
        thetalist = arma::min(arma::max(thetalist, _joint_min), _joint_max);
      }

      return false;
    };

  arma::vec result;
  for (std::size_t s = 0; s < seeds.size(); ++s) {
    if (try_ik(seeds[s], result)) {
      if (s > 0) {
        std::cout << "IK converged on seed " << s << std::endl;
      }
      return result;
    }
    std::cout << "Seed " << s << " failed, trying next..." << std::endl;
  }

  std::cout << "e-e goal:\n" << q_end_effector << std::endl;
  throw std::runtime_error("IK did not converge after " + std::to_string(seeds.size()) + " seeds");
}

arma::mat44 Transformer::joint_to_end_effector(const arma::vec & q_joint)
{

  bool inside =
    arma::all(q_joint >= _joint_min) &&
    arma::all(q_joint <= _joint_max);

  if(!inside) {
    throw std::runtime_error("Input outside of joint limits");
  }

  double pip_angle = q_joint(2);
  double dip_angle, speed_ratio;
  calculate_4bar_ratios(pip_angle, dip_angle, speed_ratio);

  arma::vec q_full = {q_joint(0), q_joint(1), q_joint(2), dip_angle};

    // compute the full forward kinematics using modern robotics
  arma::mat44 T = mr::FKinSpace(_M, _screw_axes, q_full);

  return T;
}

void Transformer::calculate_4bar_ratios(
  double pip_angle,
  double & dip_angle,
  double & speed_ratio)
{
  bool inside =
    pip_angle >= _joint_min(2) &&
    pip_angle <= _joint_max(2);

  if(!inside) {
    throw std::runtime_error("Input outside of joint limits");
  }

  double pip_angle_deg = pip_angle * 180.0 / M_PI;

    // Polynomial coefficients for dip_angle (highest degree first)
  static const double coeffs[] = {
    1.70488603e-12,
    -4.32223057e-10,
    4.06376685e-08,
    -1.09825341e-06,
    -7.29328873e-05,
    4.98961830e-03,
    9.22375906e-01,
    1.04087208e-02
  };

    // Derivative coefficients (multiply each by its degree, drop last term)
  static const double dcoeffs[] = {
    7 * 1.70488603e-12,
    6 * -4.32223057e-10,
    5 * 4.06376685e-08,
    4 * -1.09825341e-06,
    3 * -7.29328873e-05,
    2 * 4.98961830e-03,
    1 * 9.22375906e-01
  };

    // Evaluate dip_angle polynomial via Horner's method
  double result = 0.0;
  for (double coeff : coeffs) {
    result = result * pip_angle_deg + coeff;
  }
  dip_angle = result * M_PI / 180.0;

    // Evaluate derivative polynomial (speed_ratio) via Horner's method
  double dresult = 0.0;
  for (double dcoeff : dcoeffs) {
    dresult = dresult * pip_angle_deg + dcoeff;
  }
  speed_ratio = dresult;
}
