/// \file
/// \brief Runs a configurable single-shot demo trajectory. Waits for the Drake
///        heartbeat, then executes one trajectory selected by the `demo` parameter:
///        linear, linear_step, sinusoidal, force_step, ik, cartesian_ik, chirp, chirp_velocity, or lissajous.
///
/// PARAMETERS:
///   + demo (string) - Trajectory type to run: linear | linear_step | sinusoidal | force_step | ik | cartesian_ik | chirp | chirp_velocity | lissajous
///   + linear.waypoints (double[]) - Flat array of joint-space waypoints (groups of 3: splay, mcp, pipdip) (rad)
///   + linear.repeat (int) - Whether to repeat the trajectory (1 = repeat, 0 = once)
///   + linear_step.waypoints (double[]) - Flat array of joint-space waypoints (groups of 3: splay, mcp, pipdip) (rad)
///   + linear_step.freq (double) - Step frequency (Hz)
///   + linear_step.repeat (int) - Whether to repeat the step sequence (1 = repeat, 0 = once)
///   + sinusoidal.joint (int) - Joint index to oscillate (0=splay, 1=mcp, 2=pipdip)
///   + sinusoidal.amp (double) - Oscillation amplitude (rad)
///   + sinusoidal.freq (double) - Oscillation frequency (Hz)
///   + sinusoidal.v_offset (double) - Vertical offset (rad)
///   + sinusoidal.repeat (int) - Whether to repeat (1 = repeat, 0 = once)
///   + chirp.joint (int) - Joint index to sweep
///   + chirp.amp (double) - Chirp amplitude (rad)
///   + chirp.freq_init (double) - Starting frequency (Hz)
///   + chirp.freq_final (double) - Ending frequency (Hz)
///   + chirp.time (double) - Sweep duration (s)
///   + chirp.v_offset (double) - Vertical offset (rad)
///   + chirp_velocity.joint (int) - Joint index to sweep
///   + chirp_velocity.vel_amp (double) - Velocity amplitude (rad/s)
///   + chirp_velocity.freq_init (double) - Starting frequency (Hz)
///   + chirp_velocity.freq_final (double) - Ending frequency (Hz)
///   + chirp_velocity.time (double) - Sweep duration (s)
///   + chirp_velocity.start_pos (double[3]) - Starting joint position (rad)
///   + force.q_state (double[3]) - Joint-space state during force step <splay, mcp, pipdip> (rad)
///   + force.low (double[3]) - Low force vector <F_x, F_y, F_z> (N)
///   + force.high (double[3]) - High force vector <F_x, F_y, F_z> (N)
///   + force.freq (double) - Alternation frequency (Hz)
///   + force.repeat (int) - Whether to repeat (1 = repeat, 0 = once)
///   + ik.waypoints (double[]) - Flat array of Cartesian waypoints (groups of 3: x, y, z) (m)
///   + ik.repeat (int) - Number of back-and-forth repetitions for ik demos
///
/// SUBSCRIBES:
///   + /motor_pos_actual_feedback (finger_interfaces/msg/MotorFeedback) - Used to derive current joint state for relative moves
///
/// CLIENTS:
///   + /heartbeat (std_srvs/srv/Empty) - Blocks startup until the Drake simulation is ready
///   + /cartesian_move (finger_interfaces/action/Cartesian) - Sends end-effector waypoint goals
///   + /sinusoidal_move (finger_interfaces/action/Sinusoidal) - Sends sinusoidal joint trajectory goals
///   + /linear_move (finger_interfaces/action/Linear) - Sends linear joint-space trajectory goals
///   + /linear_step_move (finger_interfaces/action/LinearStep) - Sends joint-space waypoint step goals
///   + /force_step_move (finger_interfaces/action/Force) - Sends alternating force step goals
///   + /chirp_move (finger_interfaces/action/Chirp) - Sends frequency-sweep position chirp goals
///   + /chirp_velocity_move (finger_interfaces/action/ChirpVelocity) - Sends frequency-sweep velocity chirp goals

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <armadillo>

#include "finger_control/finger_control_base.hpp"


using namespace std::chrono_literals;

/// \brief A class that bridges commands and feedback between ros and drake
class FingerDemo : public FingerControlBase
{
public:

  /// \brief Create an instance of FingerDemo for running demos
  FingerDemo()
  : FingerControlBase("finger_demo")
  {
    declare_and_get_demo_params();

    if (demo_ == "linear") {
      RCLCPP_INFO(get_logger(), "Running linear joint movement demo...");

      // send_linear_goal(0, linear_waypoints_); // go to start

      send_linear_goal(linear_repeat_, linear_waypoints_);
    }

    else if (demo_ == "linear_step") {
      RCLCPP_INFO(get_logger(), "Running linear step joint movement demo...");

      // send_linear_goal(0, {linear_step_waypoints_.at(0)}); // go to start

      send_linear_step_goal(linear_step_repeat_, linear_step_freq_, linear_step_waypoints_);
    }

    else if (demo_ == "sinusoidal") {
      RCLCPP_INFO(get_logger(), "Running sinusoidal movement demo..."); // go to start

      send_sinusoid_goal(sinusoidal_repeat_, sinusoidal_joint_, sinusoidal_amp_, sinusoidal_freq_, sinusoidal_v_offset_);

      // send_linear_goal(0, {{0.0, 0.0, 0.7}});
      // rclcpp::sleep_for(50ms);
      // send_sinusoid_goal(1, 2, 0.35, 1.0, 0.7);
      // rclcpp::sleep_for(50ms);

      // send_linear_goal(0, {{0.0, 0.7, 0.0}});
      // rclcpp::sleep_for(50ms);
      // send_sinusoid_goal(1, 1, 0.35, 1.0, 0.7);
      // rclcpp::sleep_for(50ms);
      // send_linear_goal(0, {{0.0, 0.0, 0.0}});
      // rclcpp::sleep_for(50ms);
      // send_sinusoid_goal(1, 0, 0.25, 1.0, 0.0);
    }

    else if (demo_ == "force_step") {
      RCLCPP_INFO(get_logger(), "Running force step demo...");
      std::vector<float> q_joints(force_q_state_.begin(), force_q_state_.end());
      std::vector<float> force_low(force_low_.begin(), force_low_.end());
      std::vector<float> force_high(force_high_.begin(), force_high_.end());

      send_force_step_goal(q_joints, force_low, force_high, force_freq_, force_repeat_);
    }

    else if (demo_ == "ik") {
      RCLCPP_INFO(get_logger(), "Running inverse kinematics demo...");

      // move to start
      // send_cartesian_goal(0, {ik_waypoints_.at(0)});

      for (auto i = 0; i < 20; i++) {
        send_cartesian_goal(0, {ik_waypoints_.at(0)});
        rclcpp::sleep_for(250ms);
        send_cartesian_goal(0, {ik_waypoints_.at(1)});
        rclcpp::sleep_for(250ms);
        }
      }

    else if (demo_ == "cartesian_ik") {
      RCLCPP_INFO(get_logger(), "Running cartesian ik demo...");

      auto lerp_waypoints = 
      [](const std::vector<std::vector<float>> waypoints, int n = 30)
        {
          std::vector<std::vector<float>> points;
            for (int i = 1; i < int(waypoints.size()); ++i) {
                auto start = waypoints.at(i-1);
                auto end = waypoints.at(i);
                for (int j = 0; j <= n; j++) {
                  float t = static_cast<float>(j) / n;
                  points.push_back({
                      start[0] + t * (end[0] - start[0]),
                      start[1] + t * (end[1] - start[1]),
                      start[2] + t * (end[2] - start[2])});
                }
            }
         
        return points;
      };

      send_cartesian_goal(1, lerp_waypoints(ik_waypoints_));
    }

    else if (demo_ == "chirp") {
      RCLCPP_INFO(get_logger(), "Running chirp demo...");

      send_chirp_goal(chirp_joint_, chirp_amp_, chirp_freq_init_, chirp_freq_final_, chirp_time_, chirp_v_offset_);
    }

    else if (demo_ == "chirp_velocity") {
      RCLCPP_INFO(get_logger(), "Running chirp velocity demo...");

      std::vector<float> start_pos(chirp_velocity_start_pos_.begin(), chirp_velocity_start_pos_.end());

      send_chirp_velocity_goal(chirp_velocity_joint_, chirp_velocity_vel_amp_, chirp_velocity_freq_init_, chirp_velocity_freq_final_, chirp_velocity_time_, start_pos);
    }

    else if (demo_ == "lissajous") {
      RCLCPP_INFO(get_logger(), "Running lissajous demo...");
      auto lissajous = 
        [this](float y_offset = 0.07, float z_offset = -0.07, float f = 0.5, int hz = 100) {

            auto n = int(hz / f);
            RCLCPP_INFO_STREAM(get_logger(), n);
            std::vector<std::vector<float>> points;
            for (int i = 0; i <= n; ++i) {
                float t = static_cast<float>(i) /  n;
                points.push_back({
                    0.0f, // motion is in the x plane
                    1.5f * std::sinf(2 * M_PI * t) / 100.0f + y_offset,
                    1.5f * std::sinf(2 * M_PI* 2 * t + 3 * M_PI / 4) / 100.0f + z_offset - 0.0161f
                });
            }
            return points;
        };

      send_cartesian_goal(true, lissajous());
    }
  }

private:
  std::string demo_;
  int sinusoidal_repeat_;
  int sinusoidal_joint_;
  double sinusoidal_amp_;
  double sinusoidal_freq_;
  double sinusoidal_v_offset_;
  int chirp_joint_;
  double chirp_amp_;
  double chirp_freq_init_;
  double chirp_freq_final_;
  double chirp_time_;
  double chirp_v_offset_;
  int chirp_velocity_joint_;
  double chirp_velocity_vel_amp_;
  double chirp_velocity_freq_init_;
  double chirp_velocity_freq_final_;
  double chirp_velocity_time_;
  std::vector<double> chirp_velocity_start_pos_;
  std::vector<std::vector<float>> linear_waypoints_;
  int linear_repeat_;
  std::vector<std::vector<float>> linear_step_waypoints_;
  int linear_step_repeat_;
  double linear_step_freq_;
  double force_freq_;
  int force_repeat_;
  std::vector<double> force_q_state_;
  std::vector<double> force_low_;
  std::vector<double> force_high_;
  int ik_repeat_;
  std::vector<std::vector<float>> ik_waypoints_;

  void declare_and_get_demo_params()
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

    param_desc.description = "The demo that should be run.";
    declare_parameter("demo", "none", param_desc);

    param_desc.description = "Boolean (1 or 0) to enable repeat for sinusoidal";
    declare_parameter("sinusoidal.repeat", 0, param_desc);

    param_desc.description = "The joint to oscillate for sinusoidal demo.";
    declare_parameter("sinusoidal.joint", 0, param_desc);

    param_desc.description = "The amplitude to oscillate at for sinusoidal demo (rad).";
    declare_parameter("sinusoidal.amp", 0.0, param_desc);

    param_desc.description = "The frequency to oscillate at for sinusoidal demo (Hz).";
    declare_parameter("sinusoidal.freq", 0.0, param_desc);

    param_desc.description = "The vertical offset (in radians) for sinusoidal demo.";
    declare_parameter("sinusoidal.v_offset", 0.0, param_desc);

    param_desc.description = "The joint to oscillate for chirp demo.";
    declare_parameter("chirp.joint", 0, param_desc);

    param_desc.description = "The amplitude to oscillate at for chirp demo (rad).";
    declare_parameter("chirp.amp", 0.0, param_desc);

    param_desc.description = "The initial frequency for chirp demo (Hz).";
    declare_parameter("chirp.freq_init", 0.0, param_desc);

    param_desc.description = "The final frequency for chirp demo (Hz).";
    declare_parameter("chirp.freq_final", 0.0, param_desc);

    param_desc.description = "The length of time for the chirp demo in seconds.";
    declare_parameter("chirp.time", 0.0, param_desc);

    param_desc.description = "The vertical offset for chirp demo (in radians).";
    declare_parameter("chirp.v_offset", 0.0, param_desc);

    param_desc.description = "The joint to oscillate for velocity chirp demo.";
    declare_parameter("chirp_velocity.joint", 0, param_desc);

    param_desc.description = "The amplitude to oscillate at for velocity chirp demo (rad).";
    declare_parameter("chirp_velocity.vel_amp", 0.0, param_desc);

    param_desc.description = "The initial frequency for velocity chirp demo (Hz).";
    declare_parameter("chirp_velocity.freq_init", 0.0, param_desc);

    param_desc.description = "The final frequency for velocity chirp demo (Hz).";
    declare_parameter("chirp_velocity.freq_final", 0.0, param_desc);

    param_desc.description = "The length of time for the chirp velocity demo in seconds.";
    declare_parameter("chirp_velocity.time", 0.0, param_desc);

    param_desc.description = "The starting position for chirp velocity demo (in radians).";
    declare_parameter("chirp_velocity.start_pos", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "The list of 3-vector waypoints in joint space for linear move demo <splay, mcp, pipdip> in radians.";
    declare_parameter("linear.waypoints", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "Boolean (1 or 0) to indicate if linear demo should be repeated.";
    declare_parameter("linear.repeat", 0, param_desc);

    param_desc.description = "The list of 3 vector positions in joint space for linear step move demo in radians.";
    declare_parameter("linear_step.waypoints", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "The frequency to alterate linear step at for the linear step demo.";
    declare_parameter("linear_step.freq", 0.0, param_desc);

    param_desc.description = "Boolean (1 or 0) to indicate if linear step demo should be repeated.";
    declare_parameter("linear_step.repeat", 0, param_desc);

    param_desc.description = "The frequency to alterate force at for the force demo.";
    declare_parameter("force.freq", 0.0, param_desc);

    param_desc.description = "Boolean (1 or 0) to indicate if force demo should be repeated.";
    declare_parameter("force.repeat", 0, param_desc);

    param_desc.description = "The joint space state for force demo <splay, mcp, pipdip> in radians.";
    declare_parameter("force.q_state", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "The low force for the force demo <F_x, F_y, F_z> in newtons.";
    declare_parameter("force.low", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "The high force for the force demo <F_x, F_y, F_z> in newtons.";
    declare_parameter("force.high", std::vector<double>{0.0, 0.0, 0.0}, param_desc);

    param_desc.description = "Boolean (1 or 0) to indicate if ik demo should be repeated..";
    declare_parameter("ik.repeat", 0, param_desc);

    param_desc.description = "The list of 3 vector positions in cartesian space for linear step move demo in radians.";
    declare_parameter("ik.waypoints", std::vector<double>{0.0, 0.0, 0.0}, param_desc);


    demo_ = get_parameter("demo").as_string();
    sinusoidal_repeat_ = get_parameter("sinusoidal.repeat").as_int();
    sinusoidal_joint_ = get_parameter("sinusoidal.joint").as_int();
    sinusoidal_amp_ = get_parameter("sinusoidal.amp").as_double();
    sinusoidal_freq_ = get_parameter("sinusoidal.freq").as_double();
    sinusoidal_v_offset_ = get_parameter("sinusoidal.v_offset").as_double();
    chirp_joint_ = get_parameter("chirp.joint").as_int();
    chirp_amp_ = get_parameter("chirp.amp").as_double();
    chirp_freq_init_ = get_parameter("chirp.freq_init").as_double();
    chirp_freq_final_ = get_parameter("chirp.freq_final").as_double();
    chirp_time_ = get_parameter("chirp.time").as_double();
    chirp_v_offset_ = get_parameter("chirp.v_offset").as_double();
    chirp_velocity_joint_ = get_parameter("chirp_velocity.joint").as_int();
    chirp_velocity_vel_amp_ = get_parameter("chirp_velocity.vel_amp").as_double();
    chirp_velocity_freq_init_ = get_parameter("chirp_velocity.freq_init").as_double();
    chirp_velocity_freq_final_ = get_parameter("chirp_velocity.freq_final").as_double();
    chirp_velocity_time_ = get_parameter("chirp_velocity.time").as_double();
    chirp_velocity_start_pos_ = get_parameter("chirp_velocity.start_pos").as_double_array();
    linear_repeat_ = get_parameter("linear.repeat").as_int();
    linear_step_freq_ = get_parameter("linear_step.freq").as_double();
    linear_step_repeat_ = get_parameter("linear_step.repeat").as_int();
    force_q_state_ = get_parameter("force.q_state").as_double_array();
    force_repeat_ = get_parameter("force.repeat").as_int();
    force_freq_ = get_parameter("force.freq").as_double();
    force_low_ = get_parameter("force.low").as_double_array();
    force_high_ = get_parameter("force.high").as_double_array();
    ik_repeat_ = get_parameter("ik.repeat").as_int();

    // get linear step waypoints
    std::vector<double> flat_linear_step_waypoints = get_parameter("linear_step.waypoints").as_double_array();
    for (auto i = 0; i < int(flat_linear_step_waypoints.size()); i+=3) {
      linear_step_waypoints_.push_back({float(flat_linear_step_waypoints.at(i)),
                                        float(flat_linear_step_waypoints.at(i+1)),
                                        float(flat_linear_step_waypoints.at(i+2))});
    }

    std::vector<double> flat_linear_waypoints= get_parameter("linear.waypoints").as_double_array();
    for (auto i = 0; i < int(flat_linear_waypoints.size()); i+=3) {
      linear_waypoints_.push_back({float(flat_linear_waypoints.at(i)),
                                   float(flat_linear_waypoints.at(i+1)),
                                   float(flat_linear_waypoints.at(i+2))});
    }

    std::vector<double> flat_ik_waypoints= get_parameter("ik.waypoints").as_double_array();
    for (auto i = 0; i < int(flat_ik_waypoints.size()); i+=3) {
      ik_waypoints_.push_back({float(flat_ik_waypoints.at(i)),
                               float(flat_ik_waypoints.at(i+1)),
                               float(flat_ik_waypoints.at(i+2))});
    }

  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FingerDemo>());
  rclcpp::shutdown();
  return 0;
}
