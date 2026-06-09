/// \file
/// \brief High-level whack-a-mole control node. Polls the TF tree for a "goal"
///        frame published by the vision system, sends a cartesian IK move when
///        a new goal appears, and publishes /completed once the fingertip
///        arrives at the goal position. Operates as a state machine at 10 Hz:
///        IDLE → START_MOVE → MOVING → IDLE.
///
/// SUBSCRIBES:
///   + /motor_pos_actual_feedback (finger_interfaces/msg/MotorFeedback) - Used to derive current joint state for relative moves
///
/// PUBLISHERS:
///   + /completed (std_msgs/msg/Empty) - Published once the fingertip reaches the goal
///
/// TF LISTENERS:
///   + base_frame → goal       - Target position published by the vision system
///   + base_frame → fingertip  - Current fingertip position for arrival detection
///
/// CLIENTS:
///   + /heartbeat (std_srvs/srv/Empty) - Blocks startup until the Drake simulation is ready
///   + /cartesian_move (finger_interfaces/action/Cartesian) - Sends end-effector waypoint goals
///   + /sinusoidal_move (finger_interfaces/action/Sinusoidal) - Sends sinusoidal joint trajectory goals
///   + /linear_move (finger_interfaces/action/Linear) - Sends linear joint-space trajectory goals
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
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;


/// \brief State variable for moving to published goal points
enum FingerState
{
  START_MOVE,
  MOVING,
  IDLE,
};

/// \brief A class that bridges commands and feedback between ros and drake
class FingerWhackamole : public FingerControlBase
{
public:

  /// \brief Create an instance of FingerWhackamole running the whackamole demo
  FingerWhackamole()
  : FingerControlBase("finger_whackamole"),
    finger_state_ (FingerState::IDLE),
    hit_count_ (0)
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "The fingertip that should be looked for.";
    declare_parameter("bridge", "simulation", param_desc);
    auto bridge = get_parameter("bridge").as_string();

    // init tf2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // init publisher
    completed_pub_ = create_publisher<std_msgs::msg::Empty>("/completed", 10);

    // init tfs
    prev_tf_.transform.translation.x = 0.0;
    prev_tf_.transform.translation.y = 0.0;
    prev_tf_.transform.translation.z = 0.0;
    goal_tf_.transform.translation.x = 0.0;
    goal_tf_.transform.translation.y = 0.0;
    goal_tf_.transform.translation.z = 0.0;
    new_goal_tf_.transform.translation.x = 0.0;
    new_goal_tf_.transform.translation.y = 0.0;
    new_goal_tf_.transform.translation.z = 0.0;

    // init frame names
    base_frame_ = "base_frame";
    goal_frame_ = "goal";

    send_linear_goal(0, {{0.0, 0.0, 1.0}});

    // define timer callback and init
    auto hyper_alg_timer_cb =
      [this]() -> void {

        switch (finger_state_) {

          case FingerState::IDLE: {

            // listen for new goal positions
            try {
              goal_tf_ = tf_buffer_->lookupTransform(
                base_frame_, goal_frame_,
                tf2::TimePointZero);
              
              if (!similar_tfs(goal_tf_, prev_tf_)) {
                // if successful switch state and move finger to this location
                finger_state_ = FingerState::START_MOVE;
              }

            } catch (const tf2::TransformException & ex) {
              RCLCPP_INFO_ONCE(get_logger(), "Could not transform %s to %s: %s",
                base_frame_.c_str(), goal_frame_.c_str(), ex.what());
              return;
            }
            break;
          }
          case FingerState::START_MOVE: {
            // convert to vector
            std::vector<float> above_goal = {float(goal_tf_.transform.translation.x),
                                             float(goal_tf_.transform.translation.y - 0.015f),
                                             float(goal_tf_.transform.translation.z + 0.015f)};
            std::vector<float> goal = {float(goal_tf_.transform.translation.x),
                                       float(goal_tf_.transform.translation.y),
                                       float(goal_tf_.transform.translation.z)};

            // just the goal
            // send_cartesian_goal(0, {goal});
            // send_linear_goal(0, {{0,0,0},{0.0, 0.025, 1.25}});
            send_cartesian_goal(0, {above_goal});
            send_cartesian_goal(0, {above_goal, goal});
            // rclcpp::sleep_for(50ms);
            // send_cartesian_goal(0, {goal, above_goal});

            // send_linear_goal(0, {{0.0, 0.0, 0.0}});

            // update prev_tf
            prev_tf_ = goal_tf_;

            // update state
            finger_state_ = FingerState::MOVING;

            break;
            }
          case FingerState::MOVING: {
            // listen if finger has reached goal position
            try {
              new_goal_tf_ = tf_buffer_->lookupTransform(
                base_frame_, goal_frame_,
                tf2::TimePointZero);
              
              // check if goal state has moved
              if ((!similar_tfs(new_goal_tf_, goal_tf_))) {
                std_msgs::msg::Empty msg;
                completed_pub_->publish(msg);

                // if successful switch state and wait for next goal
                finger_state_ = FingerState::IDLE;
              } 
              // else if (hit_count_ == 0) {
              //   // wait 10ms and check again to see if finger hit the goal (don't change state)
              //   // rclcpp::sleep_for(50ms);

              //   // increment hit count
              //   hit_count_++;
              // } else {
              //   // if already hit once and the goal hasn't changed, try again
              //   finger_state_ = FingerState::START_MOVE;

              //   // set hit count to 0 so we check again after a delay if the point has changed
              //   hit_count_ = 0;
              // }

            } catch (const tf2::TransformException & ex) {
              RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s",
                base_frame_.c_str(), goal_frame_.c_str(), ex.what());
              return;
            }
            break;
          }
        }        
      };
      
    timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(5ms, hyper_alg_timer_cb, timer_cb_group_);

  
  }

private:
  FingerState finger_state_;
  int hit_count_;

  std::string base_frame_;
  std::string goal_frame_;
  geometry_msgs::msg::TransformStamped goal_tf_;
  geometry_msgs::msg::TransformStamped prev_tf_;
  geometry_msgs::msg::TransformStamped new_goal_tf_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr completed_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

  bool similar_tfs(geometry_msgs::msg::TransformStamped tf1, geometry_msgs::msg::TransformStamped tf2, double thresh = 0.001) {

    auto x_diff = fabs(tf1.transform.translation.x - tf2.transform.translation.x);
    auto y_diff = fabs(tf1.transform.translation.y - tf2.transform.translation.y);
    auto z_diff = fabs(tf1.transform.translation.z - tf2.transform.translation.z);

    if ((x_diff > thresh) || (y_diff > thresh) || (z_diff > thresh)) {
      return false;
    } else {
      return true;
    }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FingerWhackamole>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
