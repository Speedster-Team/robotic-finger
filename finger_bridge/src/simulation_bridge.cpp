/// \file
/// \brief Bridges commands and feedback between ROS2 and the Drake simulation.
///        Receives trajectory commands and start/stop signals via services, then
///        steps through the command trajectory at 100 Hz and publishes motor
///        positions and activity state back to the ROS2 graph.
///
/// PUBLISHERS:
///   + /motor_pos_actual_feedback (finger_interfaces/msg/MotorFeedback) - Actual motor positions echoed from the command trajectory
///   + /motor_pos_setpoint_feedback (finger_interfaces/msg/MotorFeedback) - Setpoint motor positions echoed from the command trajectory
///   + /motor_pos_activity_feedback (finger_interfaces/msg/MotorActivity) - Motor activity state (1.0 while executing, 0.0 when idle)
///
/// SERVERS:
///   + /send_command (finger_interfaces/srv/SendCommand) - Receives and stores a motor position trajectory for playback
///   + /start_command (finger_interfaces/srv/StartStopCommand) - Begins stepping through the stored trajectory
///   + /stop_command (finger_interfaces/srv/StartStopCommand) - Halts trajectory playback

#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "finger_interfaces/srv/send_command.hpp"
#include "finger_interfaces/srv/start_stop_command.hpp"
#include "finger_interfaces/msg/motor_feedback.hpp"
#include "finger_interfaces/msg/motor_activity.hpp"

using namespace std::chrono_literals;

/// \brief State variable showing if data is ready to be sent to drake
enum State
{
  READY,
  WAITING,
};

/// \brief A class that bridges commands and feedback between ros and drake
class SimulationBridge : public rclcpp::Node
{
public:
  /// \brief Create an instance of SimulationBridge
  SimulationBridge()
  : Node("simulation_bridge"),
    state_ (State::WAITING)
  {
    // define send service callback function
    auto send_service_callback =
      [this](const std::shared_ptr<finger_interfaces::srv::SendCommand::Request> request,
      std::shared_ptr<finger_interfaces::srv::SendCommand::Response> response) -> void
      {
        RCLCPP_DEBUG(get_logger(), "send service request recieved...");

        // check that length field is filled
        if (request->length == 0) {
          response->success = 0;
          RCLCPP_ERROR(get_logger(), "send service request rejected, message field 'length' is 0.");
        } else if ((request->repeat != 0) && (request->repeat != 1)) {
          response->success = 0;
          RCLCPP_ERROR(get_logger(),
          "send service request rejected, message field 'request' is not 0 or 1.");
        } else {
          // save commands as vector
          commands_ = std::vector<std::vector<float>>(request->length, std::vector<float>(3));
          for (int i = 0; i < request->length; i++) {
            commands_[i] = {request->mcp_splay[i], request->mcp_flex[i], request->pip_flex[i]};
          }

          // save length and repeat
          length_ = request->length;
          repeat_ = request->repeat;

          // simulation will always recieve command
          response->success = 1;

          RCLCPP_DEBUG_STREAM(get_logger(),
          "send service request completed, response: " << int(response->success));
        }
      };
    // create callback group for send service
    send_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create send service
    send_service_ = create_service<finger_interfaces::srv::SendCommand>("/send_command",
      send_service_callback, rclcpp::ServicesQoS(), send_cb_group_);

    // define start service callback function
    auto start_service_callback =
      [this](const std::shared_ptr<finger_interfaces::srv::StartStopCommand::Request>,
      std::shared_ptr<finger_interfaces::srv::StartStopCommand::Response> response) -> void
      {
        RCLCPP_DEBUG(get_logger(), "start service request recieved...");

        // simulation will always recieve command
        response->success = 1;

        // make state ready
        state_ = State::READY;

        RCLCPP_DEBUG_STREAM(get_logger(),
        "start service request completed, response: " << int(response->success));
      };

    // create callback group for start service
    start_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create start service
    start_service_ = create_service<finger_interfaces::srv::StartStopCommand>("/start_command",
      start_service_callback, rclcpp::ServicesQoS(), start_cb_group_);

    // define stop service callback function
    auto stop_service_callback =
      [this](const std::shared_ptr<finger_interfaces::srv::StartStopCommand::Request>,
      std::shared_ptr<finger_interfaces::srv::StartStopCommand::Response> response) -> void
      {
        RCLCPP_DEBUG(get_logger(), "stop service request recieved...");

        // simulation will always recieve command
        response->success = 1;

        // make state waiting
        state_ = State::WAITING;

        RCLCPP_DEBUG_STREAM(get_logger(),
        "stop service request completed, response: " << int(response->success));
      };

    // create callback group for stop service
    stop_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create stop service
    stop_service_ = create_service<finger_interfaces::srv::StartStopCommand>("/stop_command",
      stop_service_callback, rclcpp::ServicesQoS(), stop_cb_group_);

    // create publishers
    actual_feedback_pub_ =
      create_publisher<finger_interfaces::msg::MotorFeedback>("/motor_pos_actual_feedback", 10);
    setpoint_feedback_pub_ =
      create_publisher<finger_interfaces::msg::MotorFeedback>("/motor_pos_setpoint_feedback", 10);
    activity_feedback_pub_ =
      create_publisher<finger_interfaces::msg::MotorActivity>("/motor_pos_activity_feedback", 10);

    // init motor feedback as zeros
    actual_feedback_.motor_positions = std::vector<float>{0.0, 0.0, 0.0};
    setpoint_feedback_.motor_positions = std::vector<float>{0.0, 0.0, 0.0};
    // actual_feedback_.motor_positions = std::vector<float>{-1.68f, -1.22, 0.896f};
    // setpoint_feedback_.motor_positions = std::vector<float>{-1.68f, -1.22, 0.896f};
    activity_feedback_.active = 0.0;

    // define timer callback and init
    auto command_sender_timer_callback =
      [this]() -> void {
        // init count
        static auto count = 0;

        if (state_ == State::READY) {
          RCLCPP_DEBUG_ONCE(get_logger(), "publishing commands to drake...");

          // publish commands to drake, same for actual and setpoint
          actual_feedback_.motor_positions = {commands_.at(count).at(0), commands_.at(count).at(1),
            commands_.at(count).at(2)};
          setpoint_feedback_.motor_positions = {commands_.at(count).at(0),
            commands_.at(count).at(1), commands_.at(count).at(2)};

          // increment counter
          count++;

            // check for overflow
          if (count >= length_) {
            count = 0;
            if (repeat_ == 0) {
                    // disable control if no repeat
              state_ = State::WAITING;
            }
          }
        }

        // publish same as feedback
        activity_feedback_.active = (state_ == State::READY) ? 1.0 : 0.0;
        actual_feedback_.header.stamp = now();
        setpoint_feedback_.header.stamp = now();
        activity_feedback_.header.stamp = now();
        actual_feedback_pub_->publish(actual_feedback_);
        setpoint_feedback_pub_->publish(setpoint_feedback_);
        activity_feedback_pub_->publish(activity_feedback_);
      };
    timer_ = this->create_wall_timer(10ms, command_sender_timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  State state_;
  finger_interfaces::msg::MotorFeedback actual_feedback_;
  finger_interfaces::msg::MotorFeedback setpoint_feedback_;
  finger_interfaces::msg::MotorActivity activity_feedback_;
  rclcpp::Publisher<finger_interfaces::msg::MotorFeedback>::SharedPtr actual_feedback_pub_;
  rclcpp::Publisher<finger_interfaces::msg::MotorFeedback>::SharedPtr setpoint_feedback_pub_;
  rclcpp::Publisher<finger_interfaces::msg::MotorActivity>::SharedPtr activity_feedback_pub_;
  rclcpp::Service<finger_interfaces::srv::SendCommand>::SharedPtr send_service_;
  rclcpp::Service<finger_interfaces::srv::StartStopCommand>::SharedPtr start_service_;
  rclcpp::Service<finger_interfaces::srv::StartStopCommand>::SharedPtr stop_service_;
  rclcpp::CallbackGroup::SharedPtr send_cb_group_;
  rclcpp::CallbackGroup::SharedPtr start_cb_group_;
  rclcpp::CallbackGroup::SharedPtr stop_cb_group_;
  std::vector<std::vector<float>> commands_;
  int length_;
  int repeat_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<SimulationBridge>());
  auto node = std::make_shared<SimulationBridge>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
