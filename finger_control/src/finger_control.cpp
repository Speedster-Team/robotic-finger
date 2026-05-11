/// \file
/// \brief Runs high level control coordinating perception and movement commands.
///        Waits for the Drake simulation heartbeat, then dispatches cartesian,
///        sinusoidal, and linear trajectory goals to the finger planner.
///
/// CLIENTS:
///   + /heartbeat (std_srvs/srv/Empty) - Blocks startup until the Drake simulation is ready
///   + /cartesian_move (finger_interfaces/action/Cartesian) - Sends end-effector waypoint goals
///   + /sinusoidal_move (finger_interfaces/action/Sinusoidal) - Sends sinusoidal joint trajectory goals
///   + /linear_move (finger_interfaces/action/Linear) - Sends linear joint-space trajectory goals

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"

#include "finger_interfaces/action/cartesian.hpp"
#include "finger_interfaces/action/sinusoidal.hpp"
#include "finger_interfaces/action/linear.hpp"

using namespace std::chrono_literals;

/// \brief A class that bridges commands and feedback between ros and drake
class FingerControl : public rclcpp::Node
{
public:
  using GoalHandleCartesian = rclcpp_action::ClientGoalHandle<finger_interfaces::action::Cartesian>;
  using GoalHandleSinusoidal = rclcpp_action::ClientGoalHandle<finger_interfaces::action::Sinusoidal>;
  using GoalHandleLinear = rclcpp_action::ClientGoalHandle<finger_interfaces::action::Linear>;
  using Cartesian = finger_interfaces::action::Cartesian;
  using Sinusoidal = finger_interfaces::action::Sinusoidal;
  using Linear = finger_interfaces::action::Linear;

  /// \brief Create an instance of SimulationBridge
  FingerControl()
  : Node("finger_control")
  {
    // wait for drake to startup
    wait_for_drake_heartbeat();

    // create action clients
    cartesian_client_ = rclcpp_action::create_client<Cartesian>(this,
      "/cartesian_move");
    sinusoidal_client_ = rclcpp_action::create_client<Sinusoidal>(this,
      "/sinusoidal_move");
    linear_client_ = rclcpp_action::create_client<Linear>(this,
      "/linear_move");

    // wait for servers to appear
    while (!cartesian_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(),
          "client interrupted while waiting for cartesian_move action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for cartesian_move action to appear...");
    }
    while (!sinusoidal_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(),
          "client interrupted while waiting for sinusoidal_move action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for sinusoidal_move action to appear...");
    }
    while (!linear_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(),
          "client interrupted while waiting for linear_move action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for linear_move action to appear...");
    }

    // sleep for 1 second
    rclcpp::sleep_for(3000ms);


    // // JOINT STEP MOTION DEMO

    // // send test linear command
    // std::vector<float> start_joint_loc = {0.0, 0.0, 0.0};
    // std::vector<float> end_joint_loc = {0.0, 1.256, 0.0};
    // for (auto i = 0; i < 10; i++) {
    //   send_linear_goal(start_joint_loc, end_joint_loc);
    //   send_linear_goal(end_joint_loc, start_joint_loc);
    // }

    // // send test linear command
    // start_joint_loc = {0.0, 0.0, 0.0};
    // end_joint_loc = {0.0, 0.0, 1.256};
    // for (auto i = 0; i < 10; i++) {
    //   send_linear_goal(start_joint_loc, end_joint_loc);
    //   send_linear_goal(end_joint_loc, start_joint_loc);
    // }


    // send test sinusoidal command
    send_sinusoid_goal(1, 1, 0.2, 10.0, 0.7);

    // // ik demo
    // std::vector<float> start = {0.05f, 0.1f, -0.1f};            
    // std::vector<float> end   = {-0.05f, 0.1f, -0.1f};
                                                                
    // for (auto i = 0; i < 20; i++) {
    //     send_cartesian_goal({start, end});        
    //     send_cartesian_goal({end, start});
    // }   

  // // IK DEMO

  // // pseudo cartesian movements
  // auto lerp_waypoints = [](const std::vector<float>& start,
  //   const std::vector<float>& end, int n = 30) {                
  //       std::vector<std::vector<float>> points;
  //       for (int i = 0; i <= n; ++i) {                          
  //           float t = static_cast<float>(i) / n;
  //           points.push_back({                                  
  //               start[0] + t * (end[0] - start[0]),
  //               start[1] + t * (end[1] - start[1]),             
  //               start[2] + t * (end[2] - start[2])              
  //           });
  //       }                                                       
  //       return points;   
  //   };

  //   std::vector<float> start = {0.05f, 0.1f, -0.1f};            
  //   std::vector<float> end   = {-0.05f, 0.1f, -0.1f};
                                                                
  //   for (auto i = 0; i < 20; i++) {
  //       send_cartesian_goal(lerp_waypoints(start, end));        
  //       send_cartesian_goal(lerp_waypoints(end, start));
  //   }      
  }

private:
  rclcpp_action::Client<Cartesian>::SharedPtr cartesian_client_;
  rclcpp_action::Client<Sinusoidal>::SharedPtr sinusoidal_client_;
  rclcpp_action::Client<Linear>::SharedPtr linear_client_;

  void wait_for_drake_heartbeat()
  {
    // create dummy client to wait until drake is initialized
    auto heartbeat_client = create_client<std_srvs::srv::Empty>("/heartbeat", 10);

    // wait for server to appear
    while (!heartbeat_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for service to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for heartbeat service to appear...");
    }
  }

  void send_cartesian_goal(std::vector<std::vector<float>> waypoints)
  {
    auto goal_msg = finger_interfaces::action::Cartesian::Goal();

    // create goal request
    goal_msg.length = int(waypoints.size());
    for (auto & wp : waypoints) {
      goal_msg.x.push_back(wp.at(0));
      goal_msg.y.push_back(wp.at(1));
      goal_msg.z.push_back(wp.at(2));
    }

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Cartesian>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleCartesian::SharedPtr & goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };

    send_goal_options.feedback_callback = [this](
      GoalHandleCartesian::SharedPtr,
      const std::shared_ptr<const Cartesian::Feedback>)
      {
        RCLCPP_INFO(get_logger(), "feedback recieved...");
      };

    send_goal_options.result_callback = [this](const GoalHandleCartesian::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = cartesian_client_->async_send_goal(goal_msg, send_goal_options);

    // Wait for goal to be accepted
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected");
      return;
    }

    // Now wait for the result
    auto result_future = cartesian_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get result");
      return;
    }

    RCLCPP_INFO(get_logger(), "Goal completed");
  }


  void send_sinusoid_goal(bool repeat, int joint, float amp, float freq, float v_shift)
  {
    auto goal_msg = finger_interfaces::action::Sinusoidal::Goal();

    // create goal request
    goal_msg.repeat = int(repeat);
    goal_msg.joint = joint;
    goal_msg.amp = amp;
    goal_msg.freq = freq;
    goal_msg.v_shift = v_shift;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Sinusoidal>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleSinusoidal::SharedPtr & goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };

    send_goal_options.feedback_callback = [this](
      GoalHandleSinusoidal::SharedPtr,
      const std::shared_ptr<const Sinusoidal::Feedback>)
      {
        RCLCPP_INFO(get_logger(), "feedback recieved...");
      };

    send_goal_options.result_callback = [this](const GoalHandleSinusoidal::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = sinusoidal_client_->async_send_goal(goal_msg, send_goal_options);

    // Wait for goal to be accepted
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected");
      return;
    }

    // Now wait for the result
    auto result_future = sinusoidal_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get result");
      return;
    }

    RCLCPP_INFO(get_logger(), "Goal completed");
  }

  void send_linear_goal(std::vector<float> start, std::vector<float> end)
  {
    auto goal_msg = finger_interfaces::action::Linear::Goal();

    // create goal request
    goal_msg.start = start;
    goal_msg.end = end;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Linear>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleLinear::SharedPtr & goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };

    send_goal_options.feedback_callback = [this](
      GoalHandleLinear::SharedPtr,
      const std::shared_ptr<const Linear::Feedback>)
      {
        RCLCPP_INFO(get_logger(), "feedback recieved...");
      };

    send_goal_options.result_callback = [this](const GoalHandleLinear::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = linear_client_->async_send_goal(goal_msg, send_goal_options);

    // Wait for goal to be accepted
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rejected");
      return;
    }

    // Now wait for the result
    auto result_future = linear_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get result");
      return;
    }

    RCLCPP_INFO(get_logger(), "Goal completed");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FingerControl>());
//   auto node = std::make_shared<FingerControl>();
//   rclcpp::executors::MultiThreadedExecutor exec;
//   exec.add_node(node);
//   exec.spin();
  rclcpp::shutdown();
  return 0;
}
