/// \file
/// \brief Coordinates motion planning for finger movement. Accepts action goals
///        for cartesian, linear, and sinusoidal trajectories, generates motor
///        position commands, and manages execution state via send/start/stop services.
///
/// PARAMETERS:
///   + max_velocity (double) - Maximum joint velocity for trajectory generation
///   + max_acceleration (double) - Maximum joint acceleration for trajectory generation
///   + relative_gnd_height (double) - Height of the ground relative to the finger base (should be negative)
///
/// SUBSCRIBES:
///   + /motor_pos_actual_feedback (finger_interfaces/msg/MotorFeedback) - Actual motor positions
///   + /motor_pos_setpoint_feedback (finger_interfaces/msg/MotorFeedback) - Setpoint motor positions
///   + /motor_pos_activity_feedback (finger_interfaces/msg/MotorActivity) - Motor activity state
///
/// CLIENTS:
///   + /send_command (finger_interfaces/srv/SendCommand) - Sends a motor position trajectory to the finger
///   + /start_command (finger_interfaces/srv/StartStopCommand) - Starts execution of the sent trajectory
///   + /stop_command (finger_interfaces/srv/StartStopCommand) - Stops execution of the current trajectory
///
/// ACTIONS:
///   + /cartesian_move (finger_interfaces/action/Cartesian) - Move the end effector through a list of Cartesian waypoints
///   + /sinusoidal_move (finger_interfaces/action/Sinusoidal) - Drive a single joint with a sinusoidal trajectory
///   + /linear_move (finger_interfaces/action/Linear) - Move the end effector linearly between two joint-space points

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "finger_interfaces/srv/send_command.hpp"
#include "finger_interfaces/srv/start_stop_command.hpp"
#include "finger_interfaces/action/cartesian.hpp"
#include "finger_interfaces/action/sinusoidal.hpp"
#include "finger_interfaces/action/linear.hpp"
#include "finger_interfaces/action/force.hpp"
#include "finger_interfaces/msg/motor_feedback.hpp"
#include "finger_interfaces/msg/motor_activity.hpp"

#include "fingerlib/joint_trajectory.hpp"

using namespace std::chrono_literals;

/// \brief State variable for the command sent to the finger
enum CmdState
{
  BEGIN,
  IDLE,
  RECEIVED,
  STARTED,
  CANCELLED,
  STOPPING,
};

class FingerPlanner : public rclcpp::Node
{
public:
  using GoalHandleCartesian = rclcpp_action::ServerGoalHandle<finger_interfaces::action::Cartesian>;
  using GoalHandleSinusoidal = rclcpp_action::ServerGoalHandle<finger_interfaces::action::Sinusoidal>;
  using GoalHandleLinear = rclcpp_action::ServerGoalHandle<finger_interfaces::action::Linear>;
  using GoalHandleForce = rclcpp_action::ServerGoalHandle<finger_interfaces::action::Force>;

  FingerPlanner()
  : Node("finger_planner"),
    ra_ (0.0025), rb_ (0.0025), rc_ (0.0025),
    r1_ (0.008), r3_ (0.0045), r5_ (0.008), r7_ (0.0045), r9_ (0.009), r11_ (ra_ * 3.5),
    Ra_ {{ra_, 0, 0},          // splay
      {0, rb_, 0},             // mcp
      {0, 0, rc_}},            // pip/dip
    St_  {{-r11_, -r3_, r1_},      // splay joint
      {0, r7_, r5_},            // mcp joint
      {0, 0, r9_}},              // pip/dip joint
    slist_ {arma::vec6({0, 0, 1, 0, 0, 0}),
      arma::vec6({-1, 0, 0, 0, 0, 0.01776}),
      arma::vec6({-1, 0, 0, 0, 0, 0.07776}),
      arma::vec6({-1, 0, 0, 0, 0, 0.11836})},
    joint_min_ {-0.55, -0.2, -0.01},
    joint_max_ {0.55, 1.572, 1.572},
    M_ {{1, 0, 0, 0},
      {0, 1, 0, 0.16},
      {0, 0, 1, 0},
      {0, 0, 0, 1}},
    four_bar_lengths_ {8.83765 * 0.001,
      40.6 * 0.001,
      8.91536 * 0.001,
      37.79903 * 0.001},
    msg_attempts_ (0),
    cmd_state_ (CmdState::BEGIN)
  {
    // declare parameters
    auto param_desc1 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc1.description = "The maximum joint velocity.";
    declare_parameter("max_velocity", 0.1, param_desc1);

    auto param_desc2 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc2.description = "The maximum joint acceleration.";
    declare_parameter("max_acceleration", 0.1, param_desc2);

    auto param_desc3 = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc3.description = "The relative height of the ground (should be negative).";
    declare_parameter("relative_gnd_height", -0.25, param_desc3);

    // get parameters
    max_vel_ = get_parameter("max_velocity").as_double();
    max_accel_ = get_parameter("max_acceleration").as_double();
    gnd_height_ = get_parameter("relative_gnd_height").as_double();

    // create transformer class
    transforms_ = std::make_shared<Transformer>(Ra_, St_, slist_, M_, four_bar_lengths_, joint_min_,
      joint_max_);
    generator_ = std::make_shared<JointTrajectory>(*transforms_, 100, gnd_height_);

    auto motor_pos_actual_sub_callback =
      [this](finger_interfaces::msg::MotorFeedback::UniquePtr msg) -> void {
        // save feedback
        motor_actual_feedback_ = *msg;
      };

    motor_actual_feedback_sub_ =
      create_subscription<finger_interfaces::msg::MotorFeedback>("/motor_pos_actual_feedback",
      10, motor_pos_actual_sub_callback);

    auto motor_pos_setpoint_sub_callback =
      [this](finger_interfaces::msg::MotorFeedback::UniquePtr msg) -> void {
        // save feedback
        motor_setpoint_feedback_ = *msg;
      };

    motor_setpoint_feedback_sub_ =
      create_subscription<finger_interfaces::msg::MotorFeedback>("/motor_pos_setpoint_feedback",
      10, motor_pos_setpoint_sub_callback);

    auto motor_activity_sub_callback =
      [this](finger_interfaces::msg::MotorActivity::UniquePtr msg) -> void {
        // save feedback
        motor_activity_feedback_ = *msg;
      };

    motor_activity_feedback_sub_ =
      create_subscription<finger_interfaces::msg::MotorActivity>("/motor_pos_activity_feedback", 10,
      motor_activity_sub_callback);

    // create clients
    send_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    start_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    stop_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    send_client_ = create_client<finger_interfaces::srv::SendCommand>("/send_command", 10,
      send_cb_group_);
    start_client_ = create_client<finger_interfaces::srv::StartStopCommand>("/start_command", 10,
      start_cb_group_);
    stop_client_ = create_client<finger_interfaces::srv::StartStopCommand>("/stop_command", 10,
      stop_cb_group_);

    // wait for clients to appear
    while (!send_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for send service to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for send service to appear...");
    }
    while (!start_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for start service to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for start service to appear...");
    }
    while (!stop_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for stop service to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for stop service to appear...");
    }

    // init callback groups for actions and action timers
    action_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // create cartesian move action
    auto cartesian_handle_goal = [this](
      const rclcpp_action::GoalUUID,
      std::shared_ptr<const finger_interfaces::action::Cartesian::Goal> goal)
      {
        // check that requested waypoints are same length
        if ((goal->length == int(goal->x.size())) && (goal->length == int(goal->y.size())) &&
          (goal->length == int(goal->z.size())))
        {
          // save waypoints as vector
          auto waypoints_temp = std::vector<arma::vec>();
          for (auto i = 0; i < goal->length; i++) {
            waypoints_temp.push_back({goal->x.at(i), goal->y.at(i), goal->z.at(i)});
          }

          // print request
          RCLCPP_INFO(get_logger(), "Received cartesian goal request with length %d and waypoints:",
          goal->length);

          // check that waypoints are within the joint limits
          try {
            RCLCPP_INFO_STREAM(get_logger(),
            "waypoint 0: (" << goal->x.at(0) << ", " << goal->y.at(0) << ", " << goal->z.at(0) <<
              ")");
            for(auto i = 1; i < goal->length; i++) {
              RCLCPP_INFO_STREAM(get_logger(),
              "waypoint " << i << ": (" << goal->x.at(i) << ", " << goal->y.at(i) << ", " <<
                goal->z.at(i) << ")");
              auto start = transforms_->end_effector_to_joint(waypoints_temp[i - 1]);
              auto end = transforms_->end_effector_to_joint(waypoints_temp[i]);
            }
          } catch (std::runtime_error & e) {
            RCLCPP_ERROR_STREAM(get_logger(),
            "Goal request REJECTED because waypoint is outside of joint limits!!");
            return rclcpp_action::GoalResponse::REJECT;
          }

          // accept request
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
          // print error
          RCLCPP_INFO(get_logger(), "Goal request REJECTED because waypoints are malformed.");

          // reject request
          return rclcpp_action::GoalResponse::REJECT;
        }
      };

    auto cartesian_handle_cancel = [this](
      const std::shared_ptr<GoalHandleCartesian>)
      {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel cartesian goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto cartesian_handle_accepted = [this](
      const std::shared_ptr<GoalHandleCartesian> goal_handle)
      {
        // set cmd state
        cmd_state_ = CmdState::BEGIN;

        // init message attempts to 0 to be safe
        msg_attempts_ = 0;

        // start timer for action
        action_timer_ = create_wall_timer(100ms, [this, goal_handle](){
              return this->execute_cartesian_goal(goal_handle);
            },
        timer_cb_group_);
      };

    cartesian_action_server_ = rclcpp_action::create_server<finger_interfaces::action::Cartesian>(
      this,
      "/cartesian_move",
      cartesian_handle_goal,
      cartesian_handle_cancel,
      cartesian_handle_accepted,
      rcl_action_server_get_default_options(),
      action_cb_group_);

    // create sinusoidal move action
    auto sinusoidal_handle_goal = [this](
      const rclcpp_action::GoalUUID,
      std::shared_ptr<const finger_interfaces::action::Sinusoidal::Goal> goal)
      {
        // check that joint is 0, 1, or 2
        // TODO: add more checks here?
        if (((goal->joint == 0) || (goal->joint == 1) || (goal->joint == 2))) {

          // print request
          RCLCPP_INFO_STREAM(get_logger(), "Received sinusoidal goal request for joint" <<
            goal->joint << " with amp " << goal->amp << ", freq " << goal->freq <<
            ", and v_shift " << goal->v_shift);

          // accept request
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
          // print error
          RCLCPP_INFO(get_logger(), "Goal request REJECTED because joint is not 0, 1, or 2.");

          // reject request
          return rclcpp_action::GoalResponse::REJECT;
        }
      };

    auto sinusoidal_handle_cancel = [this](
      const std::shared_ptr<GoalHandleSinusoidal>)
      {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel sinusoidal goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto sinusoidal_handle_accepted = [this](
      const std::shared_ptr<GoalHandleSinusoidal> goal_handle)
      {
        // set cmd state
        cmd_state_ = CmdState::BEGIN;

        // init message attempts to 0 to be safe
        msg_attempts_ = 0;

        // start timer for action
        action_timer_ = create_wall_timer(100ms, [this, goal_handle](){
              return this->execute_sinusoidal_goal(goal_handle);
            },
        timer_cb_group_);
      };

    sinusoidal_action_server_ = rclcpp_action::create_server<finger_interfaces::action::Sinusoidal>(
      this,
      "/sinusoidal_move",
      sinusoidal_handle_goal,
      sinusoidal_handle_cancel,
      sinusoidal_handle_accepted,
      rcl_action_server_get_default_options(),
      action_cb_group_);

    // create linear move action
    auto linear_handle_goal = [this](
      const rclcpp_action::GoalUUID,
      std::shared_ptr<const finger_interfaces::action::Linear::Goal> goal)
      {
        RCLCPP_INFO(get_logger(), "Received linear goal request:");

        if ((int(goal->start.size()) == 3) && (int(goal->end.size()) == 3)) {
          // check that start and end are within the joint limits
          try {
            RCLCPP_INFO_STREAM(get_logger(),
          "start: (" << goal->start.at(0) << ", " << goal->start.at(1) << ", " <<
              goal->start.at(2) << ")");
            RCLCPP_INFO_STREAM(get_logger(),
          "end: (" << goal->end.at(0) << ", " << goal->end.at(1) << ", " << goal->end.at(2) << ")");

            arma::vec start_vec = {goal->start.at(0), goal->start.at(1), goal->start.at(2)};
            arma::vec end_vec = {goal->start.at(0), goal->start.at(1), goal->start.at(2)};

            auto start = transforms_->joint_to_end_effector(start_vec);
            auto end = transforms_->joint_to_end_effector(end_vec);

          } catch (std::runtime_error & e) {
            RCLCPP_ERROR_STREAM(get_logger(),
          "Goal request REJECTED because start or end is outside of joint limits!!");
            return rclcpp_action::GoalResponse::REJECT;
          }

          // accept request
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
          // print error
          RCLCPP_INFO(get_logger(), "Goal request REJECTED because waypoints are malformed.");

          // reject request
          return rclcpp_action::GoalResponse::REJECT;
        }
      };

    auto linear_handle_cancel = [this](
      const std::shared_ptr<GoalHandleLinear>)
      {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel linear goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto linear_handle_accepted = [this](
      const std::shared_ptr<GoalHandleLinear> goal_handle)
      {
        // set cmd state
        cmd_state_ = CmdState::BEGIN;

        // init message attempts to 0 to be safe
        msg_attempts_ = 0;

        // start timer for action
        action_timer_ = create_wall_timer(100ms, [this, goal_handle](){
              return this->execute_linear_goal(goal_handle);
          },
      timer_cb_group_);
      };

    linear_action_server_ = rclcpp_action::create_server<finger_interfaces::action::Linear>(
    this,
    "/linear_move",
    linear_handle_goal,
    linear_handle_cancel,
    linear_handle_accepted,
    rcl_action_server_get_default_options(),
    action_cb_group_);

    // create force step action
    auto force_step_handle_goal = [this](
      const rclcpp_action::GoalUUID,
      std::shared_ptr<const finger_interfaces::action::Force::Goal> goal)
      {
        RCLCPP_INFO(get_logger(), "Received force step goal request:");

        if ((int(goal->q_joint.size()) == 3) && (int(goal->force_low.size()) == 3) && (int(goal->force_high.size()) == 3)) {
        
          // accept request
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
          // print error
          RCLCPP_INFO(get_logger(), "Goal request REJECTED because joint state or force goals are malformed.");

          // reject request
          return rclcpp_action::GoalResponse::REJECT;
        }
      };

    auto force_step_handle_cancel = [this](
      const std::shared_ptr<GoalHandleForce>)
      {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel force step goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto force_step_handle_accepted = [this](
      const std::shared_ptr<GoalHandleForce> goal_handle)
      {
        // set cmd state
        cmd_state_ = CmdState::BEGIN;

        // init message attempts to 0 to be safe
        msg_attempts_ = 0;

        // start timer for action
        action_timer_ = create_wall_timer(100ms, [this, goal_handle](){
              return this->execute_force_step_goal(goal_handle);
          },
        timer_cb_group_);
      };

    force_step_action_server_ = rclcpp_action::create_server<finger_interfaces::action::Force>(
    this,
    "/force_step_move",
    force_step_handle_goal,
    force_step_handle_cancel,
    force_step_handle_accepted,
    rcl_action_server_get_default_options(),
    action_cb_group_);
  }

private:
  const double ra_, rb_, rc_;
  const double r1_, r3_, r5_, r7_, r9_, r11_;
  const arma::mat Ra_;
  const arma::mat St_;
  const std::vector<arma::vec6> slist_;
  const arma::vec joint_min_;
  const arma::vec joint_max_;
  const arma::mat44 M_;
  const std::vector<double> four_bar_lengths_;

  int msg_attempts_;

  CmdState cmd_state_;
  rclcpp::Subscription<finger_interfaces::msg::MotorFeedback>::SharedPtr motor_actual_feedback_sub_;
  rclcpp::Subscription<finger_interfaces::msg::MotorFeedback>::SharedPtr
    motor_setpoint_feedback_sub_;
  rclcpp::Subscription<finger_interfaces::msg::MotorActivity>::SharedPtr
    motor_activity_feedback_sub_;
  rclcpp::Client<finger_interfaces::srv::SendCommand>::SharedPtr send_client_;
  rclcpp::Client<finger_interfaces::srv::StartStopCommand>::SharedPtr start_client_;
  rclcpp::Client<finger_interfaces::srv::StartStopCommand>::SharedPtr stop_client_;
  rclcpp::CallbackGroup::SharedPtr send_cb_group_;
  rclcpp::CallbackGroup::SharedPtr start_cb_group_;
  rclcpp::CallbackGroup::SharedPtr stop_cb_group_;
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp_action::Server<finger_interfaces::action::Cartesian>::SharedPtr cartesian_action_server_;
  rclcpp_action::Server<finger_interfaces::action::Sinusoidal>::SharedPtr sinusoidal_action_server_;
  rclcpp_action::Server<finger_interfaces::action::Linear>::SharedPtr linear_action_server_;
  rclcpp_action::Server<finger_interfaces::action::Force>::SharedPtr force_step_action_server_;
  rclcpp::TimerBase::SharedPtr action_timer_;
  std::shared_ptr<Transformer> transforms_;
  std::shared_ptr<JointTrajectory> generator_;
  std::shared_ptr<finger_interfaces::action::Cartesian::Result> cartesian_result_;
  std::shared_ptr<finger_interfaces::action::Sinusoidal::Result> sinusoidal_result_;
  std::shared_ptr<finger_interfaces::action::Linear::Result> linear_result_;
  std::shared_ptr<finger_interfaces::action::Force::Result> force_step_result_;
  finger_interfaces::msg::MotorFeedback motor_actual_feedback_;
  finger_interfaces::msg::MotorFeedback motor_setpoint_feedback_;
  finger_interfaces::msg::MotorActivity motor_activity_feedback_;

  double max_vel_;
  double max_accel_;
  double gnd_height_;
  std::vector<arma::vec> q_motor_list_;

  template<typename ResultT, typename FutureT>
  void handle_service_response(
    FutureT future,
    std::shared_ptr<ResultT> result,
    std::function<void(std::shared_ptr<ResultT>)> abort_fn,
    CmdState success_state,
    CmdState retry_state,
    const char * failure_msg,
    std::function<void(std::shared_ptr<ResultT>)> on_success = nullptr)
  {
    if (future.get().second->success == 1) {
      // if there is a success function, call it
      if (on_success) {on_success(result);}

      // update state to next case
      cmd_state_ = success_state;

      // reset message attempts count
      msg_attempts_ = 0;
    } else {
      // increment message attempts
      msg_attempts_++;

      // if 5 or more failed attempts
      if (msg_attempts_ >= 5) {
        RCLCPP_INFO_STREAM(get_logger(), failure_msg);

        // assign failure to result
        result->success = 0;

        // abort action
        abort_fn(result);

        // update state to stop timer
        cmd_state_ = CmdState::STOPPING;
      } else {
        // if more attempts left, reset state to call service again
        cmd_state_ = retry_state;
      }
    }
  }

  template<typename ActionT>
  void execute_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle,
    std::shared_ptr<typename ActionT::Result> & result,
    std::function<std::vector<arma::vec>(const typename ActionT::Goal &)> generate_traj,
    int repeat = 0,
    char mode = 'P')
  {
    if (cmd_state_ == CmdState::IDLE) {
      RCLCPP_INFO_ONCE(get_logger(), "idling...");

    } else if (cmd_state_ == CmdState::STOPPING) {
      RCLCPP_INFO(get_logger(), "Stopping action...");
      action_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Action stopped.");

    } else if (cmd_state_ == CmdState::BEGIN) {
      RCLCPP_INFO(get_logger(), "Sending data");

      result = std::make_shared<typename ActionT::Result>();
      result->success = 1;

      // generate motor trajectory
      try {
        q_motor_list_ = generate_traj(*goal_handle->get_goal());
      } catch (std::runtime_error & e) {
        RCLCPP_INFO_STREAM(get_logger(), "Failed to generate motor trajectory.");
        result->success = 0;
        goal_handle->abort(result);
        cmd_state_ = CmdState::STOPPING;
        return;
      }

      // build and send the SendCommand request
      auto rq = std::make_shared<finger_interfaces::srv::SendCommand::Request>();
      rq->length = int(q_motor_list_.size());
      rq->repeat = repeat;
      rq->mode = mode;
      for (auto & q : q_motor_list_) {
        rq->mcp_splay.push_back(q[0]);
        rq->mcp_flex.push_back(q[1]);
        rq->pip_flex.push_back(q[2]);
      }

      // send data
      send_client_->async_send_request(rq,
        [this, goal_handle, result](
          rclcpp::Client<finger_interfaces::srv::SendCommand>::SharedFutureWithRequest future) {
          handle_service_response<typename ActionT::Result>(
              future, result,
            [goal_handle](auto r){goal_handle->abort(r);},
              CmdState::RECEIVED, CmdState::BEGIN,
              "Failed to send 'send' message.");
          });

      cmd_state_ = CmdState::IDLE;

    } else if (cmd_state_ == CmdState::RECEIVED) {
      RCLCPP_INFO(get_logger(), "Starting movement");

      // send start command
      auto rq = std::make_shared<finger_interfaces::srv::StartStopCommand::Request>();
      auto result_future = start_client_->async_send_request(rq,
          [this, goal_handle, result](
            rclcpp::Client<finger_interfaces::srv::StartStopCommand>::SharedFutureWithRequest future)
          {
            handle_service_response<typename ActionT::Result>(
              future, result,
              [goal_handle](auto r){goal_handle->abort(r);},
              CmdState::STARTED, CmdState::RECEIVED,
              "Failed to send 'start' message.");
          });

      cmd_state_ = CmdState::IDLE;


    } else if (cmd_state_ == CmdState::STARTED) {
      RCLCPP_INFO_ONCE(get_logger(), "Working...");

      if (motor_activity_feedback_.active < 1e-4 && motor_activity_feedback_.active > -1e-4) {
        // move to stopping once movement complete
        cmd_state_ = CmdState::STOPPING;

        // assign result to handle
        goal_handle->succeed(result);

      }

    } else if (cmd_state_ == CmdState::CANCELLED) {

      // send stop command
      auto rq = std::make_shared<finger_interfaces::srv::StartStopCommand::Request>();
      auto result_future = stop_client_->async_send_request(rq,
          [this, goal_handle, result](
            rclcpp::Client<finger_interfaces::srv::StartStopCommand>::SharedFutureWithRequest future)
          {
            handle_service_response<typename ActionT::Result>(
              future, result,
              [goal_handle](auto r){goal_handle->abort(r);},
              CmdState::STOPPING, CmdState::CANCELLED,
              "Failed to send 'stop' message.",
              [goal_handle, result](auto r){result->success = 0; goal_handle->canceled(r);});
          });

      cmd_state_ = CmdState::IDLE;
    }

    if (goal_handle->is_canceling()) {
      if (cmd_state_ != CmdState::STOPPING && cmd_state_ != CmdState::IDLE) {
        cmd_state_ = CmdState::CANCELLED;
      }
    }
  }

  void execute_cartesian_goal(const std::shared_ptr<GoalHandleCartesian> goal_handle)
  {
    execute_goal<finger_interfaces::action::Cartesian>(
      goal_handle, cartesian_result_,
      [this](const auto & goal) {
        // check that requested waypoints are same length
        if ((goal.length == int(goal.x.size())) && (goal.length == int(goal.y.size())) &&
        (goal.length == int(goal.z.size())))
        {
          // save waypoints as vector
          auto waypoints_temp = std::vector<arma::vec>();
          for (auto i = 0; i < goal.length; i++) {
            waypoints_temp.push_back({goal.x.at(i), goal.y.at(i), goal.z.at(i)});
          }

          RCLCPP_INFO(get_logger(), "Received cartesian goal request with length %d and waypoints:",
          goal.length);

          // check that waypoints are within the joint limits

          RCLCPP_INFO_STREAM(get_logger(),
          "waypoint 0: (" << goal.x.at(0) << ", " << goal.y.at(0) << ", " << goal.z.at(0) <<
            ")");
          for(auto i = 1; i < goal.length; i++) {
            RCLCPP_INFO_STREAM(get_logger(),
            "waypoint " << i << ": (" << goal.x.at(i) << ", " << goal.y.at(i) << ", " <<
              goal.z.at(i) << ")");

            // check that waypoints are within workspace
            auto start = transforms_->end_effector_to_joint(waypoints_temp[i - 1]);
            auto end = transforms_->end_effector_to_joint(waypoints_temp[i]);
          }

          // accept request
          return generator_->generate_cartesian(waypoints_temp, max_vel_, max_accel_);

        } else {
          RCLCPP_INFO(get_logger(), "Goal request REJECTED because waypoints are malformed.");

          // reject request
          throw std::runtime_error("Waypoints malformed.");
        }
      },
      0,
      'P');
  }

  void execute_linear_goal(const std::shared_ptr<GoalHandleLinear> goal_handle)
  {
    execute_goal<finger_interfaces::action::Linear>(
      goal_handle, linear_result_,
      [this](const auto & goal) {
        RCLCPP_INFO_STREAM(get_logger(),
        "start: (" << goal.start.at(0) << ", " << goal.start.at(1) << ", " << goal.start.at(2) <<
          ")");
        RCLCPP_INFO_STREAM(get_logger(),
        "end: (" << goal.end.at(0) << ", " << goal.end.at(1) << ", " << goal.end.at(2) << ")");

        arma::vec start_vec = {goal.start.at(0), goal.start.at(1), goal.start.at(2)};
        arma::vec end_vec = {goal.end.at(0), goal.end.at(1), goal.end.at(2)};

        // check that start and end points are within workspace
        auto start = transforms_->joint_to_end_effector(start_vec);
        auto end = transforms_->joint_to_end_effector(end_vec);

        return generator_->generate_linear(start_vec, end_vec, max_vel_, max_accel_);
      },
      0,
      'P');
  }

  void execute_sinusoidal_goal(const std::shared_ptr<GoalHandleSinusoidal> goal_handle)
  {
    execute_goal<finger_interfaces::action::Sinusoidal>(
      goal_handle, sinusoidal_result_,
      [this](const auto & goal) {
        if (((goal.joint == 0) || (goal.joint == 1) || (goal.joint == 2))) {
          RCLCPP_INFO_STREAM(get_logger(), "Received sinusoidal goal request for joint" <<
            goal.joint << " with amp " << goal.amp << ", freq " << goal.freq <<
            ", and v_shift " << goal.v_shift);

          // accept request
          return generator_->generate_sinusoid(goal.joint, goal.amp, goal.freq, goal.v_shift);
        } else {
          RCLCPP_INFO(get_logger(), "Goal request REJECTED because joint is not 0, 1, or 2.");

          // reject
          throw std::runtime_error("Joint is malformed.");
        }
      },
      goal_handle->get_goal()->repeat,
      'P');
  }

  void execute_force_step_goal(const std::shared_ptr<GoalHandleForce> goal_handle)
  {
    execute_goal<finger_interfaces::action::Force>(
      goal_handle, force_step_result_,
      [this](const auto & goal) {
        
        RCLCPP_INFO_STREAM(get_logger(), "Received force step goal request at\njoint state <" <<
        goal.q_joint.at(0) << ", "<< goal.q_joint.at(1) << ", " << goal.q_joint.at(2) << ">\nforce_low <" <<
        goal.force_low.at(0) << ", "<< goal.force_low.at(1) << ", " << goal.force_low.at(2) << ">\nforce_high <" <<
        goal.force_high.at(0) << ", "<< goal.force_high.at(1) << ", " << goal.force_high.at(2) << ">\nfrequency " << goal.frequency);

        // accept request
        arma::vec q_joint = {goal.q_joint.at(0), goal.q_joint.at(1), goal.q_joint.at(2)};
        arma::vec force_low = {goal.force_low.at(0), goal.force_low.at(1), goal.force_low.at(2)};
        arma::vec force_high = {goal.force_high.at(0), goal.force_high.at(1), goal.force_high.at(2)};

        return generator_->generate_force_step(q_joint, force_low, force_high, goal.frequency);
      },
      goal_handle->get_goal()->repeat,
      'T');
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FingerPlanner>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
