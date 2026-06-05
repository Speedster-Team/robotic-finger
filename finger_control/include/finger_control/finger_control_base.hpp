/// \file
/// \brief Base class for all finger control nodes. Handles Drake heartbeat
///        synchronization, finger kinematic parameter loading, motor feedback
///        subscription, and provides blocking send_*_goal helpers for all
///        trajectory action types (cartesian, linear, linear step, sinusoidal,
///        force step, chirp, chirp velocity). Subclass and call helpers from the constructor.
#ifndef FINGER_CONTROL__FINGER_CONTROL_BASE_HPP_
#define FINGER_CONTROL__FINGER_CONTROL_BASE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/empty.hpp"

#include "finger_interfaces/action/cartesian.hpp"
#include "finger_interfaces/action/sinusoidal.hpp"
#include "finger_interfaces/action/linear.hpp"
#include "finger_interfaces/action/linear_step.hpp"
#include "finger_interfaces/action/force.hpp"
#include "finger_interfaces/action/chirp.hpp"
#include "finger_interfaces/action/chirp_velocity.hpp"
#include "finger_interfaces/msg/motor_feedback.hpp"

#include "fingerlib/joint_trajectory.hpp"

using namespace std::chrono_literals;

class FingerControlBase : public rclcpp::Node
{
public:
  using Cartesian = finger_interfaces::action::Cartesian;
  using Sinusoidal = finger_interfaces::action::Sinusoidal;
  using Linear = finger_interfaces::action::Linear;
  using LinearStep = finger_interfaces::action::LinearStep;
  using Force = finger_interfaces::action::Force;
  using Chirp = finger_interfaces::action::Chirp;
  using ChirpVelocity = finger_interfaces::action::ChirpVelocity;
  using GoalHandleCartesian = rclcpp_action::ClientGoalHandle<Cartesian>;
  using GoalHandleSinusoidal = rclcpp_action::ClientGoalHandle<Sinusoidal>;
  using GoalHandleLinear = rclcpp_action::ClientGoalHandle<Linear>;
  using GoalHandleLinearStep = rclcpp_action::ClientGoalHandle<LinearStep>;
  using GoalHandleForce = rclcpp_action::ClientGoalHandle<Force>;
  using GoalHandleChirp = rclcpp_action::ClientGoalHandle<Chirp>;
  using GoalHandleChirpVelocity = rclcpp_action::ClientGoalHandle<ChirpVelocity>;

  explicit FingerControlBase(const std::string & node_name)
  : Node(node_name)
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "The relative height of the ground (should be negative).";
    declare_parameter("relative_gnd_height", -0.25, param_desc);
    gnd_height_ = get_parameter("relative_gnd_height").as_double();

    declare_and_get_finger_params();

    transforms_ = std::make_shared<Transformer>(Ra_, St_, slist_, M_, four_bar_lengths_,
      joint_min_, joint_max_);
    generator_ = std::make_shared<JointTrajectory>(*transforms_, 100, gnd_height_);

    wait_for_drake_heartbeat();

    cartesian_client_ = rclcpp_action::create_client<Cartesian>(this, "/cartesian_move");
    sinusoidal_client_ = rclcpp_action::create_client<Sinusoidal>(this, "/sinusoidal_move");
    linear_client_ = rclcpp_action::create_client<Linear>(this, "/linear_move");
    linear_step_client_ = rclcpp_action::create_client<LinearStep>(this, "/linear_step_move");
    force_step_client_ = rclcpp_action::create_client<Force>(this, "/force_step_move");
    chirp_client_ = rclcpp_action::create_client<Chirp>(this, "/chirp_move");
    chirp_velocity_client_ = rclcpp_action::create_client<ChirpVelocity>(this, "/chirp_velocity_move");

    while (!cartesian_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for cartesian_move action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for cartesian_move action to appear...");
    }
    while (!sinusoidal_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for sinusoidal_move action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for sinusoidal_move action to appear...");
    }
    while (!linear_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for linear_move action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for linear_move action to appear...");
    }
    while (!linear_step_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for linear_step_move action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for linear_move action to appear...");
    }
    while (!force_step_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for force_step action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for force_step action to appear...");
    }
    while (!chirp_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for chirp action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for chirp action to appear...");
    }
    while (!chirp_velocity_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for chirp velocity action to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for chirp_velocity action to appear...");
    }

    motor_actual_feedback_sub_ = create_subscription<finger_interfaces::msg::MotorFeedback>(
      "/motor_pos_actual_feedback", 10,
      [this](finger_interfaces::msg::MotorFeedback::UniquePtr msg) {
        motor_actual_feedback_ = *msg;
      });

    RCLCPP_INFO(get_logger(), "Waiting for motor feedback...");
    while (motor_actual_feedback_.motor_positions.size() < 3) {
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(100ms);
    }
    RCLCPP_INFO(get_logger(), "Motor feedback received, proceeding.");
  
  }

protected:
  double ra_, rb_, rc_;
  double r1_, r3_, r5_, r7_, r9_, r11_;
  arma::mat Ra_;
  arma::mat St_;
  std::vector<arma::vec6> slist_;
  arma::vec joint_min_;
  arma::vec joint_max_;
  arma::mat44 M_;
  std::vector<double> four_bar_lengths_;
  double gnd_height_;
  std::shared_ptr<Transformer> transforms_;
  std::shared_ptr<JointTrajectory> generator_;
  rclcpp_action::Client<Cartesian>::SharedPtr cartesian_client_;
  rclcpp_action::Client<Sinusoidal>::SharedPtr sinusoidal_client_;
  rclcpp_action::Client<Linear>::SharedPtr linear_client_;
  rclcpp_action::Client<LinearStep>::SharedPtr linear_step_client_;
  rclcpp_action::Client<Force>::SharedPtr force_step_client_;
  rclcpp_action::Client<Chirp>::SharedPtr chirp_client_;
  rclcpp_action::Client<ChirpVelocity>::SharedPtr chirp_velocity_client_;
  rclcpp::Subscription<finger_interfaces::msg::MotorFeedback>::SharedPtr motor_actual_feedback_sub_;
  finger_interfaces::msg::MotorFeedback motor_actual_feedback_;

  void declare_and_get_finger_params()
  {
    declare_parameter("ra", 0.0);
    declare_parameter("rb", 0.0);
    declare_parameter("rc", 0.0);
    declare_parameter("r1", 0.0);
    declare_parameter("r3", 0.0);
    declare_parameter("r5", 0.0);
    declare_parameter("r7", 0.0);
    declare_parameter("r9", 0.0);
    declare_parameter("r11", 0.0);
    declare_parameter("slist", std::vector<double>{
        0, 0, 1, 0, 0, 0,
        -1, 0, 0, 0, 0, 0.0178,
        -1, 0, 0, 0, 0, 0.079,
        -1, 0, 0, 0, 0, 0.1195});
    declare_parameter("joint_min", std::vector<double>{-0.55, -0.2, -0.01});
    declare_parameter("joint_max", std::vector<double>{0.55, 1.572, 1.572});
    declare_parameter("M", std::vector<double>{
        1, 0, 0, 0,
        0, 1, 0, 0.15,
        0, 0, 1, 0,
        0, 0, 0, 1});
    declare_parameter("four_bar_lengths", std::vector<double>{
        8.83765e-3, 40.6e-3, 8.91536e-3, 37.79903e-3});

    ra_ = get_parameter("ra").as_double();
    rb_ = get_parameter("rb").as_double();
    rc_ = get_parameter("rc").as_double();
    r1_ = get_parameter("r1").as_double();
    r3_ = get_parameter("r3").as_double();
    r5_ = get_parameter("r5").as_double();
    r7_ = get_parameter("r7").as_double();
    r9_ = get_parameter("r9").as_double();
    r11_ = get_parameter("r11").as_double();
    Ra_ = arma::diagmat(arma::vec3{ra_, rb_, rc_});
    St_ = arma::mat{{r11_, r3_, r1_}, {0.0, r7_, r5_}, {0.0, 0.0, r9_}};

    auto slist_flat = get_parameter("slist").as_double_array();
    slist_ = {
        arma::vec6(slist_flat.data()),
        arma::vec6(slist_flat.data() + 6),
        arma::vec6(slist_flat.data() + 12),
        arma::vec6(slist_flat.data() + 18)};

    auto jmin_flat = get_parameter("joint_min").as_double_array();
    joint_min_ = arma::vec(jmin_flat);

    auto jmax_flat = get_parameter("joint_max").as_double_array();
    joint_max_ = arma::vec(jmax_flat);

    // add a small buffer so that planning doesn't fail when the finger is at a joint limit
    for (int i = 0; i < int(joint_max_.size()); i++) {
      joint_min_(i) -= 0.05;
      joint_max_(i) += 0.05;
    }

    auto M_flat = get_parameter("M").as_double_array();
    M_ = arma::mat44(arma::mat(M_flat.data(), 4, 4).t());

    auto fbl_flat = get_parameter("four_bar_lengths").as_double_array();
    four_bar_lengths_ = std::vector<double>(fbl_flat.begin(), fbl_flat.end());
  }

  void send_cartesian_goal(bool repeat, std::vector<std::vector<float>> waypoints)
  {
    auto goal_msg = Cartesian::Goal();
    
    goal_msg.length = int(waypoints.size());
    goal_msg.repeat = int(repeat);

    // check if they provided more than just the end
    if (goal_msg.length > 1) {
      for (auto & wp : waypoints) {
        goal_msg.x.push_back(wp.at(0));
        goal_msg.y.push_back(wp.at(1));
        goal_msg.z.push_back(wp.at(2));
      }
    }
    else {
      // derive current position in cartesian coordinates

      arma::vec motor_pos = {
        motor_actual_feedback_.motor_positions.at(0),
        motor_actual_feedback_.motor_positions.at(1),
        motor_actual_feedback_.motor_positions.at(2)};

      arma::vec joint_pos = transforms_->motor_to_joint(motor_pos);
      arma::mat44 cartesian = transforms_->joint_to_end_effector(joint_pos);
      RCLCPP_INFO_STREAM(get_logger(), "motor_pos: " << motor_pos.at(0) << " " << motor_pos.at(1) << " " << motor_pos.at(2));
      RCLCPP_INFO_STREAM(get_logger(), "joint_pos: " << joint_pos(0) << " " << joint_pos(1) << " " << joint_pos(2));
      RCLCPP_INFO_STREAM(get_logger(), "starting cartesian point: " << cartesian(0, 3) << " " << cartesian(1, 3) << " " << cartesian(2, 3));
      // add the current position in cartesian coordinates
      goal_msg.x.push_back(cartesian(0, 3));
      goal_msg.y.push_back(cartesian(1, 3));
      goal_msg.z.push_back(cartesian(2, 3));

      // then add the last point
      for (auto & wp : waypoints) {
        goal_msg.x.push_back(wp.at(0));
        goal_msg.y.push_back(wp.at(1));
        goal_msg.z.push_back(wp.at(2));
      }

      goal_msg.length = int(waypoints.size() + 1);

    }    

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Cartesian>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandleCartesian::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](
      GoalHandleCartesian::SharedPtr,
      const std::shared_ptr<const Cartesian::Feedback>) {
        RCLCPP_INFO(get_logger(), "feedback received...");
      };
    send_goal_options.result_callback = [this](const GoalHandleCartesian::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted"); return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled"); return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code"); return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = cartesian_client_->async_send_goal(goal_msg, send_goal_options);
    spin_until_complete(goal_handle_future);
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_INFO(get_logger(), "Goal rejected"); return;
    }
    auto result_future = cartesian_client_->async_get_result(goal_handle);
    spin_until_complete(result_future);
    RCLCPP_INFO(get_logger(), "Goal completed");
  }

  void send_sinusoid_goal(bool repeat, int joint, float amp, float freq, float v_shift)
  {
    auto goal_msg = Sinusoidal::Goal();
    goal_msg.repeat = int(repeat);
    goal_msg.joint = joint;
    goal_msg.amp = amp;
    goal_msg.freq = freq;
    goal_msg.v_shift = v_shift;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Sinusoidal>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleSinusoidal::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](
      GoalHandleSinusoidal::SharedPtr,
      const std::shared_ptr<const Sinusoidal::Feedback>) {
        RCLCPP_INFO(get_logger(), "feedback received...");
      };
    send_goal_options.result_callback = [this](const GoalHandleSinusoidal::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted"); return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled"); return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code"); return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = sinusoidal_client_->async_send_goal(goal_msg, send_goal_options);
    spin_until_complete(goal_handle_future);
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_INFO(get_logger(), "Goal rejected"); return;
    }
    auto result_future = sinusoidal_client_->async_get_result(goal_handle);
    spin_until_complete(result_future);
    RCLCPP_INFO(get_logger(), "Goal completed");
  }

  void send_linear_goal(bool repeat, std::vector<std::vector<float>> waypoints)
  {
    auto goal_msg = Linear::Goal();

    goal_msg.length = int(waypoints.size());
    goal_msg.repeat = int(repeat);

    // check if they provided more than just the end
    if (goal_msg.length > 1) {
      for (auto & wp : waypoints) {
        goal_msg.splay.push_back(wp.at(0));
        goal_msg.mcp.push_back(wp.at(1));
        goal_msg.pipdip.push_back(wp.at(2));
      }
    }
    else {
      // derive current position in cartesian coordinates

      arma::vec motor_pos = {
        motor_actual_feedback_.motor_positions.at(0),
        motor_actual_feedback_.motor_positions.at(1),
        motor_actual_feedback_.motor_positions.at(2)};

      arma::vec joint_pos = transforms_->motor_to_joint(motor_pos);
      RCLCPP_INFO_STREAM(get_logger(), "motor_pos: " << motor_pos.at(0) << " " << motor_pos.at(1) << " " << motor_pos.at(2));
      RCLCPP_INFO_STREAM(get_logger(), "starting joint_pos: " << joint_pos(0) << " " << joint_pos(1) << " " << joint_pos(2));
      // add the current position in cartesian coordinates
      goal_msg.splay.push_back(joint_pos(0));
      goal_msg.mcp.push_back(joint_pos(1));
      goal_msg.pipdip.push_back(joint_pos(2));

      // then add the last point
      for (auto & wp : waypoints) {
        goal_msg.splay.push_back(wp.at(0));
        goal_msg.mcp.push_back(wp.at(1));
        goal_msg.pipdip.push_back(wp.at(2));
      }

      goal_msg.length = int(waypoints.size() + 1);
    }    

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Linear>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      [this](const GoalHandleLinear::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](
      GoalHandleLinear::SharedPtr,
      const std::shared_ptr<const Linear::Feedback>) {
        RCLCPP_INFO(get_logger(), "feedback received...");
      };
    send_goal_options.result_callback = [this](const GoalHandleLinear::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted"); return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled"); return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code"); return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };


    auto goal_handle_future = linear_client_->async_send_goal(goal_msg, send_goal_options);
    spin_until_complete(goal_handle_future);
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_INFO(get_logger(), "Goal rejected"); return;
    }
    auto result_future = linear_client_->async_get_result(goal_handle);
    spin_until_complete(result_future);
    RCLCPP_INFO(get_logger(), "Goal completed");
  }

  void send_linear_step_goal(bool repeat, double freq, std::vector<std::vector<float>> waypoints)
  {
    auto goal_msg = LinearStep::Goal();

    goal_msg.length = int(waypoints.size());
    goal_msg.repeat = int(repeat);
    goal_msg.freq = freq;

    for (auto & wp : waypoints) {
      goal_msg.splay.push_back(wp.at(0));
      goal_msg.mcp.push_back(wp.at(1));
      goal_msg.pipdip.push_back(wp.at(2));
    }
  
    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<LinearStep>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      [this](const GoalHandleLinearStep::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](
      GoalHandleLinearStep::SharedPtr,
      const std::shared_ptr<const LinearStep::Feedback>) {
        RCLCPP_INFO(get_logger(), "feedback received...");
      };
    send_goal_options.result_callback = [this](const GoalHandleLinearStep::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted"); return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled"); return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code"); return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = linear_step_client_->async_send_goal(goal_msg, send_goal_options);
    spin_until_complete(goal_handle_future);
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_INFO(get_logger(), "Goal rejected"); return;
    }
    auto result_future = linear_step_client_->async_get_result(goal_handle);
    spin_until_complete(result_future);
    RCLCPP_INFO(get_logger(), "Goal completed");
  }


  void send_force_step_goal(
    std::vector<float> joint_state,
    std::vector<float> force_low,
    std::vector<float> force_high,
    double frequency,
    bool repeat)
  {

    auto goal_msg = Force::Goal();
    goal_msg.q_joint = joint_state;
    goal_msg.force_low = force_low;
    goal_msg.force_high = force_high;
    goal_msg.frequency = frequency;
    goal_msg.repeat = int(repeat);

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Force>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleForce::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](
      GoalHandleForce::SharedPtr,
      const std::shared_ptr<const Force::Feedback>) {
        RCLCPP_INFO(get_logger(), "feedback received...");
      };
    send_goal_options.result_callback = [this](const GoalHandleForce::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted"); return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled"); return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code"); return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = force_step_client_->async_send_goal(goal_msg, send_goal_options);
    spin_until_complete(goal_handle_future);
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_INFO(get_logger(), "Goal rejected"); return;
    }
    auto result_future = force_step_client_->async_get_result(goal_handle);
    spin_until_complete(result_future);
    RCLCPP_INFO(get_logger(), "Goal completed");
  }

  void send_chirp_goal(int joint, float amp, float freq_init, float freq_final, float time, float v_shift)
  {
    auto goal_msg = Chirp::Goal();
    goal_msg.joint = joint;
    goal_msg.amp = amp;
    goal_msg.freq_init = freq_init;
    goal_msg.freq_final = freq_final;
    goal_msg.time = time;
    goal_msg.v_shift = v_shift;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Chirp>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleChirp::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](
      GoalHandleChirp::SharedPtr,
      const std::shared_ptr<const Chirp::Feedback>) {
        RCLCPP_INFO(get_logger(), "feedback received...");
      };
    send_goal_options.result_callback = [this](const GoalHandleChirp::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted"); return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled"); return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code"); return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = chirp_client_->async_send_goal(goal_msg, send_goal_options);
    spin_until_complete(goal_handle_future);
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_INFO(get_logger(), "Goal rejected"); return;
    }
    auto result_future = chirp_client_->async_get_result(goal_handle);
    spin_until_complete(result_future);
    RCLCPP_INFO(get_logger(), "Goal completed");
  }

  void send_chirp_velocity_goal(int joint, float amp, float freq_init, float freq_final, float time, std::vector<float> start_pos)
  {
    // send the start pos
    std::vector<float> zero_pos = {0, 0, 0};
    send_linear_goal(0, {start_pos});
    
    auto goal_msg = ChirpVelocity::Goal();
    goal_msg.joint = joint;
    goal_msg.amp = amp;
    goal_msg.freq_init = freq_init;
    goal_msg.freq_final = freq_final;
    goal_msg.time = time;
    goal_msg.start_pos = start_pos;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<ChirpVelocity>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleChirpVelocity::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    send_goal_options.feedback_callback = [this](
      GoalHandleChirpVelocity::SharedPtr,
      const std::shared_ptr<const ChirpVelocity::Feedback>) {
        RCLCPP_INFO(get_logger(), "feedback received...");
      };
    send_goal_options.result_callback = [this](const GoalHandleChirpVelocity::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was aborted"); return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(get_logger(), "Goal was canceled"); return;
          default:
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code"); return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "result code: " << result.result.get()->success);
      };

    auto goal_handle_future = chirp_velocity_client_->async_send_goal(goal_msg, send_goal_options);
    spin_until_complete(goal_handle_future);
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_INFO(get_logger(), "Goal rejected"); return;
    }
    auto result_future = chirp_velocity_client_->async_get_result(goal_handle);
    spin_until_complete(result_future);
    RCLCPP_INFO(get_logger(), "Goal completed");
  }

private:
  template<typename FutureT>
  void spin_until_complete(FutureT & future)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    try {
      exec.add_node(get_node_base_interface());
      exec.spin_until_future_complete(future);
    } catch (const std::runtime_error &) {
      // Node is already owned by a running executor (e.g. MultiThreadedExecutor in main).
      // That executor's other threads will deliver callbacks — just block on the future directly.
      future.wait();
    }
  }

  void wait_for_drake_heartbeat()
  {
    auto heartbeat_client = create_client<std_srvs::srv::Empty>("/heartbeat", 10);
    while (!heartbeat_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "client interrupted while waiting for service to appear.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "waiting for heartbeat service to appear...");
    }
  }
};

#endif  // FINGER_CONTROL__FINGER_CONTROL_BASE_HPP_
