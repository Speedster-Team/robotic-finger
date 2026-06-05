/// \file
/// \brief Converts whack-a-mole target pixel coordinates into a 3D goal TF frame.
///        Subscribes to the whackamole server's target and hit topics, transforms
///        pixel coordinates to metric space using a configurable phone camera TF,
///        and republishes the goal as a visualization marker and TF at 10 Hz.
///        State machine: NONE → NEW (on target) → PUBLISHING → NONE (on hit).
///
/// PARAMETERS:
///   + tf.trans.x / .y / .z (double) - Phone frame translation relative to base_frame (m)
///   + tf.rot.roll / .pitch / .yaw (double) - Phone frame rotation relative to base_frame (rad)
///   + px_per_meter (double) - Pixel density of the phone camera image (px/m)
///
/// SUBSCRIBERS:
///   + /whackamole/target (whackamole_interfaces/msg/Point) - Pixel coordinates of the new target from the whackamole server
///   + /whackamole/hit_ms (std_msgs/msg/Int32) - Hit confirmation signal; resets goal state
///
/// PUBLISHERS:
///   + /goals (visualization_msgs/msg/Marker) - Sphere marker at the current goal position in phone_frame
///
/// TF BROADCASTERS:
///   + phone_frame → base_frame (static) - Camera-to-robot calibration transform
///   + phone_frame → goal (dynamic)      - Current goal position in phone_frame

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <armadillo>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/int32.hpp"
#include "fingerlib/transformer.hpp"
#include "whackamole_interfaces/msg/point.hpp"

using namespace std::chrono_literals;

std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};

    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

/// \brief State of target
enum TargetState {
    NEW,
    PUBLISHING,
    NONE
};

/// \brief Publishes random points for the finger to go towards
class PixelVision : public rclcpp::Node
{
public:
  PixelVision()
  : Node("pixel_vision"),
    id_(0),
    has_marker_(false),
    count_(0),
    target_state_ (TargetState::NONE)
  {
    // declare and get parameters
    declare_parameter("x_offset", 0.006);
    declare_parameter("tf.trans.x", 0.0);
    declare_parameter("tf.trans.y", 0.0);
    declare_parameter("tf.trans.z", 0.0);
    declare_parameter("tf.rot.roll", 0.0);
    declare_parameter("tf.rot.pitch", 0.0);
    declare_parameter("tf.rot.yaw", 0.0);
    declare_parameter("px_per_meter", 0.0);
    declare_parameter("fingertip_radius", 0.0);

    phone_tf_.header.stamp = now();
    phone_tf_.header.frame_id = "base_frame";
    phone_tf_.child_frame_id = "phone_frame";
    phone_tf_.transform.translation.x = get_parameter("tf.trans.x").as_double();
    phone_tf_.transform.translation.y = get_parameter("tf.trans.y").as_double();
    phone_tf_.transform.translation.z = get_parameter("tf.trans.z").as_double();
    tf2::Quaternion q;
    q.setRPY(get_parameter("tf.rot.roll").as_double(), get_parameter("tf.rot.pitch").as_double(), get_parameter("tf.rot.yaw").as_double());
    phone_tf_.transform.rotation.x = q.x();
    phone_tf_.transform.rotation.y = q.y();
    phone_tf_.transform.rotation.z = q.z();
    phone_tf_.transform.rotation.w = q.w();
    pixels_per_meter_ = get_parameter("px_per_meter").as_double();

    // screen_width=402 screen_length=714
    phone_corner_tf_.header.stamp = now();
    phone_corner_tf_.header.frame_id = "phone_frame";
    phone_corner_tf_.child_frame_id = "phone_corner";
    phone_corner_tf_.transform.translation.x = (402.0 / 2.0) / pixels_per_meter_;
    phone_corner_tf_.transform.translation.y = ((714.0 / 2.0) / pixels_per_meter_) +  get_parameter("x_offset").as_double();
    phone_corner_tf_.transform.translation.z = -get_parameter("fingertip_radius").as_double();
    q.setRPY(0.0, 0.0, 3.14);
    phone_corner_tf_.transform.rotation.x = q.x();
    phone_corner_tf_.transform.rotation.y = q.y();
    phone_corner_tf_.transform.rotation.z = q.z();
    phone_corner_tf_.transform.rotation.w = q.w();
    // create static transformer for phone transform and publish
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_static_broadcaster_->sendTransform(phone_tf_);
    tf_static_broadcaster_->sendTransform(phone_corner_tf_);

    // create publishers
    auto qos = rclcpp::QoS(10).transient_local();
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/goals", qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create subscribers
    auto target_feedback_sub_cb_ =
      [this](whackamole_interfaces::msg::Point::UniquePtr msg) -> void {
        // save feedback
        target_ = *msg;
        target_state_ = TargetState::NEW;
      };
    auto hit_feedback_sub_cb_ =
      [this](std_msgs::msg::Int32::UniquePtr) -> void {
        // save feedback
        target_state_ = TargetState::NONE;
      };


    target_feedback_sub_ = create_subscription<whackamole_interfaces::msg::Point>("/whackamole/target", 10, target_feedback_sub_cb_);
    hit_feedback_sub_ = create_subscription<std_msgs::msg::Int32>("/whackamole/hit_ms", 10, hit_feedback_sub_cb_);

    auto timer_callback =
      [this]()-> void {
        
        switch (target_state_) {

            case TargetState::NEW:
            
                // update stored marker from new detection
                updateMarker();

                // update state
                target_state_ = TargetState::PUBLISHING;
                break;
            case TargetState::PUBLISHING:

                // republish current marker and goal tf until the next detection arrives
                marker_.header.stamp = now();
                marker_pub_->publish(marker_);
                goal_tf_.header.stamp = now();
                tf_broadcaster_->sendTransform(goal_tf_);
                break;

            case TargetState::NONE:
                // do nothing
                break;

    }
      };

    timer_ = create_wall_timer(1ms, timer_callback);

    RCLCPP_INFO(get_logger(), "simulation_vision node started");
  }

private:
  void updateMarker()
  {
    // convert pixels to meters
    auto x = target_.x / pixels_per_meter_;
    auto y = target_.y / pixels_per_meter_;
    auto size = target_.size / pixels_per_meter_;
    goal_tf_.header.frame_id = "phone_corner";
    goal_tf_.child_frame_id = "goal";
    goal_tf_.transform.translation.x = x;
    goal_tf_.transform.translation.y = y;
    goal_tf_.transform.translation.z = 0.0f;
    goal_tf_.transform.rotation.w = 1.0;

    marker_.header.frame_id = "phone_corner";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::SPHERE;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.position.x = x;
    marker_.pose.position.y = y;
    marker_.pose.position.z = 0.0f;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = size;
    marker_.scale.y = size;
    marker_.scale.z = size;
    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 0.8f;
    has_marker_ = true;
  }

  int id_;
  bool has_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  visualization_msgs::msg::Marker marker_;
  geometry_msgs::msg::TransformStamped goal_tf_;
  int count_;
  int max_count_;
  geometry_msgs::msg::TransformStamped phone_tf_;
  geometry_msgs::msg::TransformStamped phone_corner_tf_;
  double pixels_per_meter_;


  whackamole_interfaces::msg::Point target_;
  TargetState target_state_;
  rclcpp::Subscription<whackamole_interfaces::msg::Point>::SharedPtr target_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hit_feedback_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PixelVision>());
  rclcpp::shutdown();
  return 0;
};
