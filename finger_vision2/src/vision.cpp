/// \file
/// \brief Finds goal positions using OpenCV and publishes transforms.
///
/// Processes aligned RGB-D camera streams to detect and localize a colored
/// object target in 3D space. Applies a configurable HSV colorspace filter,
/// bilateral and Gaussian smoothing, morphological closing, Canny edge
/// detection, and contour analysis to identify the object. Reprojects the
/// detected 2D centroid to 3D using depth camera intrinsics and broadcasts
/// the result as a TF transform at 20 Hz. All HSV, depth clip, area, and
/// blur parameters are hot-reloadable from the ROS2 parameter server at
/// 100 Hz.
///
/// SUBSCRIBERS:
///   + /camera/camera/aligned_depth_to_color/image_raw (sensor_msgs/msg/Image)
///   + /camera/camera/color/image_raw (sensor_msgs/msg/Image)
///   + /camera/camera/color/camera_info (sensor_msgs/msg/CameraInfo)
///
/// PUBLISHERS:
///   + processed_result_image (sensor_msgs/msg/Image)
///   + annotated_result_image (sensor_msgs/msg/Image)
///   + sliced_result_image (sensor_msgs/msg/Image)
///
/// BROADCASTERS:
///   + goal{i} -> camera_color_optical_frame
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/// \brief Find goal poses and publish.
class FingerVision : public rclcpp::Node
{
public:
  FingerVision()
  : Node("finger_vision")
  {
    // ---- parameter ranges ----
    rcl_interfaces::msg::IntegerRange color_range;
    color_range.from_value = 0;
    color_range.to_value = 255;
    color_range.step = 1;

    rcl_interfaces::msg::IntegerRange clip_range;
    clip_range.from_value = 0;
    clip_range.to_value = 3000;
    clip_range.step = 1;

    rcl_interfaces::msg::IntegerRange area_range;
    area_range.from_value = 0;
    area_range.to_value = 50000;
    area_range.step = 1;

    rcl_interfaces::msg::IntegerRange gaussian_range;
    gaussian_range.from_value = 1;
    gaussian_range.to_value = 25;
    gaussian_range.step = 2;

    auto make_desc = [](const std::string & description,
      const rcl_interfaces::msg::IntegerRange & range) {
        rcl_interfaces::msg::ParameterDescriptor d;
        d.description = description;
        d.integer_range.push_back(range);
        return d;
      };

    // ---- declare parameters ----
    declare_parameter<int>(
      "low_color_h", 40, make_desc("Lower bound on hue color filter.", color_range));
    declare_parameter<int>(
      "high_color_h", 255, make_desc("Upper bound on hue color filter.", color_range));
    declare_parameter<int>(
      "low_color_s", 30, make_desc("Lower bound on saturation color filter.", color_range));
    declare_parameter<int>(
      "high_color_s", 255, make_desc("Upper bound on saturation color filter.", color_range));
    declare_parameter<int>(
      "low_color_v", 82, make_desc("Lower bound on value color filter.", color_range));
    declare_parameter<int>(
      "high_color_v", 129, make_desc("Upper bound on value color filter.", color_range));
    declare_parameter<int>(
      "low_clip_dist", 150,
      make_desc("Lower bound on clip distance filter in mm.", clip_range));
    declare_parameter<int>(
      "high_clip_dist", 1000,
      make_desc("Upper bound on clip distance filter in mm.", clip_range));
    declare_parameter<int>(
      "low_area", 100, make_desc("Lower bound on contour area filter.", area_range));
    declare_parameter<int>(
      "high_area", 22000, make_desc("Upper bound on contour area filter.", area_range));
    declare_parameter<int>(
      "gaussian_blur", 1, make_desc("Gaussian blur kernel size (odd).", gaussian_range));

    refresh_parameters();

    // ---- subs ----
    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw", 10,
      std::bind(&FingerVision::image_callback, this, _1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/aligned_depth_to_color/image_raw", 10,
      std::bind(&FingerVision::depth_image_callback, this, _1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera/color/camera_info", 10,
      std::bind(&FingerVision::info_callback, this, _1));

    // ---- pubs ----
    processed_pub_ = create_publisher<sensor_msgs::msg::Image>("processed_result_image", 10);
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>("annotated_result_image", 10);
    sliced_pub_ = create_publisher<sensor_msgs::msg::Image>("sliced_result_image", 10);

    // ---- tf ----
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // ---- timers ----
    process_timer_ = create_wall_timer(
      50ms, std::bind(&FingerVision::process_image, this));   // 20 Hz
    param_timer_ = create_wall_timer(
      10ms, std::bind(&FingerVision::refresh_parameters, this));  // 100 Hz
  }

private:
  // -------- callbacks --------

  /// \brief Update parameters from ROS parameter server.
  void refresh_parameters()
  {
    low_color_h_ = get_parameter("low_color_h").as_int();
    high_color_h_ = get_parameter("high_color_h").as_int();
    low_color_s_ = get_parameter("low_color_s").as_int();
    high_color_s_ = get_parameter("high_color_s").as_int();
    low_color_v_ = get_parameter("low_color_v").as_int();
    high_color_v_ = get_parameter("high_color_v").as_int();
    min_clip_dist_ = get_parameter("low_clip_dist").as_int();
    max_clip_dist_ = get_parameter("high_clip_dist").as_int();
    min_area_ = get_parameter("low_area").as_int();
    max_area_ = get_parameter("high_area").as_int();
    int g = get_parameter("gaussian_blur").as_int();
    if (g % 2 == 0) {g += 1;}   // OpenCV requires odd kernel size
    gaussian_blur_ = g;
  }

  /// \brief Get most up to date camera info.
  void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    k_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        k_.at<double>(i, j) = msg->k[i * 3 + j];
      }
    }
    d_ = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i) {
      d_.at<double>(0, static_cast<int>(i)) = msg->d[i];
    }
    have_intrinsics_ = true;
  }

  /// \brief Save received color image.
  void image_callback(const sensor_msgs::msg::Image::SharedPtr image)
  {
    image_ = image;
  }

  /// \brief Save received depth-aligned image.
  void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr depth_image)
  {
    depth_image_ = depth_image;
  }

  /// \brief Publish most up to date goal positions.
  void process_image()
  {
    const auto stamp = now();

    if (!image_ || !depth_image_ || !have_intrinsics_) {
      return;
    }

    // convert ROS images to OpenCV mats
    cv::Mat cv_image;
    cv::Mat cv_depth_image;
    try {
      cv_image = cv_bridge::toCvCopy(image_, "bgr8")->image;
      cv_depth_image = cv_bridge::toCvCopy(depth_image_, "16UC1")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat annotated;
    cv::Mat processed_bgr;
    cv::Mat sliced;
    std::vector<double> x_list;
    std::vector<double> y_list;
    std::vector<double> z_list;
    std::vector<std::array<double, 4>> q_list;   // (w, x, y, z)

    contour_filter(
      cv_image, cv_depth_image, k_,
      annotated, processed_bgr, sliced,
      x_list, y_list, z_list, q_list);

    // publish processed images
    auto processed_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", processed_bgr).toImageMsg();
    auto annotated_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", annotated).toImageMsg();
    auto sliced_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", sliced).toImageMsg();

    processed_pub_->publish(*processed_msg);
    annotated_pub_->publish(*annotated_msg);
    sliced_pub_->publish(*sliced_msg);

    // broadcast a transform for every detected goal
    for (size_t i = 0; i < x_list.size(); ++i) {
      geometry_msgs::msg::TransformStamped goal_tf;
      goal_tf.header.stamp = stamp;
      goal_tf.header.frame_id = "camera_color_optical_frame";
      goal_tf.child_frame_id = "goal" + std::to_string(i);
      goal_tf.transform.translation.x = x_list[i];
      goal_tf.transform.translation.y = y_list[i];
      goal_tf.transform.translation.z = z_list[i];
      goal_tf.transform.rotation.w = q_list[i][0];
      goal_tf.transform.rotation.x = q_list[i][1];
      goal_tf.transform.rotation.y = q_list[i][2];
      goal_tf.transform.rotation.z = q_list[i][3];
      broadcaster_->sendTransform(goal_tf);
    }
  }

  // -------- image processing --------

  /// \brief Slice image based on depth values (currently unused; kept for parity).
  cv::Mat slice_img(const cv::Mat & color_image, const cv::Mat & depth_image) const
  {
    cv::Mat out = color_image.clone();
    for (int v = 0; v < color_image.rows; ++v) {
      for (int u = 0; u < color_image.cols; ++u) {
        const uint16_t z = depth_image.at<uint16_t>(v, u);
        if (z == 0 || z < min_clip_dist_ || z > max_clip_dist_) {
          out.at<cv::Vec3b>(v, u) = cv::Vec3b(153, 153, 153);
        }
      }
    }
    return out;
  }

  /// \brief Filter out colors on image.
  void colorspace_filter(
    const cv::Mat & color_image, const cv::Mat & /*depth_image*/,
    cv::Mat & mask_bgr_out, cv::Mat & mask_out,
    cv::Mat & res_out, cv::Mat & sliced_out) const
  {
    // slicing disabled in original; preserve that behavior
    sliced_out = color_image;

    cv::Mat hsv;
    cv::cvtColor(sliced_out, hsv, cv::COLOR_BGR2HSV);

    cv::Mat tmp;
    cv::bilateralFilter(hsv, tmp, 9, 75, 75);
    hsv = tmp;

    const int k = gaussian_blur_;
    cv::GaussianBlur(hsv, hsv, cv::Size(k, k), 0);

    // Note: original Python passed a tuple as the second arg to morphologyEx,
    // which OpenCV silently coerced. The intent is a kxk structuring element.
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
    cv::morphologyEx(hsv, hsv, cv::MORPH_CLOSE, element);

    cv::Scalar lower(low_color_h_, low_color_s_, low_color_v_);
    cv::Scalar upper(high_color_h_, high_color_s_, high_color_v_);

    cv::Mat mask;
    cv::inRange(hsv, lower, upper, mask);

    cv::Mat mask_bgr;
    cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);

    cv::Mat res;
    cv::bitwise_and(sliced_out, sliced_out, res, mask);

    mask_out = mask;
    mask_bgr_out = mask_bgr;
    res_out = res;
  }

  /// \brief Reproject (u, v) + depth back to a 3D point using color intrinsics.
  void position(
    const cv::Mat & cv_depth_img, int u, int v, const cv::Mat & k,
    double & x, double & y, double & z) const
  {
    const double fx = k.at<double>(0, 0);
    const double fy = k.at<double>(1, 1);
    const double cx = k.at<double>(0, 2);
    const double cy = k.at<double>(1, 2);

    constexpr double scale = 0.001;   // mm -> m
    z = static_cast<double>(cv_depth_img.at<uint16_t>(v, u)) * scale;
    x = (u - cx) * z / fx;
    y = (v - cy) * z / fy;
  }

  /// \brief Use OpenCV to find target position(s) and orientation(s).
  void contour_filter(
    cv::Mat & image, const cv::Mat & cv_depth_image, const cv::Mat & k,
    cv::Mat & annotated_out, cv::Mat & thre_bgr_out, cv::Mat & sliced_out,
    std::vector<double> & x_list, std::vector<double> & y_list,
    std::vector<double> & z_list,
    std::vector<std::array<double, 4>> & quaternion_list) const
  {
    cv::Mat thre_bgr;
    cv::Mat thre_image;
    cv::Mat res;
    cv::Mat sliced;
    colorspace_filter(image, cv_depth_image, thre_bgr, thre_image, res, sliced);

    cv::Mat edge;
    cv::Canny(thre_image, edge, 175, 175);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edge, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
      // find the largest contour by area
      int max_index = 0;
      double max_area = -1.0;
      for (size_t i = 0; i < contours.size(); ++i) {
        const double a = cv::contourArea(contours[i]);
        if (a > max_area) {
          max_area = a;
          max_index = static_cast<int>(i);
        }
      }

      if (max_area > min_area_ && max_area < max_area_) {
        cv::Point2f center;
        float radius = 0.0f;
        cv::minEnclosingCircle(contours[max_index], center, radius);

        // annotate centroid
        cv::circle(
          image, cv::Point(static_cast<int>(center.x), static_cast<int>(center.y)),
          20, cv::Scalar(0, 0, 255), -1);

        double x, y, z;
        position(
          cv_depth_image,
          static_cast<int>(center.x), static_cast<int>(center.y),
          k, x, y, z);

        x_list.push_back(x);
        y_list.push_back(y);
        z_list.push_back(z);
        quaternion_list.push_back({1.0, 0.0, 0.0, 0.0});   // (w, x, y, z) identity
      }
    }

    annotated_out = image;
    thre_bgr_out = thre_bgr;
    sliced_out = sliced;
  }

  // -------- members --------

  // parameters
  int low_color_h_{0}, high_color_h_{0};
  int low_color_s_{0}, high_color_s_{0};
  int low_color_v_{0}, high_color_v_{0};
  int min_clip_dist_{0}, max_clip_dist_{0};
  int min_area_{0}, max_area_{0};
  int gaussian_blur_{1};

  // camera intrinsics
  cv::Mat k_;
  cv::Mat d_;
  bool have_intrinsics_{false};

  // latest images
  sensor_msgs::msg::Image::SharedPtr image_;
  sensor_msgs::msg::Image::SharedPtr depth_image_;

  // subs
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  // pubs
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sliced_pub_;

  // tf
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // timers
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr param_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FingerVision>());
  rclcpp::shutdown();
  return 0;
}
