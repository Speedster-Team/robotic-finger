"""
Finds goal positions using OpenCV and publishes transforms.

Processes aligned RGB-D camera streams to detect and localize a colored
object target in 3D space. Applies a configurable HSV colorspace filter,
bilateral and Gaussian smoothing, morphological closing, Canny edge
detection, and contour analysis to identify the object. Reprojects the
detected 2D centroid to 3D using depth camera intrinsics and broadcasts
the result as a TF transform at 20 Hz. All HSV, depth clip, area, and
blur parameters are hot-reloadable from the ROS2 parameter server at
100 Hz.

SUBSCRIBERS:
  + /camera/camera/aligned_depth_to_color/image_raw (sensor_msgs/msg/Image)
    - Aligned depth image used for 3D reprojection and depth-clip filtering
  + /camera/camera/color/image_raw (sensor_msgs/msg/Image)
    - RGB color image used for HSV-based color segmentation
  + /camera/camera/color/camera_info (sensor_msgs/msg/CameraInfo)
    - Camera intrinsic matrix and distortion coefficients

PUBLISHERS:
  + /processed_result_image (sensor_msgs/msg/Image)
    - Binary HSV mask rendered as BGR for tuning and visualization
  + /annotated_result_image (sensor_msgs/msg/Image)
    - Original color image annotated with detected centroid marker
  + /sliced_result_image (sensor_msgs/msg/Image)
    - Depth-clipped color image (slicing currently disabled)

BROADCASTERS:
  + goal{i} → camera_color_optical_frame (geometry_msgs/msg/TransformStamped)
    - Dynamic TF transform for each detected object, indexed from 0
"""

import cv2

from cv_bridge import CvBridge

import numpy as np

from rcl_interfaces.msg import IntegerRange, ParameterDescriptor

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image

from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformStamped


class FingerVision(Node):
    """Find goal poses and publish."""

    def __init__(self):
        """Create instance of FingerVision class."""
        super().__init__('finger_vision')

        # declare parameters
        color_range = IntegerRange()
        color_range.from_value = 0
        color_range.to_value = 255
        clip_range = IntegerRange()
        clip_range.from_value = 0
        clip_range.to_value = 3000
        area_range = IntegerRange()
        area_range.from_value = 0
        area_range.to_value = 50000
        gaussian_range = IntegerRange()
        gaussian_range.from_value = 1
        gaussian_range.to_value = 25
        gaussian_range.step = 2

        self.declare_parameter('low_color_h', 40, ParameterDescriptor(
            description='Lower bound on hue color filter.',
            integer_range=[color_range]
            ))
        self.declare_parameter('high_color_h', 255, ParameterDescriptor(
            description='Upper bound on hue color filter.',
            integer_range=[color_range]
            ))
        self.declare_parameter('low_color_s', 30, ParameterDescriptor(
            description='Lower bound on saturation color filter.',
            integer_range=[color_range]
            ))
        self.declare_parameter('high_color_s', 255, ParameterDescriptor(
            description='Upper bound on saturation color filter.',
            integer_range=[color_range]
            ))
        self.declare_parameter('low_color_v', 82, ParameterDescriptor(
            description='Lower bound on value color filter.',
            integer_range=[color_range]
            ))
        self.declare_parameter('high_color_v', 129, ParameterDescriptor(
            description='Upper bound on value color filter.',
            integer_range=[color_range]
            ))
        self.declare_parameter('low_clip_dist', 150, ParameterDescriptor(
            description='Lower bound on clip distance filter in mm.',
            integer_range=[clip_range]
            ))
        self.declare_parameter('high_clip_dist', 1000, ParameterDescriptor(
            description='Upper bound on clip distance filter in mm.',
            integer_range=[clip_range]
            ))
        self.declare_parameter('low_area', 100, ParameterDescriptor(
            description='Lower bound on contour area filter.',
            integer_range=[area_range]
            ))
        self.declare_parameter('high_area', 22000, ParameterDescriptor(
            description='Upper bound on contour area filter.',
            integer_range=[area_range]
            ))
        self.declare_parameter('gaussian_blur', 1, ParameterDescriptor(
            description='Lower bound on contour area filter.',
            integer_range=[gaussian_range]
            ))
        self.low_color_h = self.get_parameter('low_color_h').value
        self.high_color_h = self.get_parameter('high_color_h').value
        self.low_color_s = self.get_parameter('low_color_s').value
        self.high_color_s = self.get_parameter('high_color_s').value
        self.low_color_v = self.get_parameter('low_color_v').value
        self.high_color_v = self.get_parameter('high_color_v').value
        self.min_clip_dist = self.get_parameter('low_clip_dist').value
        self.max_clip_dist = self.get_parameter('high_clip_dist').value
        self.min_area = self.get_parameter('low_area').value
        self.max_area = self.get_parameter('high_area').value
        self.gaussian_blur = self.get_parameter('gaussian_blur').value

        # init camera characteristics
        self.k = None
        self.d = None
        self.intrinsics = None

        # init count variable
        self.count = 0
        self.goal_list = []

        # init image vars
        self.image = None
        self.depth_image = None

        # create open CV bridge
        self.bridge = CvBridge()

        # create subscribers
        self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_image_callback, 10)
        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.info_callback, 10)

        # create publishers
        self.processed_pub = self.create_publisher(
            Image, 'processed_result_image', 10)
        self.annotated_pub = self.create_publisher(
            Image, 'annotated_result_image', 10)
        self.sliced_pub = self.create_publisher(
            Image, 'sliced_result_image', 10)
        self.sliced_pub = self.create_publisher(
            Image, 'sliced_result_image', 10)

        # create transform broadcaster
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # create timer to update transforms at constant freq
        period = 1.0/20.0
        self.process_timer = self.create_timer(period, self.process_image)
        period = 1.0/100.0
        self.param_timer = self.create_timer(period, self.param_timer_callback)

    def param_timer_callback(self):
        """Update parameters from ROS parameter server."""
        self.low_color_h = self.get_parameter('low_color_h').value
        self.high_color_h = self.get_parameter('high_color_h').value
        self.high_color_s = self.get_parameter('high_color_s').value
        self.low_color_v = self.get_parameter('low_color_v').value
        self.high_color_v = self.get_parameter('high_color_v').value
        self.min_clip_dist = self.get_parameter('low_clip_dist').value
        self.min_clip_dist = self.get_parameter('low_clip_dist').value
        self.max_clip_dist = self.get_parameter('high_clip_dist').value
        self.min_area = self.get_parameter('low_area').value
        self.max_area = self.get_parameter('high_area').value
        self.gaussian_blur = self.get_parameter('gaussian_blur').value

    def info_callback(self, msg):
        """Get most up to date camera info."""
        self.k = np.array(msg.k).reshape([3, 3])
        self.d = np.array(msg.d)

    def image_callback(self, image):
        """Save recieved image."""
        self.image = image

    def depth_image_callback(self, depth_image):
        """Save depth aligned image."""
        self.depth_image = depth_image

    def process_image(self):
        """Publish most up to date goal positions."""
        t = self.get_clock().now().to_msg()

        if self.image is not None and self.depth_image is not None:

            # convert to cv objects
            cv_image = self.bridge.imgmsg_to_cv2(self.image, 'bgr8')
            cv_depth_image = self.bridge.imgmsg_to_cv2(
                self.depth_image, '16UC1')

            # find circles
            (
                cv_annotated_img,
                cv_processed_img,
                cv_sliced_img,
                x_list,
                y_list,
                z_list,
                q_list,
            ) = self.contour_filter(
                cv_image,
                cv_depth_image,
                self.k,
            )

            # bridge processed images
            processed_img = self.bridge.cv2_to_imgmsg(cv_processed_img, 'bgr8')
            annotated_img = self.bridge.cv2_to_imgmsg(cv_annotated_img, 'bgr8')
            sliced_img = self.bridge.cv2_to_imgmsg(cv_sliced_img, 'bgr8')

            # publish processed images
            self.processed_pub.publish(processed_img)
            self.annotated_pub.publish(annotated_img)
            self.sliced_pub.publish(sliced_img)

            # save number of circles detected
            count = len(x_list)

            # check if any circles detected
            if count > 0:
                # empty the list
                self.goal_list = []

                for ii in range(count):
                    goal_tf = TransformStamped()
                    goal_tf.header.frame_id = 'camera_color_optical_frame'
                    goal_tf.transform.translation.x = x_list[ii]
                    goal_tf.transform.translation.y = y_list[ii]
                    goal_tf.transform.translation.z = z_list[ii]
                    goal_tf.transform.rotation.w = q_list[ii][0]
                    goal_tf.transform.rotation.x = q_list[ii][1]
                    goal_tf.transform.rotation.y = q_list[ii][2]
                    goal_tf.transform.rotation.z = q_list[ii][3]
                    goal_tf.header.stamp = t
                    goal_tf.child_frame_id = f'goal{ii}'
                    self.broadcaster.sendTransform(goal_tf)

    # SLICING CURRENTLY DISABLED
    def slice_img(self, color_image, depth_image):
        """Slice image based on depth values."""
        np_img = np.asanyarray(depth_image)
        depth_image_3d = np.dstack((np_img, np_img, np_img))

        # depth image is 1 channel, color is 3 channels
        sliced_img = np.where((
            depth_image_3d > self.max_clip_dist) | (
                depth_image_3d < self.min_clip_dist
                ) | (depth_image_3d <= 0), 153, color_image
            )
        return sliced_img

    def colorspace_filter(self, color_image, depth_image):
        """Filter out colors on image."""
        # do not use slicing right now
        # sliced_img = self.slice_img(color_image, depth_image)
        sliced_img = color_image

        # Convert BGR to HSV
        hsv = cv2.cvtColor(sliced_img, cv2.COLOR_BGR2HSV)
        hsv = cv2.bilateralFilter(hsv, 9, 75, 75)
        hsv = cv2.GaussianBlur(
            hsv, (self.gaussian_blur, self.gaussian_blur), 0)
        hsv = cv2.morphologyEx(
            hsv, cv2.MORPH_CLOSE, (self.gaussian_blur, self.gaussian_blur))

        # define range of the color in HSV
        lower_color = np.array(
            [self.low_color_h, self.low_color_s, self.low_color_v],
            dtype=np.uint8)
        upper_color = np.array(
            [self.high_color_h, self.high_color_s, self.high_color_v],
            dtype=np.uint8)

        # Threshold the HSV image to get only the color required
        mask = cv2.inRange(hsv, lower_color, upper_color)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(sliced_img, sliced_img, mask=mask)

        return mask_bgr, mask, res, sliced_img

    def position(self, cv_depth_img, u, v, k):
        """Calculate point position using depth camera and color intrinsics."""
        fx = k[0, 0]
        fy = k[1, 1]
        cx = k[0, 2]
        cy = k[1, 2]

        scale = 0.001  # scales from mm to m
        z = cv_depth_img[v, u] * scale
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return x, y, z

    def contour_filter(self, image, cv_depth_image, k):
        """Use opencv to find the position and orientation."""
        # filter based on color first
        thre_bgr, thre_image, _, sliced_image = self.colorspace_filter(
            image, cv_depth_image)

        # Getting the edge of morphology
        edge = cv2.Canny(thre_image, 175, 175)
        contours, _ = cv2.findContours(edge, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # Find the index of the largest contour
        areas = [cv2.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        cnt = contours[max_index]

        # # find countours
        # contours, _ = cv2.findContours(
        #     thre_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        coord_list = []
        radius_list = []
        x_list = []
        y_list = []
        z_list = []
        quaternion_list = []

        (x, y), radius = cv2.minEnclosingCircle(cnt)
        area = np.max(areas)

        if self.min_area < area < self.max_area:
            coord_list.append((x, y))
            radius_list.append(int(radius))
            quaternion_list.append([0.0, 0.0, 0.0, 1.0])

        for i, coord in enumerate(coord_list):

            coord_x, coord_y = coord
            # cv2.circle(image, (int(coord_x), int(coord_y)), radius_list[i],
            #            (0, 255, 0), 2)
            cv2.circle(image, (int(coord_x), int(coord_y)), 20,
                       (0, 0, 255), -1)

            # Get the 3D position of the centroid
            x, y, z = self.position(
                cv_depth_image, int(coord_x), int(coord_y), k)

            x_list.append(x)
            y_list.append(y)
            z_list.append(z)

        return (
            image,
            thre_bgr,
            sliced_image,
            x_list,
            y_list,
            z_list,
            quaternion_list,
        )


def main(args=None):
    """Run the circle finder node."""
    try:
        rclpy.init(args=args)
        node = FingerVision()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
