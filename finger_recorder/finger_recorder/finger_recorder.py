"""
Records finger motor data from ROS2 topics to an MCAP bag file.

SUBSCRIBERS:
  + motor_pos_actual_feedback (finger_interfaces/msg/MotorFeedback)
    - Actual motor positions
  + motor_pos_setpoint_feedback (finger_interfaces/msg/MotorFeedback)
    - Setpoint motor positions
  + motor_pos_activity_feedback (finger_interfaces/msg/MotorActivity)
    - Motor activity state
"""

from datetime import datetime

from finger_interfaces.msg import MotorActivity, MotorFeedback

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import serialize_message

import rosbag2_py


class FingerRecorder(Node):
    """Class to record finger data."""

    def __init__(self):
        """Create instance of FingerRecorder."""
        super().__init__('finger_recorder')
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=f'src/robotic-finger/finger_recorder/bags/\
finger_bag_{datetime.now().strftime("%Y%m%d_%H%M%S")}',
            storage_id='mcap')
        self.writer.open(storage_options, rosbag2_py.ConverterOptions('', ''))

        for i, (name, msg_type) in enumerate([
            ('motor_pos_actual_feedback',
                'finger_interfaces/msg/MotorFeedback'),
            ('motor_pos_setpoint_feedback',
                'finger_interfaces/msg/MotorFeedback'),
            ('motor_pos_activity_feedback',
                'finger_interfaces/msg/MotorActivity'),
            ('setpoint/joint_feedback',
                'finger_interfaces/msg/MotorActivity'),
            ('actual/joint_feedback',
                'finger_interfaces/msg/MotorActivity'),
        ]):
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                id=i, name=name, type=msg_type, serialization_format='cdr'))

        self.create_subscription(MotorFeedback,
                                 'motor_pos_actual_feedback',
                                 self.actual_callback,   10)
        self.create_subscription(MotorFeedback,
                                 'motor_pos_setpoint_feedback',
                                 self.setpoint_callback, 10)
        self.create_subscription(MotorActivity,
                                 'motor_pos_activity_feedback',
                                 self.activity_callback, 10)
        self.create_subscription(MotorFeedback,
                                 'setpoint/joint_feedback',
                                 self.setpoint_joint_callback, 10)
        self.create_subscription(MotorFeedback,
                                 'actual/joint_feedback',
                                 self.actual_joint_callback, 10)

    def actual_callback(self, msg):
        """Subscription callback for actual motor position feedback."""
        self.writer.write('motor_pos_actual_feedback',
                          serialize_message(msg),
                          self.get_clock().now().nanoseconds)

    def setpoint_callback(self, msg):
        """Subscription callback for actual motor setpoint feedback."""
        self.writer.write('motor_pos_setpoint_feedback',
                          serialize_message(msg),
                          self.get_clock().now().nanoseconds)

    def activity_callback(self, msg):
        """Subscription callback for actual motor activity feedback."""
        self.writer.write('motor_pos_activity_feedback',
                          serialize_message(msg),
                          self.get_clock().now().nanoseconds)

    def actual_joint_callback(self, msg):
        """Subscription callback for actual joint position feedback."""
        self.writer.write('actual/joint_feedback',
                          serialize_message(msg),
                          self.get_clock().now().nanoseconds)
        
    def setpoint_joint_callback(self, msg):
        """Subscription callback for setpoint motor setpoint feedback."""
        self.writer.write('setpoint/joint_feedback',
                          serialize_message(msg),
                          self.get_clock().now().nanoseconds)

def main(args=None):
    """Start the node."""
    try:
        with rclpy.init(args=args):
            rclpy.spin(FingerRecorder())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
