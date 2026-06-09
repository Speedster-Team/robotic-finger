"""Leafsystem class connecting drake simulation outputs to ROS."""

from finger_interfaces.msg import MotorFeedback

from pydrake.systems.framework import LeafSystem


class Drake2Ros(LeafSystem):
    """Leaf system connecting drake outputs to ros."""

    def __init__(self, node):
        """Create instance of Drake2Ros."""
        super().__init__()
        self.nu = 8

        # save node
        self._node = node

        # Input port for motor position
        self.pos_input_port = self.DeclareVectorInputPort(
            'finger_state', self.nu
        )

        # init publisher
        self._joint_pub = self._node.create_publisher(
            MotorFeedback,
            'joint_feedback',
            10
        )

        # create event to publish at specified freq
        self.DeclarePeriodicPublishEvent(
            period_sec=1.0/100.0,   # 100 Hz
            offset_sec=0.0,
            publish=self._publish_joint_feedback,
        )

        # state x (size 10) = [q_splay, q_mcp_flex, q_pip1, q_dip1, q_pip2,
        #                      v_splay, v_mcp_flex, v_pip1, v_dip1, v_pip2]

    def _publish_joint_feedback(self, context):
        """Publish the joint position on a ROS topic."""
        pos = self.pos_input_port.Eval(context)
        msg = MotorFeedback()
        msg.motor_positions = pos.tolist()[0:3]  # take first 3
        msg.header.stamp = self._node.get_clock().now().to_msg()
        self._joint_pub.publish(msg)
