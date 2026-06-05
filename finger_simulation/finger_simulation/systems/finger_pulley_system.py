"""Class leafsystem that transforms tendon force to joint torques."""

from finger_interfaces.msg import MotorFeedback

import numpy as np

from pydrake.systems.framework import LeafSystem


class FingerPulleySystem(LeafSystem):
    """Leaf system converting motor torque tendon forces."""

    def __init__(self, node=None):
        """Create instance of FingerPulleySystem."""
        super().__init__()

        self._node = node

        # define structure matrix
        ra = 0.0075  # splay motor shaft radius

        r11 = ra * 3.5
        r1 = 8 * 0.001
        r3 = 4.5 * 0.001
        r5 = 8 * 0.001
        r7 = 4.5 * 0.001
        r9 = 10 * 0.001

        self.St = np.array([[-r11, r3, -r1],  # splay joint
                            [0,    -r7,  -r5],  # mcp joint
                            [0,     0,  -r9]])  # pip/dip joint

        self.St_inv = np.linalg.pinv(self.St)

        # init size of input and output
        nu = 3

        # declare input and output ports with functions
        self.tendon_input_port = self.DeclareVectorInputPort(
            'tendon_tension', nu)

        self.position_input_port = self.DeclareVectorInputPort(
            'tendon_position', nu)

        self.DeclareVectorOutputPort('joint_torque', nu, self._calc_torque)
        self.DeclareVectorOutputPort('joint_position', nu, self._calc_position)

        if self._node is not None:
            # init publisher
            self._joint_torque_pub = self._node.create_publisher(
                MotorFeedback,
                '/joint_torque_drake_feedback',
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
        tension = self.tendon_input_port.Eval(context)
        torque = self.St_inv.T @ tension
        msg = MotorFeedback()
        msg.motor_positions = torque.tolist()[0:3]  # take first 3
        self._joint_torque_pub.publish(msg)

    def _calc_torque(self, context, output):
        """Convert motor torque to tendon forces."""
        tension = self.tendon_input_port.Eval(context)
        torque = self.St_inv.T @ tension
        output.SetFromVector(torque)

    def _calc_position(self, context, output):
        """Convert motor torque to tendon forces."""
        position = self.position_input_port.Eval(context)
        output.SetFromVector(self.St_inv.T @ position)
