"""Class leafsystem that computes tendon velocities."""

import numpy as np

from pydrake.systems.framework import LeafSystem


class TendonFeedbackSystem(LeafSystem):
    """Leaf system computing tendon velocities."""

    def __init__(self):
        """Create instance of TendonFeedbackSystem."""
        super().__init__()

        # define structure matrix
        ra = 0.0025  # splay motor shaft radius

        r11 = ra * 3.5
        r1 = 8 * 0.001
        r3 = 4.5 * 0.001
        r5 = 8 * 0.001
        r7 = 4.5 * 0.001
        r9 = 9 * 0.001

        self.St = np.array([[-r11, -r3, r1],  # splay joint
                            [0,    r7,  r5],  # mcp joint
                            [0,     0,  r9]])  # pip/dip joint

        self.St_inv = np.linalg.inv(self.St)

        # Tendon stiffness vector
        self.k = np.array([2500.0, 2500.0, 2500.0, 2500.0])

        # init size of input and output
        nq = 5  # five link positions
        nv = 5  # five link velocities
        nu = 3  # three tendon velocities

        # declare input and output ports with functions
        self.joint_state_input_port = self.DeclareVectorInputPort(
            'finger_state', nq + nv)
        self.tendon_tension_input_port = self.DeclareVectorInputPort(
            'tendon_tension', nu)
        self.DeclareVectorOutputPort(
            'tendon_velocity', nu, self._tendon_vel)
        self.DeclareVectorOutputPort(
            'tendon_position', nu, self._tendon_pos)

        # structure of finger state
        # state x (size 10) = [q_splay, q_mcp_flex, q_pip1, q_dip1, q_pip2,
        #                      v_splay, v_mcp_flex, v_pip1, v_dip1, v_pip2]

    def _tendon_vel(self, context, output):
        """Convert joint state to tendon velocity."""
        state = self.joint_state_input_port.Eval(context)
        # tension = self.tendon_tension_input_port.Eval(context)

        # filter out splay, mcp, and pip
        vels = state[5:8]

        # calculate tension extension due to force
        # stretch = tension * self.k  # do nothing with it for nowcb

        # transform to tendon space
        output.SetFromVector(self.St.T @ vels)

    def _tendon_pos(self, context, output):
        """Convert joint state to tendon position."""
        state = self.joint_state_input_port.Eval(context)
        # tension = self.tendon_tension_input_port.Eval(context)

        # filter out splay, mcp, and pip
        poses = state[0:3]

        # print(f'joint states: {poses}')

        # calculate tension extension due to force
        # stretch = tension * self.k  # do nothing with it for now

        # transform to tendon space
        output.SetFromVector(self.St.T @ poses)
