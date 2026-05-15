"""Class leafsystem that transforms tendon force to joint torques."""

import numpy as np

from pydrake.systems.framework import LeafSystem


class FingerPulleySystem(LeafSystem):
    """Leaf system converting motor torque tendon forces."""

    def __init__(self):
        """Create instance of FingerPulleySystem."""
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

        self.St_inv = np.linalg.pinv(self.St)

        # init size of input and output
        nu = 3

        # declare input and output ports with functions
        self.tendon_input_port = self.DeclareVectorInputPort(
            'tendon_tension', nu)
        self.DeclareVectorOutputPort('joint_torque', nu, self._calc_torque)

        self.position_input_port = self.DeclareVectorInputPort(
            'tendon_position', nu)
        self.DeclareVectorOutputPort('joint_position', nu, self._calc_position)

    def _calc_torque(self, context, output):
        """Convert motor torque to tendon forces."""
        tension = self.tendon_input_port.Eval(context)
        output.SetFromVector(self.St_inv.T @ tension)

    def _calc_position(self, context, output):
        """Convert motor torque to tendon forces."""
        position = self.position_input_port.Eval(context)
        output.SetFromVector(self.St_inv.T @ position)
