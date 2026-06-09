"""Class describing leafsystem transforming motor torque to tendon tension."""

import numpy as np

from pydrake.systems.framework import LeafSystem


class MotorTorqueToForceSystem(LeafSystem):
    """Leaf system converting motor torque tendon tension."""

    def __init__(self):
        """Create instance of MotorTorqueToForceSystem."""
        super().__init__()

        # define radii
        ra, rb, rc = 0.0075, 0.0025, 0.0025

        # define radius matrix
        self.Ra = np.array([[ra, 0, 0],
                            [0, rb, 0],
                            [0, 0, rc]])

        nu = 3

        # declare input and output ports with functions
        self.torque_input_port = self.DeclareVectorInputPort(
            'motor_torque', nu)
        self.DeclareVectorOutputPort('tendon_tension', nu, self._calc_force)

        self.position_input_port = self.DeclareVectorInputPort(
            'motor_position', nu)
        self.DeclareVectorOutputPort('tendon_position', nu, self._calc_linear)

    def _calc_force(self, context, output):
        """Convert motor torque to tendon forces."""
        torque = self.torque_input_port.Eval(context)

        torque[0] *= 3.5
        output.SetFromVector(self.Ra @ torque)

    def _calc_linear(self, context, output):
        """Convert motor position to tendon position."""
        position = self.position_input_port.Eval(context)
        output.SetFromVector(self.Ra @ position)
