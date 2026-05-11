"""
Runs Drake dynamics simulation.

Runs the Drake simulation of the robotic finger with the full tendon-driven
system pipeline. Loads the finger SDF and scene models, closes the four-bar
linkage loop constraint, and wires the complete Drake system graph:
motor torques and positions → tendon tensions and positions → joint torques
→ MultibodyPlant → finger state → tendon feedback → motor feedback → ROS2.
Exposes a heartbeat service to signal that Drake has finished initializing.

SUBSCRIBERS:
  + /motor_pos_setpoint_feedback (finger_interfaces/msg/MotorFeedback)
    - Motor torque and position setpoints consumed by Ros2Drake

PUBLISHERS:
  + /motor_pos_actual_feedback (finger_interfaces/msg/MotorFeedback)
    - Actual motor positions published by Drake2Ros
  + /motor_velocity (finger_interfaces/msg/MotorFeedback)
    - Actual motor velocities published by Drake2Ros

SERVERS:
  + /heartbeat (std_srvs/srv/Empty)
    - Indicates that the Drake simulation has finished initializing
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

import drake_ros.core
from drake_ros.core import ClockSystem, RosInterfaceSystem
from drake_ros.tf2 import SceneTfBroadcasterParams, SceneTfBroadcasterSystem
from drake_ros.viz import RvizVisualizer, RvizVisualizerParams

from finger_simulation.systems.drake2ros_system import Drake2Ros
from finger_simulation.systems.finger_pulley_system import FingerPulleySystem
from finger_simulation.systems.motor_feedback_system import MotorFeedbackSystem
from finger_simulation.systems.motor_radius_system import (
    MotorTorqueToForceSystem
)
from finger_simulation.systems.ros2drake_system import Ros2Drake
from finger_simulation.systems.tendon_feedback_system import (
    TendonFeedbackSystem
)

import numpy as np

import pydot

from pydrake.geometry import Meshcat, MeshcatVisualizer
from pydrake.geometry import MeshcatVisualizerParams
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, TriggerType
from pydrake.systems.primitives import ConstantVectorSource

import rclpy

from std_srvs.srv import Empty


class FingerSimulation():
    """Class runs drake simulation of the finger."""

    def __init__(self):
        """Create instance of FingerSimulation."""
        self.builder = DiagramBuilder()
        drake_ros.core.init(args=sys.argv)

        # init time step
        self.dt = 0.001

        self.sys_ros_interface = self.builder.AddSystem(
            RosInterfaceSystem('fingersim'))
        ClockSystem.AddToBuilder(self.builder,
                                 self.sys_ros_interface.get_ros_interface())

        self.plant, self.scene_graph = AddMultibodyPlant(
            MultibodyPlantConfig(time_step=self.dt),
            self.builder,
        )

    def init_viz(self, enable_rviz):
        """Initialize all visualization aspects of drake simulation."""
        # start tf broadcaster
        viz_dt = 1 / 100.0
        scene_tf_broadcaster = self.builder.AddSystem(
            SceneTfBroadcasterSystem(
                self.sys_ros_interface.get_ros_interface(),
                params=SceneTfBroadcasterParams(
                    publish_triggers={TriggerType.kPeriodic},
                    publish_period=viz_dt,
                ),
            )
        )

        self.builder.Connect(
            self.scene_graph.get_query_output_port(),
            scene_tf_broadcaster.get_graph_query_input_port(),
        )

        if (enable_rviz):
            # FOR RVIZ
            scene_visualizer = self.builder.AddSystem(
                RvizVisualizer(
                    self.sys_ros_interface.get_ros_interface(),
                    params=RvizVisualizerParams(
                        publish_triggers={TriggerType.kPeriodic},
                        publish_period=viz_dt,
                    ),
                )
            )

            self.builder.Connect(
                self.scene_graph.get_query_output_port(),
                scene_visualizer.get_graph_query_input_port(),
            )
        else:
            # FOR MESHCAT
            meshcat = Meshcat()

            MeshcatVisualizer.AddToBuilder(
                self.builder,
                self.scene_graph,
                meshcat,
                MeshcatVisualizerParams()
            )

    def load_scene(self):
        """Load all models into scene and fix in space."""
        # Load finger SDF
        parser = Parser(self.plant)
        pm = parser.package_map()

        # Derive workspace install dir from any known package
        share_dir = get_package_share_directory('finger_simulation')
        # share/pkg -> install/
        install_dir = os.path.join(share_dir, '..', '..', '..')
        pm.PopulateFromFolder(os.path.realpath(install_dir))

        # Weld the grass to the world so that it's fixed during the simulation.
        self.finger, = parser.AddModels(
            url='package://finger_description/sdf/finger.sdf')
        self.plant.RenameModelInstance(
            model_instance=self.finger, name='speedster_finger')
        parser.AddModels(
            url='package://finger_simulation/models/grasspatch/model.sdf')
        parser.AddModels(
            url='package://finger_simulation/models/Standard_Toilet/model.sdf')
        # self.box, = parser.AddModels(
        #     url='package://finger_simulation/models/box/model.sdf')

        base_frame = self.plant.GetFrameByName('base_link', self.finger)
        self.plant.WeldFrames(self.plant.world_frame(),
                              base_frame,
                              RigidTransform(RollPitchYaw([0, 0, 0]),
                                             np.array([0, 0, .2])))

        # box_frame = self.plant.GetFrameByName('box_frame', self.box)
        # self.plant.WeldFrames(self.plant.world_frame(),
        #                       box_frame,
        #                       RigidTransform(RollPitchYaw([0, 0, 0]),
        #                                      np.array([0, 0.2, .05])))

        # close loop for four bar
        middle_phalanx2 = self.plant.GetBodyByName(
            'middle_phalanx2', self.finger)
        distal_phalanx = self.plant.GetBodyByName(
            'distal_phalanx', self.finger)

        # dip_flex2 in middle_phalanx2 frame
        p_AP = np.array([0.0001,  0.0348, -0.0147])

        # dip_flex2 in distal_phalanx frame
        p_BQ = np.array([-0.0054, 0.0022, -0.0087])

        self.plant.AddBallConstraint(
            body_A=middle_phalanx2,
            p_AP=p_AP,
            body_B=distal_phalanx,
            p_BQ=p_BQ,
        )
        self.plant.Finalize()

        # turn off gravity
        # self.plant.mutable_gravity_field().set_gravity_vector([0, 0, 10.0])

        # set up other object locations
        grasspatch_frame = self.plant.GetFrameByName('grasspatch_frame')
        plant_context = self.plant.CreateDefaultContext()
        tf_world_grasspatch = grasspatch_frame.CalcPoseInWorld(plant_context)

        standard_toilet_body = self.plant.GetBodyByName('toilet_base_link')
        tf_grasspatch_toilet = RigidTransform(
            RollPitchYaw(np.asarray([45, 30, 0]) * np.pi / 180),
            p=[1.0, 0, 0.8])
        tf_world_toilet = tf_world_grasspatch.multiply(tf_grasspatch_toilet)
        self.plant.SetDefaultFloatingBaseBodyPose(
            standard_toilet_body, tf_world_toilet)

    def constant_torques(self):
        """Initialize joint torque inputs."""
        # constant torques
        nu = self.plant.num_actuated_dofs(self.finger)
        u0 = np.ones(nu) * .1
        constant = self.builder.AddSystem(ConstantVectorSource(u0))
        self.builder.Connect(
            constant.get_output_port(0),
            self.plant.get_actuation_input_port(self.finger),
        )

    def build_diagram(self):
        """Build the diagram."""
        self.diagram = self.builder.Build()

    def save_diagram(self):
        """Save the diagram as a png."""
        dot_string = self.diagram.GetGraphvizString()
        graphs = pydot.graph_from_dot_data(dot_string)
        graphs[0].write_png('diagram.png')

    def run(self):
        """Start the simulation."""
        simulator = Simulator(self.diagram)
        simulator.Initialize()
        self.simulator_context = simulator.get_mutable_context()
        simulator.set_target_realtime_rate(0)
        # After plant.Finalize()
        self.diagram.GetMutableSubsystemContext(
            self.plant, self.simulator_context)

        self.plant_context = self.diagram.GetMutableSubsystemContext(
            self.plant, self.simulator_context)

        sim_time = float('inf')
        while self.simulator_context.get_time() < sim_time:
            next_time = min(
                self.simulator_context.get_time() + self.dt,
                sim_time,
            )
            simulator.AdvanceTo(next_time)

            self.debug()

    def debug(self):
        """Debug print messages."""
        # nq = self.plant.num_positions(self.finger)
        # nv = self.plant.num_velocities(self.finger)
        # print(f'finger: nq={nq}, nv={nv}, state size={nq+nv}')
        # print()
        # print('Joints in this model instance:')
        # for i in range(self.plant.num_joints()):
        #     j = self.plant.get_joint(JointIndex(i))
        #     if j.model_instance() != self.finger:
        #         continue
        #     print(f'  {j.name():30s} type={type(j).__name__:20s} '
        #           f'nq={j.num_positions()} nv={j.num_velocities()} '
        #           f'q_start={j.position_start()} v_start={
        #                 j.velocity_start()}')

        # # get contact forces on box
        contact_results = self.plant.get_contact_results_output_port()\
            .Eval(self.plant_context)
        for i in range(contact_results.num_point_pair_contacts()):
            info = contact_results.point_pair_contact_info(i)
            body_A = self.plant.get_body(info.bodyA_index()).name()
            body_B = self.plant.get_body(info.bodyB_index()).name()
            if 'box' in body_A or 'box' in body_B:
                print(f't={self.simulator_context.get_time():.2f} '
                      f'{body_A}<->{body_B} '
                      f'force={info.contact_force()}')

    def create_service(self, node):
        """Create a dummy service to indicate drake has started up."""
        node.create_service(Empty, '/heartbeat', self.service_callback)

    def service_callback(self):
        """Empty callback function for dummy service."""
        pass


def main():
    """Set up and start simulation."""
    # initialize finger simulation class
    fingersim = FingerSimulation()
    fingersim.init_viz(enable_rviz=True)
    fingersim.load_scene()

    # create and init ros node
    rclpy.init(args=None)
    node = rclpy.create_node('drakesim')

    # add external systems
    ros2drake_system = fingersim.builder.AddSystem(Ros2Drake(node))

    motor_torque_to_force_system = fingersim.builder.AddSystem(
        MotorTorqueToForceSystem())

    motor_tension_to_joint_torque_system = fingersim.builder.AddSystem(
        FingerPulleySystem())

    tendon_feedback_system = fingersim.builder.AddSystem(
        TendonFeedbackSystem())

    motor_feedback_system = fingersim.builder.AddSystem(
        MotorFeedbackSystem())
    drake2ros_system = fingersim.builder.AddSystem(Drake2Ros(node))

    fingersim.builder.Connect(
        ros2drake_system.GetOutputPort('motor_torque'),
        motor_torque_to_force_system.GetInputPort('motor_torque'),
    )
    fingersim.builder.Connect(
        motor_torque_to_force_system.GetOutputPort('tendon_tension'),
        motor_tension_to_joint_torque_system.GetInputPort('tendon_tension'),
    )
    fingersim.builder.Connect(
        motor_tension_to_joint_torque_system.GetOutputPort('joint_torque'),
        fingersim.plant.get_actuation_input_port(fingersim.finger),
    )
    fingersim.builder.Connect(
        fingersim.plant.get_state_output_port(fingersim.finger),
        tendon_feedback_system.GetInputPort('finger_state'),
    )
    fingersim.builder.Connect(
        motor_torque_to_force_system.GetOutputPort('tendon_tension'),
        tendon_feedback_system.GetInputPort('tendon_tension'),
    )
    fingersim.builder.Connect(
        tendon_feedback_system.GetOutputPort('tendon_velocity'),
        motor_feedback_system.GetInputPort('tendon_velocity'),
    )
    fingersim.builder.Connect(
        tendon_feedback_system.GetOutputPort('tendon_position'),
        motor_feedback_system.GetInputPort('tendon_position'),
    )
    fingersim.builder.Connect(
        motor_feedback_system.GetOutputPort('motor_velocity'),
        drake2ros_system.GetInputPort('motor_velocity'),
    )
    fingersim.builder.Connect(
        motor_feedback_system.GetOutputPort('motor_position'),
        drake2ros_system.GetInputPort('motor_position'),
    )

    fingersim.build_diagram()
    # fingersim.save_diagram()
    fingersim.create_service(node)
    fingersim.run()


if __name__ == '__main__':
    main()
