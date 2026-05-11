"""Plotting code for recorded bags."""

from finger_interfaces.msg import MotorFeedback

import matplotlib.pyplot as plt

import numpy as np

from rclpy.serialization import deserialize_message

import rosbag2_py

JOINT_LABELS = ['splay [rad]', 'mcp_flex [rad]', 'pip/dip_flex [rad]']

TOPICS = {
    'motor_pos_actual_feedback':   ('motor_pos', 'actual'),
    'motor_pos_setpoint_feedback': ('motor_pos', 'setpoint'),
    'actual/joint_feedback':       ('joint_angle', 'actual'),
    'setpoint/joint_feedback':     ('joint_angle', 'setpoint'),
}

data = {
    'motor_pos':   {'actual': [], 'setpoint': []},
    'joint_angle': {'actual': [], 'setpoint': []},
}

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri='src/robotic-finger/finger_recorder/bags/\
finger_bag_20260511_142701',
        storage_id='mcap'),
    rosbag2_py.ConverterOptions('', ''))

while reader.has_next():
    topic, raw, t = reader.read_next()
    if topic in TOPICS:
        group, series = TOPICS[topic]  # must unpack tuple
        msg = deserialize_message(raw, MotorFeedback)
        positions = list(msg.motor_positions)
        if len(positions) == len(JOINT_LABELS):
            data[group][series].append((t / 1e9, positions))

fig, axes = plt.subplots(len(JOINT_LABELS), 2, figsize=(14, 8), sharex=True)

for col, (group, title) in enumerate([('motor_pos', 'Motor Position'),
                                      ('joint_angle', 'Joint Angle')]):

    for i, label in enumerate(JOINT_LABELS):
        ax = axes[i, col]
        for series_name, color in [('actual', 'tab:blue'), ('setpoint', 'tab:orange')]:
            if not data[group][series_name]:
                continue
            filtered = [(t, v) for t, v in data[group][series_name]]
            ts, vals_raw = zip(*filtered)
            vals = np.array(vals_raw)
            ax.plot(ts, vals[:, i], label=series_name, color=color,
                    linestyle='-' if series_name == 'actual' else '--')
        if col == 0:
            ax.set_ylabel(label)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.set_title(title)
    axes[-1, col].set_xlabel('Time (s)')

fig.suptitle('Motor Position & Joint Angle: Setpoint vs Actual')
plt.tight_layout()
plt.show()
