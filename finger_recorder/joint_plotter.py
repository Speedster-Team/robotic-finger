"""Plotting code for recorded bags."""
from finger_interfaces.msg import MotorFeedback
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py

JOINT_LABELS = ['splay [rad]', 'mcp_flex [rad]', 'pip/dip_flex [rad]']

TOPICS = {
    'actual/joint_feedback':   ('joint_angle', 'actual'),
    'setpoint/joint_feedback': ('joint_angle', 'setpoint'),
}

data = {
    'joint_angle': {'actual': [], 'setpoint': []},
}

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri='src/robotic-finger/finger_recorder/bags/finger_bag_20260511_140504',
        storage_id='mcap'),
    rosbag2_py.ConverterOptions('', ''))

while reader.has_next():
    topic, raw, t = reader.read_next()
    if topic in TOPICS:
        group, series = TOPICS[topic]
        msg = deserialize_message(raw, MotorFeedback)
        positions = list(msg.motor_positions)
        if len(positions) == len(JOINT_LABELS):
            data[group][series].append((t / 1e9, positions))

fig, axes = plt.subplots(len(JOINT_LABELS), 1, figsize=(10, 8), sharex=True)

for i, label in enumerate(JOINT_LABELS):
    ax = axes[i]
    for series_name, color in [('actual', 'tab:blue'), ('setpoint', 'tab:orange')]:
        if not data['joint_angle'][series_name]:
            continue
        ts, vals_raw = zip(*data['joint_angle'][series_name])
        vals = np.array(vals_raw)
        ax.plot(ts, vals[:, i], label=series_name, color=color,
                linestyle='-' if series_name == 'actual' else '--')
    ax.set_ylabel(label)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

axes[0].set_title('Joint Angle: Setpoint vs Actual')
axes[-1].set_xlabel('Time (s)')

fig.suptitle('Joint Angle: Setpoint vs Actual')
plt.tight_layout()
plt.show()