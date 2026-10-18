"""
Ground truth recorder: records the true pose of each robot published by Gazebo.

Each model becomes a subfolder: tb3_1 -> <output_dir>/tb3_1/ground_truth.csv
"""
import csv
import math
import os

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rclpy.parameter import Parameter


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GroundTruthRecorder(Node):

    def __init__(self):
        super().__init__(
            'ground_truth_recorder',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.declare_parameter('output_dir', '')
        self.declare_parameter('model_names', ['tb3_1', 'tb3_2', 'tb3_3', 'tb3_4', 'tb3_5'])
        if not self.get_parameter('output_dir').value:
            raise ValueError('set the output_dir parameter')
        output_dir = os.path.expanduser(self.get_parameter('output_dir').value)

        self.outputs = {}  # model -> {'file', 'writer', 'count'}
        for name in self.get_parameter('model_names').value:
            folder = os.path.join(output_dir, name)
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, 'ground_truth.csv')
            # Mode 'x': fails if the file already exists, so a run is never overwritten
            out = {'file': open(path, 'x', newline=''), 'count': 0}
            out['writer'] = csv.writer(out['file'])
            out['writer'].writerow(['t', 'x', 'y', 'yaw'])
            self.outputs[name] = out
            self.get_logger().info(f'Recording the pose of "{name}" -> {path}')
        self.missing = set()

        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_model_states, 10)

    def on_model_states(self, msg):
        # ModelStates has no timestamp: uses the reception time (simulation time)
        t = self.get_clock().now().nanoseconds * 1e-9
        if t == 0.0:
            return
        for name, out in self.outputs.items():
            if name not in msg.name:
                if name not in self.missing:
                    self.get_logger().warn(f'Model "{name}" not found in {msg.name}')
                    self.missing.add(name)
                continue
            pose = msg.pose[msg.name.index(name)]
            out['writer'].writerow([
                f'{t:.6f}',
                f'{pose.position.x:.6f}',
                f'{pose.position.y:.6f}',
                f'{yaw_from_quaternion(pose.orientation):.6f}',
            ])
            out['file'].flush()
            out['count'] += 1

    def close(self):
        for name, out in self.outputs.items():
            out['file'].close()
            print(f'[ground_truth_recorder] {name}: {out["count"]} poses recorded', flush=True)


def main():
    rclpy.init()
    node = GroundTruthRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
