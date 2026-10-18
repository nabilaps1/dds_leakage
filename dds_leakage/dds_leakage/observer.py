"""
Observer: a regular DDS participant that discovers the robots' topics and records everything.

It gets no configuration about the robots: it periodically queries the ROS graph and subscribes
to every known topic (STREAMS table) that shows up with the expected type. Each robot (namespace)
becomes a subfolder and each topic a file:
/tb3_1/cmd_vel -> <output_dir>/tb3_1/observed_data.csv
/tb3_1/scan    -> <output_dir>/tb3_1/observed_scan.csv
"""
import csv
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


# Last name of the topic -> how to subscribe and record it. 'header' is the list of columns or,
# when it depends on the message (the number of scan beams), a function applied to the first
# message. 'row' receives the reception time and the message.
STREAMS = {
    'cmd_vel': {
        'type': Twist,
        'file': 'observed_data.csv',
        'qos': QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        'header': ['t', 'linear_x', 'angular_z'],
        'row': lambda t, msg: [f'{t:.6f}', msg.linear.x, msg.angular.z],
    },
    # Gazebo's laser plugin publishes with best effort QoS: a reliable subscriber would receive
    # nothing. LaserScan has its own timestamp (stamp), recorded next to the reception time.
    'scan': {
        'type': LaserScan,
        'file': 'observed_scan.csv',
        'qos': qos_profile_sensor_data,
        'header': lambda msg: ['t', 'stamp'] + [f'r{i:03d}' for i in range(len(msg.ranges))],
        'row': lambda t, msg: [f'{t:.6f}', f'{stamp_seconds(msg.header):.6f}']
        + [f'{r:.4f}' for r in msg.ranges],
    },
}
TYPE_NAMES = {
    name: f'{spec["type"].__module__.split(".")[0]}/msg/{spec["type"].__name__}'
    for name, spec in STREAMS.items()
}


class Observer(Node):

    def __init__(self):
        super().__init__(
            'observer',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.declare_parameter('output_dir', '')
        self.declare_parameter('discovery_period', 0.2)  # [s] between queries of the ROS graph
        if not self.get_parameter('output_dir').value:
            raise ValueError('set the output_dir parameter')
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.streams = {}  # topic -> {'topic', 'spec', 'file', 'writer', 'count'}

        self.get_logger().info(f'Looking for topics {list(STREAMS)} -> {self.output_dir}')
        self.discover()
        self.create_timer(self.get_parameter('discovery_period').value, self.discover)

    def discover(self):
        # DDS discovery hands the list of topics (and their types) to any participant
        for topic, types in self.get_topic_names_and_types():
            kind = topic.rsplit('/', 1)[1]
            if topic in self.streams or kind not in STREAMS or TYPE_NAMES[kind] not in types:
                continue
            self.start_stream(topic, kind)

    def start_stream(self, topic, kind):
        spec = STREAMS[kind]
        robot = topic.rsplit('/', 1)[0].strip('/')  # '/tb3_1/cmd_vel' -> 'tb3_1'; '/scan' -> ''
        folder = os.path.join(self.output_dir, robot)
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, spec['file'])

        # Mode 'x': fails if the file already exists, so a run is never overwritten
        stream = {'topic': topic, 'spec': spec, 'file': open(path, 'x', newline=''), 'count': 0}
        stream['writer'] = csv.writer(stream['file'])
        if not callable(spec['header']):
            stream['writer'].writerow(spec['header'])
        self.streams[topic] = stream

        # DDS discovery reveals who subscribes to the topic (it may exist before any publisher)
        self.get_logger().info(
            f'Discovered topic: {topic} ({TYPE_NAMES[kind]}), subscribed by '
            f'{self.endpoints(self.get_subscriptions_info_by_topic(topic))} -> {path}')
        self.create_subscription(
            spec['type'], topic, lambda msg, s=stream: self.on_message(s, msg), spec['qos'])

    @staticmethod
    def endpoints(infos):
        return ', '.join(f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in infos) or '?'

    def on_message(self, stream, msg):
        # Uses the reception time (simulation time), the same for every topic
        t = self.get_clock().now().nanoseconds * 1e-9
        if t == 0.0:
            return  # no /clock yet, so the message cannot be timestamped
        if stream['count'] == 0:
            # ... and also who publishes on it
            self.get_logger().info(
                f'{stream["topic"]}: first message, published by '
                f'{self.endpoints(self.get_publishers_info_by_topic(stream["topic"]))}')
            if isinstance(msg, LaserScan):
                self.get_logger().info(
                    f'{stream["topic"]}: {len(msg.ranges)} beams, angle from '
                    f'{msg.angle_min:.4f} to {msg.angle_max:.4f} rad (step '
                    f'{msg.angle_increment:.6f}), range from {msg.range_min:.2f} to '
                    f'{msg.range_max:.2f} m, frame {msg.header.frame_id}')
            if callable(stream['spec']['header']):
                stream['writer'].writerow(stream['spec']['header'](msg))
        stream['writer'].writerow(stream['spec']['row'](t, msg))
        stream['file'].flush()
        stream['count'] += 1

    def close(self):
        for topic, stream in self.streams.items():
            stream['file'].close()
            print(f'[observer] {topic}: {stream["count"]} messages recorded', flush=True)


def main():
    rclpy.init()
    node = Observer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
