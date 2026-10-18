"""
Controller: publishes a deterministic sequence of velocities on cmd_vel.

The topic is relative: with --ros-args -r __ns:=/tb3_1 the controller publishes on /tb3_1/cmd_vel.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter

RATE_HZ = 10.0  # cmd_vel publishing rate (simulation time)


# Each segment is (name, v [m/s], w [rad/s], number of commands). The duration is rounded to a
# whole number of commands and v/w are recomputed to keep the geometry exact.
def _ticks(duration):
    return max(1, round(duration * RATE_HZ))


def line(distance, speed, name='line'):
    n = _ticks(distance / speed)
    return (name, distance / (n / RATE_HZ), 0.0, n)


def arc(radius, angle_deg, speed, name='arc'):
    """Arc of radius `radius`: a positive angle turns left, a negative one turns right."""
    angle = math.radians(angle_deg)
    n = _ticks(radius * abs(angle) / speed)
    w = angle / (n / RATE_HZ)
    return (name, abs(w) * radius, w, n)


def turn(angle_deg, rate_deg_s, name='in-place turn'):
    angle = math.radians(angle_deg)
    n = _ticks(abs(angle) / math.radians(rate_deg_s))
    return (name, 0.0, angle / (n / RATE_HZ), n)


# Simple shapes, each with its own signature on cmd_vel. All of them turn left, at 30°/s for the
# in-place turns, and end at the starting point.

# 0.60 m square: 4 lines (v only) alternating with 4 turns of 90° (w only); ~28 s per lap
SQUARE = [line(0.60, 0.15, 'side'), turn(90, 30, '90° turn')] * 4

# 0.80 m equilateral triangle: 3 lines and 3 longer turns, of 120°; ~28 s per lap
TRIANGLE = [line(0.80, 0.15, 'side'), turn(120, 30, '120° turn')] * 3

# Circle of 0.50 m radius: constant v and w at the same time, in a single block; ~21 s per lap
CIRCLE = [arc(0.50, 360, 0.15, 'circle')]

# 1.50 m up and down: 2 long lines with a 180° turn in between; ~26 s per lap.
# Returns to the starting point, but facing the opposite direction.
UP_DOWN = [line(1.50, 0.15, 'out'), turn(180, 30, '180° turn'), line(1.50, 0.15, 'back')]

# 0.60 m hexagon: 6 lines and 6 short turns, of 60°; ~36 s per lap
HEXAGON = [line(0.60, 0.15, 'side'), turn(60, 30, '60° turn')] * 6

PATHS = {
    'square': SQUARE, 'triangle': TRIANGLE, 'circle': CIRCLE, 'up_down': UP_DOWN,
    'hexagon': HEXAGON,
}


class Controller(Node):

    def __init__(self):
        super().__init__(
            'controller',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.declare_parameter('path', '')          # one of PATHS
        self.declare_parameter('laps', 1)           # number of laps of the path
        self.declare_parameter('start_delay', 2.0)  # [s] of zero commands before the path
        self.declare_parameter('stop_time', 1.0)    # [s] of zero commands after the path

        path = self.get_parameter('path').value
        laps = self.get_parameter('laps').value
        start_ticks = round(self.get_parameter('start_delay').value * RATE_HZ)
        stop_ticks = max(1, round(self.get_parameter('stop_time').value * RATE_HZ))
        if path not in PATHS:
            raise ValueError(f'path must be one of {list(PATHS)}, got: {path!r}')
        segments = PATHS[path]

        # Tick-by-tick plan: one command (v, w, label) per timer call.
        # The label only appears on the first command of each segment and goes to the log.
        self.plan = [(0.0, 0.0, None)] * start_ticks
        for lap in range(1, laps + 1):
            for k, (name, v, w, n) in enumerate(segments, 1):
                label = f'lap {lap}/{laps}, segment {k}/{len(segments)}: {name}'
                self.plan += [(v, w, label)] + [(v, w, None)] * (n - 1)
        self.plan += [(0.0, 0.0, 'final stop')] + [(0.0, 0.0, None)] * (stop_ticks - 1)

        moving = sum(1 for v, w, _ in self.plan if v != 0.0 or w != 0.0)
        self.tick = 0
        self.done = False
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # With use_sim_time, the timer follows the simulation clock (/clock)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.on_timer)
        self.get_logger().info(
            f'Plan with {len(self.plan)} commands, {moving} moving '
            f'({path} x{laps}, {len(self.plan) / RATE_HZ:.1f} s of simulation) '
            f'on {self.publisher.topic_name}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_timer(self):
        if self.tick >= len(self.plan):
            self.done = True
            return
        v, w, label = self.plan[self.tick]
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.publisher.publish(msg)
        if self.tick == 0:
            self.get_logger().info(f'First command at t={self.now():.3f} s (sim)')
        if label:
            self.get_logger().info(f't={self.now():.3f} s (sim): {label}')
        self.tick += 1


def main():
    rclpy.init()
    node = Controller()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(f'Path finished at t={node.now():.3f} s (sim)')
    except KeyboardInterrupt:
        node.publisher.publish(Twist())  # interrupted: make sure the robot stops
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
