#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_custom_interfaces.msg import Status
from std_msgs.msg import String
from enum import Enum


class RobotState(Enum):
    IDLE = 0
    FORWARD = 1
    TURN = 2


class Figure(Enum):
    SQUARE = 0
    RECTANGLE = 1
    TRIANGLE = 2
    DIAMOND = 3
    DONE = 4


class TurtleFigureDrawer(Node):
    def __init__(self):
        super().__init__('turtle_figure_drawer')
        
        self.pose = None
        self.goal = Pose()
        self.state = RobotState.IDLE
        self.figure = Figure.SQUARE
        self.figure_step = 0
        self.can_go_forward = True
        self.offset_applied = False

        # Subscribers & Publishers
        self.pose_subscription = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.cmd_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.status_publisher = self.create_publisher(Status, '/turle1/status', 10)

        # Timers
        self.send_command_timer = self.create_timer(0.02, self.draw_figures_sequence)
        self.send_status_timer = self.create_timer(0.1, self.send_status)

    def pose_callback(self, msg):
        self.pose = msg

    def send_status(self):
        status = Status()
        state_to_string = {
            RobotState.IDLE: "idle",
            RobotState.FORWARD: "forward",
            RobotState.TURN: "turn"
        }
        status.status = String(data=state_to_string.get(self.state, "unknown"))
        self.status_publisher.publish(status)

    def send_command(self, linear_velocity: float, angular_velocity: float):
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_publisher.publish(twist)

    def stop(self):
        self.state = RobotState.IDLE
        self.send_command(0.0, 0.0)

    def hasReachedGoal(self) -> bool:
        x_diff = abs(self.goal.x - self.pose.x)
        y_diff = abs(self.goal.y - self.pose.y)
        theta_diff = abs(self.normalize_angle(self.goal.theta - self.pose.theta))
        return x_diff < 0.01 and y_diff < 0.01 and theta_diff < 0.01

    def compute_goal(self, distance: float, angle: float):
        self.goal.x = math.cos(self.pose.theta) * distance + self.pose.x
        self.goal.y = math.sin(self.pose.theta) * distance + self.pose.y
        self.goal.theta = self.normalize_angle(self.pose.theta + angle)

    def normalize_angle(self, angle: float):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def move_forward(self, linear_velocity: float, distance: float) -> bool:
        if self.state == RobotState.IDLE:
            self.state = RobotState.FORWARD
            self.compute_goal(distance, 0.0)
        if self.hasReachedGoal():
            self.stop()
            return False
        self.send_command(linear_velocity, 0.0)
        return True

    def turn_around(self, angular_velocity: float, angle: float) -> bool:
        if self.state == RobotState.IDLE:
            self.state = RobotState.TURN
            self.compute_goal(0.0, angle)
        if self.hasReachedGoal():
            self.stop()
            return False
        self.send_command(0.0, angular_velocity)
        return True

    def draw_figures_sequence(self):
        if not self.pose or self.figure == Figure.DONE:
            return

        # Décalage avant chaque nouvelle figure (sauf la première)
        if not self.offset_applied and self.figure != Figure.SQUARE:
            if self.move_forward(0.5, 2.0):
                return
            else:
                self.offset_applied = True
                return

        if self.figure == Figure.SQUARE:
            self.draw_regular_polygon(4, 3.0)
        elif self.figure == Figure.RECTANGLE:
            self.draw_rectangle(4.0, 2.0)
        elif self.figure == Figure.TRIANGLE:
            self.draw_regular_polygon(3, 4.0)
        elif self.figure == Figure.DIAMOND:
            self.draw_diamond(3.0, 60)

    def draw_regular_polygon(self, sides, length):
        angle = 2 * math.pi / sides
        self._draw_shape(length, angle, sides)

    def draw_rectangle(self, long_len, short_len):
        sides = [long_len, short_len, long_len, short_len]
        angles = [math.pi / 2] * 4
        self._draw_shape_from_lists(sides, angles)

    def draw_diamond(self, length, angle_deg):
        angle_rad = math.radians(angle_deg)
        sides = [length] * 4
        angles = [angle_rad, math.pi - angle_rad] * 2
        self._draw_shape_from_lists(sides, angles)

    def _draw_shape(self, length, angle, sides):
        if self.figure_step < sides:
            if self.state != RobotState.TURN and self.can_go_forward:
                if not self.move_forward(0.5, length):
                    self.can_go_forward = False
            elif self.state != RobotState.FORWARD and not self.can_go_forward:
                if not self.turn_around(0.5, angle):
                    self.can_go_forward = True
                    self.figure_step += 1
        else:
            self._next_figure()

    def _draw_shape_from_lists(self, lengths, angles):
        if self.figure_step < len(lengths):
            if self.state != RobotState.TURN and self.can_go_forward:
                if not self.move_forward(0.5, lengths[self.figure_step]):
                    self.can_go_forward = False
            elif self.state != RobotState.FORWARD and not self.can_go_forward:
                if not self.turn_around(0.5, angles[self.figure_step]):
                    self.can_go_forward = True
                    self.figure_step += 1
        else:
            self._next_figure()

    def _next_figure(self):
        self.figure_step = 0
        self.offset_applied = False
        self.figure = Figure(self.figure.value + 1) if self.figure.value + 1 < len(Figure) else Figure.DONE
        self.get_logger().info(f"Passing to next figure: {self.figure.name}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleFigureDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
