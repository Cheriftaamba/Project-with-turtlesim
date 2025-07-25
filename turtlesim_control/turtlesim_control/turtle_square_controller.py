#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_custom_interfaces.msg import Status
from enum import Enum
from std_msgs.msg import String


class RobotState(Enum):
    IDLE = 0
    FORWARD = 1
    TURN = 2


class TurtleSquareController(Node):
    def __init__(self):
        super().__init__('turtle_square_controller')
        self.pose_subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        self.pose = None
        self.goal = Pose()
        self.cmd_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.status_publisher = self.create_publisher(Status, '/turle1/status', 10)
        self.send_command_timer = self.create_timer(0.02, self.draw_square)
        self.send_status_timer = self.create_timer(0.1, self.send_status)
        self.state = RobotState.IDLE
        self.counter = 0
        self.can_go_forward = True

    def pose_callback(self, msg):
        self.pose = msg
        
    def send_status(self):
        status = Status()
        state_to_string = {
            RobotState.IDLE: "idle",
            RobotState.FORWARD: "forward",
            RobotState.TURN: "turn"
        }
        status_msg = String(data=state_to_string.get(self.state, "unknow"))
        status.status = status_msg
        self.status_publisher.publish(status)

    def send_command(self, linear_velocity: float, angular_velocity: float):
        twist = Twist()
        twist.linear.x = linear_velocity # m/s
        twist.angular.z = angular_velocity  # rad/s
        self.cmd_publisher.publish(twist)

    def move_forward(self, linear_velocity: float, distance: float) -> bool:
        """"""
        if self.state == RobotState.IDLE:
            self.get_logger().info("Moving forward")
            self.state = RobotState.FORWARD
            self.compute_goal(distance, 0.0)
        if self.hasReachedGoal():
            self.stop()
            return False
        self.send_command(linear_velocity, 0.0)
        return True
    
    def turn_around(self, angular_velocity, angle: float) -> bool:
        if self.state == RobotState.IDLE:
            self.get_logger().info("Turning around")
            self.state = RobotState.TURN
            self.compute_goal(0.0, angle)
        if self.hasReachedGoal():
            self.stop()
            return False
        self.send_command(0.0, angular_velocity)
        return True
    
    def stop(self):
        self.get_logger().info("Robot stopped")
        self.state = RobotState.IDLE
        self.send_command(0.0, 0.0)

    def hasReachedGoal(self) -> bool:
        x_diff = math.fabs(self.goal.x - self.pose.x)
        y_diff = math.fabs(self.goal.y - self.pose.y)
        theta_diff = math.fabs(self.goal.theta - self.pose.theta)
        return x_diff < 0.01 and y_diff < 0.01 and theta_diff < 0.01

    def compute_goal(self, distance: float, angle: float):
        self.goal.x = math.cos(self.pose.theta) * distance + self.pose.x
        self.goal.y = math.sin(self.pose.theta) * distance + self.pose.y
        self.goal.theta = self.normalize_angle(self.pose.theta + angle)

    def draw_square(self):
        if not self.pose:
            return
        if self.counter < 4:
            if self.state != RobotState.TURN and self.can_go_forward:
                is_moving_forward = self.move_forward(0.5, 3.0)
                if not is_moving_forward:
                    self.can_go_forward = False
            elif self.state != RobotState.FORWARD and not self.can_go_forward:
                is_turning_aroud = self.turn_around(0.5, math.pi / 2)
                if not is_turning_aroud:
                    self.can_go_forward = True
                    self.counter = self.counter + 1
                
    def normalize_angle(self, angle: float):
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    turtle_square_controller = TurtleSquareController()
    rclpy.spin(turtle_square_controller)
    turtle_square_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
