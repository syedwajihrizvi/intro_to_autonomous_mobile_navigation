import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from turtlesim.msg import Pose

class NavigateToPointNodeAndAlign(Node):
    def __init__(self):
        super().__init__('navigate_to_point_node_and_align')
        self.l = 0.15
        self.Kp = 1.0
        self.L_inv = np.array([[1, 0], [0, 1 / self.l]])
        self.current_pose = None
        self.tolerance = 0.01
        self.rotation_phase = False
        self.goal_reached = False
        target_pose = PoseStamped()
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = 10.0
        target_pose.pose.position.y = 10.0
        target_pose.pose.position.z = 0.0

        target_yaw = 120*math.pi/180  # Change to math.pi / 4.0 for 45 degrees
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        target_pose.pose.orientation.w = math.cos(target_yaw / 2.0)
        
        self.target_pose = target_pose

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, qos)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', qos)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('NavigateToPointNodeAndAlign has been started.')

    def turtle1_pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            return
        if self.goal_reached:
            self.get_logger().info('Goal already reached. Stopping the turtle.')
            self.cmd_vel_publisher.publish(Twist())
            return
        v, w = self.kinematic_model()
        if v == 0.0 and w == 0.0:
            self.get_logger().info('Target reached. Stopping the turtle.')
            self.cmd_vel_publisher.publish(Twist())
            self.rotation_phase = False
            self.goal_reached = True
            return
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(f'Current Pose: {self.current_pose}')

    def get_rotation_matrix(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta),  np.cos(theta)]])

    def get_yaw_from_quaternion(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        return np.arctan2(siny_cosp, cosy_cosp)

    def kinematic_model(self):
        x, y, theta = self.current_pose.x, self.current_pose.y, self.current_pose.theta
        p_xg, p_yg = self.target_pose.pose.position.x, self.target_pose.pose.position.y
        target_theta = self.get_yaw_from_quaternion(self.target_pose.pose.orientation)
        p_xl = x + self.l * np.cos(theta)
        p_yl = y + self.l * np.sin(theta)

        distance_to_target = np.sqrt((p_xg - p_xl) ** 2 + (p_yg - p_yl) ** 2)
        angle_error = target_theta - theta
        angle_error = np.arctan(np.sin(angle_error) / np.cos(angle_error))
        v, w = 0.0, 0.0

        if distance_to_target > self.tolerance and not self.rotation_phase:
            e_x = p_xg - p_xl
            e_y = p_yg - p_yl
            p_dot_x = self.Kp * e_x
            p_dot_y = self.Kp * e_y
            control_inputs = self.L_inv @ self.get_rotation_matrix(theta).transpose() @ np.array([[p_dot_x], [p_dot_y]])
            v, w = control_inputs[0, 0], control_inputs[1, 0]
            self.get_logger().info('Translation in progress.')
        else:
            self.rotation_phase = True
            self.get_logger().info('Position reached, aligning orientation.')
            if abs(angle_error) > 0.01:
                w = self.Kp * angle_error
            else:
                self.get_logger().info('Target reached and aligned. Stopping the turtle.')
                v, w = 0.0, 0.0
        v = np.clip(v, -5.0, 5.0)
        w = np.clip(w, -3.0, 3.0)
        return v, w
    
def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPointNodeAndAlign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()