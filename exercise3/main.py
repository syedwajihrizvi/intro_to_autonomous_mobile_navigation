import rclpy
import math
from collections import deque
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from turtlesim.msg import Pose

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.l = 0.15
        self.Kp = 1.0
        self.L_inv = np.array([[1, 0], [0, 1 / self.l]])
        self.current_pose = None
        self.current_pickup_target = None
        self.tolerance = 0.01
        self.rotation_phase = False
        self.goal_reached = False
        self.pick_some_object = False
        self.queue = deque()
        self.target_pose = None
        self.return_to_origin = False
        self._origin_pose = PoseStamped()
        self._origin_pose.header.frame_id = 'map'
        self._origin_pose.pose.position.x = 5.5
        self._origin_pose.pose.position.y = 5.5
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, qos)
        self.create_subscription(PoseStamped, '/pickup_target', self.pickup_target_callback, qos)
        self.picked_up_object_publisher = self.create_publisher(String, '/pick_and_place/reached_object', qos)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', qos)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('PickAndPlaceNode has been started.')

    def pickup_target_callback(self, msg):
        self.get_logger().info(f"Received new pickup target: {msg}")
        self.queue.append(msg)

    def turtle1_pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            return
        if len(self.queue) > 0 or self.current_pickup_target is not None:
            if self.current_pickup_target is None:
                self.current_pickup_target = self.queue.popleft()
                self.get_logger().info(f"Setting current pickup target to: {self.current_pickup_target}")
            self.target_pose = self.current_pickup_target
            v, w = self.kinematic_model()
            if v == 0.0 and w == 0.0:
                self.get_logger().info('Target Reached')
                self.cmd_vel_publisher.publish(Twist())
                self.picked_up_object_publisher.publish(String(data=self.current_pickup_target.header.frame_id))
                self.current_pickup_target = None
                self.get_logger().info('Ready for next target.')
                return
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_vel_publisher.publish(cmd)
        else:
            self.target_pose = self._origin_pose
            v, w = self.kinematic_model()
            if v == 0.0 and w == 0.0:
                self.get_logger().info('Returned to origin. Stopping the turtle.')
                self.cmd_vel_publisher.publish(Twist())
                return
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Control loop is now running.')

    def get_rotation_matrix(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta),  np.cos(theta)]])

    def kinematic_model(self):
        x, y, theta = self.current_pose.x, self.current_pose.y, self.current_pose.theta
        p_xg, p_yg = self.target_pose.pose.position.x, self.target_pose.pose.position.y

        p_xl = x + self.l * np.cos(theta)
        p_yl = y + self.l * np.sin(theta)

        distance_to_target = np.sqrt((p_xg - p_xl) ** 2 + (p_yg - p_yl) ** 2)
        v, w = 0.0, 0.0

        if distance_to_target > self.tolerance:
            e_x = p_xg - p_xl
            e_y = p_yg - p_yl
            p_dot_x = self.Kp * e_x
            p_dot_y = self.Kp * e_y
            control_inputs = self.L_inv @ self.get_rotation_matrix(theta).transpose() @ np.array([[p_dot_x], [p_dot_y]])
            v, w = control_inputs[0, 0], control_inputs[1, 0]
            v = np.clip(v, -3.0, 3.0)
            w = np.clip(w, -3.0, 3.0)
        return v, w

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
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

    
