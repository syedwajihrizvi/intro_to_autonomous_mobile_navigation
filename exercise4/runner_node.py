import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from turtlesim.msg import Pose

class RobberNode(Node):
    def __init__(self):
        super().__init__('robber_node')
        self.current_pose = None

        self.radius = 5.0
        self.linear_speed = 3.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.create_subscription(Pose, '/turtle2/pose', self.pose_callback, qos)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', qos)

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('RobberNode has been started.')

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            return

        w = self.linear_speed / self.radius
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = w

        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobberNode()
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



