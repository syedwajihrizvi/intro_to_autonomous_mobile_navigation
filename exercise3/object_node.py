import math
import random
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from turtlesim.srv import Spawn, SetPen, Kill
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class ObjectPublisherNode(Node):
    def __init__(self):
        super().__init__('object_publisher_node')
        
        self.max_commands = 7
        self.published_count = 0
        self.spawned_positions = []
        self.timer = None
        
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.target_pose_pub = self.create_publisher(PoseStamped, '/pickup_target', qos)
        self.pick_up_target_sub = self.create_subscription(String, '/pick_and_place/reached_object', self.pickup_target_reached_callback, qos)
        self.spawn_client = self.create_client(Spawn, '/spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        self.kill_client = self.create_client(Kill, '/kill')
            
        # Trigger the first spawn immediately via a 0.01s initial timer
        self.schedule_next_spawn(0.01)
        self.get_logger().info('Object Publisher Node initialized.')

    def schedule_next_spawn(self, delay_seconds):
        """Helper to cleanly reset the timer for the next delay."""
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
        self.timer = self.create_timer(delay_seconds, self.timer_callback)

    def get_spread_out_position(self, min_distance=2.5):
        for _ in range(100):
            x = float(np.round(random.uniform(1.0, 10.0), 2))
            y = float(np.round(random.uniform(1.0, 10.0), 2))
            valid = True
            all_points = self.spawned_positions + [(5.5, 5.5)]
            
            for px, py in all_points:
                dist = math.hypot(x - px, y - py)
                if dist < min_distance:
                    valid = False
                    break
            
            if valid:
                self.spawned_positions.append((x, y))
                return x, y
        return float(np.round(random.uniform(1.0, 10.0), 2)), float(np.round(random.uniform(1.0, 10.0), 2))

    def pickup_target_reached_callback(self, msg):
        self.get_logger().info(f"Received pickup target reached notification: {msg.data}")
        self.despawn_object(msg.data)
    
    def timer_callback(self):
        if self.published_count >= self.max_commands:
            self.get_logger().info('Published all 4 object target positions. Stopping timer.')
            if self.timer:
                self.timer.cancel()
            return

        x_pos, y_pos = self.get_spread_out_position(min_distance=2.8)
        theta_angle = float(np.round(random.uniform(-math.pi, math.pi), 2))

        object_name = f'object_{self.published_count + 1}'
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = object_name
        msg.pose.position.x = x_pos
        msg.pose.position.y = y_pos
        msg.pose.position.z = 0.0
        
        self.target_pose_pub.publish(msg)
        self.spawn_object_in_turtlesim(x_pos, y_pos, theta_angle, object_name)

        self.published_count += 1
        self.get_logger().info(f'[{self.published_count}/{self.max_commands}] Published target for {object_name} at ({x_pos}, {y_pos})')

        # Schedule the next spawn with a random delay between 6.0 and 10.0 seconds
        if self.published_count < self.max_commands:
            next_delay = random.uniform(6.0, 10.0)
            self.schedule_next_spawn(next_delay)

    def spawn_object_in_turtlesim(self, x, y, theta, name):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            lambda f: self.disable_object_pen(f.result().name) if f.result() else None
        )

    def disable_object_pen(self, name):
        pen_client = self.create_client(SetPen, f'/{name}/set_pen')
        if pen_client.wait_for_service(timeout_sec=1.0):
            req = SetPen.Request()
            req.off = 1
            pen_client.call_async(req)

    def despawn_object(self, name):
        if not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /kill not available!')
            return

        request = Kill.Request()
        request.name = name

        future = self.kill_client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'Successfully despawned {name}')
        )

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPublisherNode()
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