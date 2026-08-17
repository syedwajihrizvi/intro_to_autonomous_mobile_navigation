import rclpy
import math
from scipy.optimize import minimize
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from turtlesim.msg import Pose

class NavigateToPointWithMultipleObstaclesNode(Node):
    def __init__(self):
        super().__init__('navigate_to_point_node_with_multiple_obstacles')
        self.l = 0.15
        self.Kp = 1.0
        self.L_inv = np.array([[1, 0], [0, 1 / self.l]])
        self.current_pose = None
        self.tolerance = 0.01
        self.safety_distance = 0.6
        self.obstacles = {}
        self.gamma_cbf = 1.5
        self.clf_penalty = 5.0
        target_pose = PoseStamped()
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = 1.0
        target_pose.pose.position.y = 10.0
        target_pose.pose.position.z = 0.0
        self.target_pose = target_pose

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(Pose, '/turtle10/pose', self.turtle10_pose_callback, qos)
        self.create_subscription(Pose, '/turtle9/pose', self.turtle9_pose_callback, qos)
        self.create_subscription(Pose, '/turtle8/pose', self.turtle8_pose_callback, qos)
        self.create_subscription(Pose, '/turtle7/pose', self.turtle7_pose_callback, qos)
        self.create_subscription(Pose, '/turtle6/pose', self.turtle6_pose_callback, qos)
        self.create_subscription(Pose, '/turtle5/pose', self.turtle5_pose_callback, qos)
        self.create_subscription(Pose, '/turtle4/pose', self.turtle4_pose_callback, qos)
        self.create_subscription(Pose, '/turtle3/pose', self.turtle3_pose_callback, qos)
        self.create_subscription(Pose, '/turtle2/pose', self.turtle2_pose_callback, qos)
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, qos)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', qos)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('NavigateToPointNode has been started.')

    def turtle1_pose_callback(self, msg):
        self.current_pose = msg

    def turtle2_pose_callback(self, msg):
        self.obstacles['turtle2'] = msg

    def turtle3_pose_callback(self, msg):
        self.obstacles['turtle3'] = msg
    
    def turtle4_pose_callback(self, msg):
        self.obstacles['turtle4'] = msg

    def turtle5_pose_callback(self, msg):
        self.obstacles['turtle5'] = msg

    def turtle6_pose_callback(self, msg):
        self.obstacles['turtle6'] = msg

    def turtle7_pose_callback(self, msg):
        self.obstacles['turtle7'] = msg

    def turtle8_pose_callback(self, msg):
        self.obstacles['turtle8'] = msg

    def turtle9_pose_callback(self, msg):
        self.obstacles['turtle9'] = msg

    def turtle10_pose_callback(self, msg):
        self.obstacles['turtle10'] = msg

    def control_loop(self):
        if self.current_pose is None:
            return
        v, w = self.kinematic_model()
        if v == 0.0 and w == 0.0:
            self.get_logger().info('Target reached. Stopping the turtle.')
            self.cmd_vel_publisher.publish(Twist())
            return
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(f'Current Pose: {self.current_pose}')

    def get_rotation_matrix(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta),  np.cos(theta)]])
    
    def solve_clf_cbf_constraints(self, p_xl, p_yl, p_dot_x, p_dot_y):
        u_nom = np.array([p_dot_x, p_dot_y])
        p_l = np.array([p_xl, p_yl])
        if not self.obstacles:
            return p_dot_x, p_dot_y
        A_cbf = []
        B_cbf = []
        for _, obs_pose in self.obstacles.items():
            p_obs = np.array([obs_pose.x, obs_pose.y])
            dist_sq = np.sum((p_l - p_obs) ** 2)
            h = dist_sq - self.safety_distance ** 2
            grad_h = 2*(p_l - p_obs)
            A_cbf.append(-grad_h)
            B_cbf.append(self.gamma_cbf * h)
        if not A_cbf:
            return p_dot_x, p_dot_y
        A_ub = np.array(A_cbf)
        B_ub = np.array(B_cbf)

        def objective(u):
            return 0.5*np.sum((u - u_nom)**2)
        def objective_grad(u):
            return u - u_nom
        constraints = [{'type': 'ineq', 'fun': lambda u, i=i: B_ub[i] - A_ub[i] @ u} for i in range(len(B_ub))]

        res = minimize(objective, u_nom, jac=objective_grad, constraints=constraints, method='SLSQP')

        if res.success:
            return res.x[0], res.x[1]
        else:
            self.get_logger().warn('CBF QP infeasible! Applying zero velocity.')
            return 0.0, 0.0

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
            p_dot_x, p_dot_y = self.solve_clf_cbf_constraints(p_xl, p_yl, p_dot_x, p_dot_y)
            control_inputs = self.L_inv @ self.get_rotation_matrix(theta).transpose() @ np.array([[p_dot_x], [p_dot_y]])
            v, w = control_inputs[0, 0], control_inputs[1, 0]
            v = np.clip(v, -5.0, 5.0)
            w = np.clip(w, -3.0, 3.0)
        return v, w
    
def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPointWithMultipleObstaclesNode()
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