import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.timer = self.create_timer(5, self.publish_initial_pose)

    def publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = -1.0  # 初始位置的 X 坐標
        initial_pose.pose.pose.position.y = -1.0  # 初始位置的 Y 坐標
        initial_pose.pose.pose.position.z = 0.0  # 初始位置的 Z 坐標
        initial_pose.pose.pose.orientation.w = 1.0  # 方向的四元數表示
        # 假設姿態的協方差，需要根據實際情況調整
        initial_pose.pose.covariance = [
            0.25, 0., 0., 0., 0., 0.,
            0., 0.25, 0., 0., 0., 0.,
            0., 0., 0.25, 0., 0., 0.,
            0., 0., 0., 0.06853891945200942, 0., 0.,
            0., 0., 0., 0., 0.06853891945200942, 0.,
            0., 0., 0., 0., 0., 0.1
        ]
        self.publisher_.publish(initial_pose)
        self.get_logger().info('Initial pose published.')

class NavigationPublisher(Node):
    def __init__(self):
        super().__init__('navigation_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(1, self.publish_goal)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = 0.24  # 目標位置的 X 坐標
        goal.pose.position.y = 0.70  # 目標位置的 Y 坐標
        goal.pose.position.z = 0.0  # 目標位置的 Z 坐標
        goal.pose.orientation.w = 0.707  # 方向的四元數表示
        goal.pose.orientation.z = 0.707  # 方向的四元數表示
        self.publisher_.publish(goal)
        self.get_logger().info('Publishing: "%s"' % goal)


def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()
    rclpy.spin_once(initial_pose_publisher)
    initial_pose_publisher.destroy_node()
    navigation_publisher = NavigationPublisher()
    rclpy.spin_once(navigation_publisher)
    rclpy.spin_once(navigation_publisher)
    navigation_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()