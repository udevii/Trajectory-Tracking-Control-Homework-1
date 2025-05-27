import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import csv
import math

class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        self.declare_parameter('csv_file', '/root/ros2_ws/src/trajectory_tracker/trajectory.csv')
        self.csv_file = self.get_parameter('csv_file').get_parameter_value().string_value

        self.trajectory = self.load_trajectory(self.csv_file)
        self.target_index = 0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info('Trajectory Tracker Node Started')

    def load_trajectory(self, path):
        points = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                x, y = float(row[0]), float(row[1])
                points.append((x, y))
        return points

    def odom_callback(self, msg):
        if self.target_index >= len(self.trajectory):
            self.stop_car()
            return

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        target_x, target_y = self.trajectory[self.target_index]

        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

        if distance < 0.2:
            self.get_logger().info(f'Reached waypoint {self.target_index}')
            self.target_index += 1
        else:
            velocity = Twist()
            velocity.linear.x = min(1.0, distance)  # Cap max speed
            self.cmd_pub.publish(velocity)

    def stop_car(self):
        velocity = Twist()
        velocity.linear.x = 0.0
        self.cmd_pub.publish(velocity)
        self.get_logger().info('All waypoints reached. Vehicle stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
