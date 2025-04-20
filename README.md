# Longitudinal Trajectory Tracking Control for Autonomous Vehicle (AV)

## Objective:
Your task is to develop a trajectory tracking control (TTC) algorithm for the longitudinal
motion of an AV and testing it in the Gazebo environment. The waypoints, or target points,
on the trajectory will be provided for you. The desired trajectory involves a motion in the x-
direction only, hence does not require any steering. Your position control algorithm will use
the error between the desired vehicle position (or, position reference denoted by each
waypoint) and the actual vehicle position to provide feedback for your TTC algorithm. This
algorithm should calculate a control input to attain the target point. In this case, this control
input is a velocity reference that you pass on to Gazebo, where an internal velocity
controller generates the actual velocity of the AV. Your aim should be,

1. To develop an algorithm to have the AV track the waypoints,
2. To accelerate smoothly towards each target point.
3. To decelerate as the car approaches the target, gradually reducing speed until it stops at the target point.
   
This project will help you understand the principles of longitudinal position control in autonomous vehicles using ROS2, and testing your algorithm’s performance on the Gazebo platform.

## Requirements:

### 1. CSV File with Target Points to obtain the Position Reference:

The autonomous car will follow a trajectory specified in a CSV file. Each row in the CSV file contains the target (x, y) coordinates to which the car should move. Example [trajectory.csv](./trajectory.csv) is given in repository.

CSV format example:
- -
- 2.0, 0.0
- 4.5, 0.0
- 7.2, 0.0

### 2. Odometry Subscription to obtain the Position Feedback:
- The Node must subscribe to the /odom topic to receive real-time position and velocity information.

### 3. ROS2 Node Development:
You need to implement this functionality as a ROS2 node. This means your code should:
- Subscribe to the /odom topic to get the current position of the car.
- Compute the velocity command to be sent to the car.
- Publish the velocity commands (/cmd_vel) to achieve the desired motion.

## Help

### 1. Understanding the /odom Topic

The /odom topic provides odometry data describing the robot’s estimated position and orientation relative to an initial starting position, typically represented by a fixed frame called odom. This information is critical for robot navigation tasks without relying on global positioning. 

The /odom topic contains the following components:

- Header: Includes a timestamp and frame identifier, usually set to **odom**.
- Child Frame ID: Usually **base link**, denoting the robot’s reference
frame.
- Pose: Position and orientation (Quaternion).
- Twist: Linear and angular velocities.

To observe odometry data, use command:
```shell
ros2 topic echo /odom
```
For detailed odometry message structure, visit: [ROS Odometry
Message Documentation](https://docs.ros.org/en/jade/api/nav_msgs/html/msg/Odometry.html)

### 2. Odometry Publisher Example
Below is a simple example of how to publish odometry messages in ROS2:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__(’odometry_publisher’)

        self.publisher = self.create_publisher(Odometry, ’odom’, 10)
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ’odom’
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 0.5
        msg.twist.twist.angular.z = 0.1
        self.publisher.publish(msg)
        self.get_logger().info(’Publishing␣Odometry␣Message’)

def main():
    rclpy.init()
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    rclpy.shutdown()

if __name__ == ’__main__’:
    main()
```

This example shows how to create a ROS2 node that publishes odometry
messages to the /odom topic. The position and velocity are set in the message, and the message is published every 0.1 seconds.