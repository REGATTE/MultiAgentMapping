import rclpy
import rclpy.duration, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import read_points_numpy, create_cloud

from tf2_ros import Buffer, TransformListener

import numpy as np
from scipy.spatial.transform import Rotation as R

class Explore(Node):
    def __init__(self):
        super().__init__('explore')

        # Set QoS policy for TF to keep all history
        self.tf_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=self.tf_qos)

        self.obstacle_threshold = 2.0 # Distance to consider an obstacle in meters
        self.z_min = 0.1              # Minimum height to consider an obstacle
        self.z_max = 0.5              # Maximum height to consider an obstacle
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.rotating = False
        self.paused = False           # Flag to pause robot processing

        self.declare_parameter('namespace', 'scout_x_x')
        namespace = self.get_parameter('namespace').value

        self.cmd_vel_pub_topic = f'/{namespace}/cmd_vel'
        self.pointcloud_sub_topic = f'/{namespace}/scan3D'

        self.cmd_vel_pub_ = self.create_publisher(Twist, self.cmd_vel_pub_topic, 10)
        self.pointcloud_sub_ = self.create_subscription(PointCloud2, self.pointcloud_sub_topic, self.pointcloud_callback, 10)

        self.pause_timer = None     # Time to manage pause duration
        self.rotation_timer = None  # Time to manage rotation duration

    def transform_pointcloud(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().info(f"Could not get transform for point cloud: {e}")
            return None

        t = np.eye(4)
        transform_rotation = transform.transform.rotation
        transform_translation = transform.transform.translation
        t[:3, :3] = R.from_quat([transform_rotation.x, transform_rotation.y, transform_rotation.z, transform_rotation.w]).as_matrix()
        t[:3, 3] = [transform_translation.x, transform_translation.y, transform_translation.z]

        points_msg = read_points_numpy(msg)
        points_map = np.ones((len(points_msg), 4))
        points_map[:, :3] = points_msg
        points_map = np.dot(t, points_map.T).T

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pcl_map_header = msg.header
        pcl_map_header.frame_id = 'LiDAR'
        pcl_map = create_cloud(pcl_map_header, fields, points_map[:, :3])
        return pcl_map
    
    def static_transform_pointcloud(self, msg):
        # Static transform: LiDAR is at (0, 3.6, 0) relative to base_link
        static_transform = np.eye(4)
        static_transform[1, 3] = 3.6  # y-offset of 3.6 meters

        # Read the point cloud and apply the static transform
        points_msg = read_points_numpy(msg)
        points_map = np.ones((len(points_msg), 4))
        points_map[:, :3] = points_msg
        points_map = np.dot(static_transform, points_map.T).T

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pcl_map_header = msg.header
        pcl_map_header.frame_id = 'LiDAR'
        pcl_map = create_cloud(pcl_map_header, fields, points_map[:, :3])
        return pcl_map
    
    def pointcloud_callback(self, msg):
        # If paused, do nothing
        if self.paused:
            return

        # Reset obstacle flags
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False

        # Transform the point cloud
        pcl = self.transform_pointcloud(msg)

        if pcl is not None:
            points_array = read_points_numpy(pcl)
            self.get_logger().info(f"Received point cloud with {len(points_array)} points")

            # Minimum points threshold to avoid false positives
            min_points_threshold = 20
            front_count, left_count, right_count = 0, 0, 0

            # Check for obstacles in the relevant regions
            for point in points_array:
                x, y, z = point[:3]

                # Consider only points within the relevant height range
                if z < self.z_min or z > self.z_max:
                    continue

                # Front obstacle detection
                if x > 0.2 and x < self.obstacle_threshold:
                    if abs(y) < 0.5:  # Narrow band for front detection
                        front_count += 1
                    elif y > 0.5 and y < 1.5:  # Left region
                        if x > 0.1:  # Ignore stray points
                            left_count += 1
                    elif y < -0.5 and y > -1.5:  # Right region
                        if x > 0.1:  # Ignore stray points
                            right_count += 1

            # Set obstacle flags based on thresholds
            if front_count > min_points_threshold:
                self.front_obstacle = True
            if left_count > min_points_threshold:
                self.left_obstacle = True
            if right_count > min_points_threshold:
                self.right_obstacle = True

            # Log the obstacle detection results
            self.get_logger().info(f"Obstacle flags - Front: {self.front_obstacle}, Left: {self.left_obstacle}, Right: {self.right_obstacle}")


            # Publish Twist message based on obstacle detection
            twist_msg = Twist()

            if not self.front_obstacle and not self.left_obstacle and not self.right_obstacle:
                # No obstacle, move forward
                twist_msg.linear.x = 0.5
                twist_msg.angular.z = 0.0
                self.rotating = False
                self.get_logger().info("No obstacles detected. Moving forward.")
            elif self.front_obstacle:
                # Obstacle in front, rotate and pause
                twist_msg.linear.x = 0.0
                if self.left_obstacle and not self.right_obstacle:
                    twist_msg.angular.z = -2.0  # Rotate right if left is blocked
                    self.get_logger().info("Obstacles on left and front. Turning right.")
                elif self.right_obstacle and not self.left_obstacle:
                    twist_msg.angular.z = 2.0  # Turn left if right is blocked
                    self.get_logger().info("Obstacles on right and front. Turning left.")
                else:
                    twist_msg.angular.z = 2.0  # Default: turn left
                    self.get_logger().info("Obstacle in front. Rotating left.")

                self.rotating = True
                self.paused = True  # Set paused state
                self.cmd_vel_pub_.publish(twist_msg)

                # Start rotation timer for 1 second and pause for 5 seconds
                if self.rotation_timer is None:  # Avoid creating multiple timers
                    self.rotation_timer = self.create_timer(1.0, self.stop_rotation)
            else:
                # General case: cautiously move forward
                twist_msg.linear.x = 0.5
                twist_msg.angular.z = 0.0
                self.rotating = False
                self.get_logger().info("General obstacle detected. Moving forward cautiously.")

            # Publish the Twist command
            self.cmd_vel_pub_.publish(twist_msg)
        else:
            self.get_logger().info("Could not transform point cloud")
    
    def stop_rotation(self):
        """Callback to stop rotation after 1 second."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub_.publish(twist_msg)
        self.get_logger().info("Rotation stopped after 1 second. Starting 5-second pause.")

        # Cancel the rotation timer
        if self.rotation_timer:
            self.rotation_timer.cancel()
            self.rotation_timer = None
        if self.pause_timer is None:
            self.pause_timer = self.create_timer(4.0, self.resume_after_pause)
        
        # Check if no obstacles are present and start moving forward
        if not self.front_obstacle and not self.left_obstacle and not self.right_obstacle:
            twist_msg.linear.x = 0.5  # Move forward
            twist_msg.angular.z = 0.0
            self.get_logger().info("No obstacles detected. Moving forward.")
            self.cmd_vel_pub_.publish(twist_msg)

    def resume_after_pause(self):
        """Callback to resume robot processing after pause."""
        self.paused = False
        self.get_logger().info("Resuming after 5-second pause.")
        if self.pause_timer:
            self.pause_timer.cancel()  # Cancel the timer
            self.pause_timer = None


def main(args=None):
    rclpy.init(args=args)

    explore = Explore()

    rclpy.spin(explore)

    explore.destroy_node()
    rclpy.shutdown()