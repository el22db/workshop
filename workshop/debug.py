import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

# Fix for Wayland display issues
os.environ["QT_QPA_PLATFORM"] = "xcb"

class MultiCameraSubscriber(Node):
    def __init__(self):
        super().__init__('multi_camera_subscriber')
        self.bridge = CvBridge()
        self.last_color_positions = {}

        width = 320
        height = 240
        fov_x = 1.7  # radians

        self.fx = width / (2 * math.tan(fov_x / 2))
        self.fy = self.fx
        self.cx = width / 2
        self.cy = height / 2

        self.camera_poses = {
            "Table Camera": ((0.0, -2.0, 2.0), (0.0, 0.5, 1.57)),
            "Shelf Camera": ((-1.5, -2.3, 1.45), (0.0, 0.5, 1.57)),
            "Room Camera": ((-0.5, 1.3, 2.0), (0.0, 0.5, -1.57)),
        }
        self.surface_dimensions = {
            "Table": (1.2, 0.8, 0.75),      # width, depth, height (in meters)
            "Shelf": (1.5, 0.5, 2.0),       # width, depth, height (in meters)
        }
        self.furniture_poses = {
            "Table": ((0.0, -0.5, 0.0), (0.0, 0.0, 1.57)),      # (x, y, z), (roll, pitch, yaw)
            "Shelf": ((-1.5, -1.9, 0.05), (0.0, 0.0, 0.0)),
        }

        # RGB camera from the table
        self.table_sub = self.create_subscription(
            Image,
            '/table_camera/rgb_camera_table/image_raw',
            self.table_callback,
            10
        )

        # RGB camera from the shelf
        self.corner_sub = self.create_subscription(
            Image,
            '/shelf_camera/rgb_camera_shelf/image_raw',
            self.corner_callback,
            10
        )

        # Room RGB camera (corrected topic)
        self.room_sub = self.create_subscription(
            Image,
            '/room_camera/room_camera/image_raw',
            self.room_callback,
            10
        )

    def table_callback(self, msg):
        self.display_image(msg, "Table Camera")

    def corner_callback(self, msg):
        self.display_image(msg, "Shelf Camera")

    def room_callback(self, msg):
        self.display_image(msg, "Room Camera")

    def display_image(self, msg, window_name):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            color_ranges = {
                "Red": [(0, 50, 50), (15, 255, 255), (165, 50, 50), (180, 255, 255)],
                "Green": [(35, 50, 50), (85, 255, 255)],
                "Blue": [(90, 50, 50), (130, 255, 255)],
                "Yellow": [(20, 100, 100), (40, 255, 255)],
            }

            for color, bounds in color_ranges.items():
                if color == "Red":
                    lower1, upper1, lower2, upper2 = bounds
                    mask1 = cv2.inRange(hsv, lower1, upper1)
                    mask2 = cv2.inRange(hsv, lower2, upper2)
                    mask = cv2.bitwise_or(mask1, mask2)
                else:
                    lower, upper = bounds
                    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area > 100:
                        M = cv2.moments(cnt)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            self.last_color_positions[color] = (cx, cy)

                            cv2.drawContours(cv_image, [cnt], -1, (0, 255, 0), 2)
                            cv2.circle(cv_image, (cx, cy), 5, (255, 255, 255), -1)
                            cv2.putText(cv_image, f"{color} ({cx},{cy})", (cx + 10, cy),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

            cv2.imshow(window_name, cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing {window_name}: {e}")

    def deproject_pixel_to_point(self, x, y, depth):
        X = (x - self.cx) * depth / self.fx
        Y = (y - self.cy) * depth / self.fy
        Z = depth
        return (X, Y, Z)

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    



import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    models_path = os.path.expanduser("~/ros2_ws/src/workshop/urdf")
    shelf_z = 1.25
    world_name = 'workshop'

    return LaunchDescription([
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                        '-file', os.path.join(models_path, 'box.urdf'),
                        '-entity', 'test_box',
                        '-x', '-1.3', '-y', '-0.8', '-z', '0.25'
                    ],
                    output='screen'
                ),
            ]
        )
    ])
