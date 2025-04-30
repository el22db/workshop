import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

os.environ["QT_QPA_PLATFORM"] = "xcb"

class MultiCameraSubscriber(Node):
    def __init__(self):
        super().__init__('multi_camera_subscriber')
        self.bridge = CvBridge()
        self.last_color_positions = {}
        self.last_printed_positions = {}  # To store the last printed world positions
        self.position_threshold = 0.1  # Increased threshold (10 cm)
        self.position_stable_frames = 5  # Frames to wait before updating
        self.color_frame_stability = {}  # Track stability of frames for each color

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

        # RGB camera from the table
        self.table_sub = self.create_subscription(
            Image,
            '/table_camera/rgb_camera_table/image_raw',
            self.table_callback,
            10
        )
        self.corner_sub = self.create_subscription(
            Image,
            '/shelf_camera/rgb_camera_shelf/image_raw',
            self.corner_callback,
            10
        )
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
                "Yellow": [(10, 100, 100), (40, 255, 255)],
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
                    x, y, w, h = cv2.boundingRect(cnt)
                    area = w * h
                    aspect_ratio = w / h if h != 0 else 0
                    cv2.drawContours(cv_image, [cnt], -1, (0, 255, 0), 2)

                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(cv_image, (cx, cy), 5, (255, 255, 255), -1)
                        label = f"{color}"

                        if 10 < area < 2000 and 0.4 < aspect_ratio < 2.5 and w > 2 and h > 2:
                            self.last_color_positions[color] = (cx, cy)
                            label += " BLOCK"

                            # If it's from Shelf Camera, estimate world coordinates
                            if window_name == "Shelf Camera":
                                depth = 1.25  # approximate shelf depth
                                cam_x, cam_y, cam_z = self.camera_poses[window_name][0]
                                _, _, cam_yaw = self.camera_poses[window_name][1]

                                # Pixel to camera frame
                                point_cam = self.deproject_pixel_to_point(cx, cy, depth)

                                # Rotate by camera yaw (Z rotation only)
                                x_rot = math.cos(cam_yaw) * point_cam[0] - math.sin(cam_yaw) * point_cam[1]
                                y_rot = math.sin(cam_yaw) * point_cam[0] + math.cos(cam_yaw) * point_cam[1]

                                # Translate to world frame
                                world_x = cam_x + x_rot
                                world_y = cam_y + y_rot
                                world_z = depth  # same as shelf depth

                                # Check if this is the first time we are printing this block's position
                                last_pos = self.last_printed_positions.get(color, None)
                                if last_pos is None:
                                    # Initial print
                                    self.last_printed_positions[color] = (world_x, world_y, world_z)
                                    self.get_logger().info(f"Detected {color} block at world position: [{world_x:.2f}, {world_y:.2f}, {world_z:.2f}]")
                                    self.color_frame_stability[color] = 0  # Reset frame count after initial print
                                else:
                                    # After initial print, check if the position has changed significantly
                                    if self.position_changed(last_pos, (world_x, world_y, world_z), color):
                                        self.last_printed_positions[color] = (world_x, world_y, world_z)
                                        self.color_frame_stability[color] = self.color_frame_stability.get(color, 0) + 1
                                        
                                        # If the position remains stable for a certain number of frames, print it
                                        if self.color_frame_stability[color] >= self.position_stable_frames:
                                            self.get_logger().info(f"Detected {color} block at world position: [{world_x:.2f}, {world_y:.2f}, {world_z:.2f}]")
                                            self.color_frame_stability[color] = 0  # Reset after printing

                        # Optionally, you can still draw labels in the window without coordinates
                        cv2.putText(cv_image, f"{label} ({cx},{cy})", (cx + 10, cy),
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

    def position_changed(self, last_pos, new_pos, color):
        """
        Returns True if the new position differs from the last position by more than the threshold.
        """
        dist = math.sqrt((last_pos[0] - new_pos[0])**2 + (last_pos[1] - new_pos[1])**2 + (last_pos[2] - new_pos[2])**2)
        if dist > self.position_threshold:
            self.color_frame_stability[color] = 0  # Reset frame count if a significant change is detected
        return dist > self.position_threshold

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
