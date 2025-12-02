#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np

class StoneDetector:
    def __init__(self):
        rospy.init_node("stone_detector", anonymous=True)

        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info = None
        
        # Intrinsic parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # === 1. UPDATED TOPICS (Matches your new URDF) ===
        # We use the standard topics defined in the libgazebo_ros_openni_kinect plugin
        self.rgb_topic = "/camera/color/image_raw"
        self.depth_topic = "/camera/depth/image_rect_raw"
        self.info_topic = "/camera/color/camera_info"

        rospy.Subscriber(self.rgb_topic, Image, self.image_callback)
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        rospy.Subscriber(self.info_topic, CameraInfo, self.camera_info_callback)

        # === 2. UPDATED FRAME ID ===
        # CRITICAL: We must use the OPTICAL frame for 3D math (Z is forward)
        self.camera_frame = "realsense_d435i_depth_optical_frame"

        # === Publishers ===
        self.position_pub = rospy.Publisher("/stone_position", PointStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/stone_marker", Marker, queue_size=1)

        # Debug visualization
        self.show_debug = rospy.get_param("~show_debug", True)
        
        rospy.loginfo("Waiting for camera info...")

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def depth_callback(self, msg):
        try:
            # The depth image is a float32 map of meters
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            rospy.logerr(f"Depth error: {e}")

    def image_callback(self, msg):
        if self.depth_image is None or self.camera_info is None:
            return

        try:
            color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"RGB error: {e}")
            return

        # === 3. UPDATED COLOR DETECTION (RED STONE) ===
        hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
        
        # Red is tricky in HSV because it wraps around 0 and 180.
        # We define two ranges and combine them.
        
        # Lower Red (0-10)
        lower_red1 = np.array([0, 100, 50])
        upper_red1 = np.array([10, 255, 255])
        
        # Upper Red (170-180)
        lower_red2 = np.array([170, 100, 50])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Clean up noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        if self.show_debug:
            cv2.imshow("Stone Mask", mask)
            cv2.waitKey(1)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Filter small noise
            if cv2.contourArea(largest_contour) > 500:
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Center of the stone in the image (u, v)
                cx_pixel = x + w // 2
                cy_pixel = y + h // 2

                # Get the Depth (Z) at this pixel
                # We clamp coordinates to ensure we don't go out of bounds
                h_img, w_img = self.depth_image.shape
                cx_safe = np.clip(cx_pixel, 0, w_img - 1)
                cy_safe = np.clip(cy_pixel, 0, h_img - 1)
                
                Z_metric = self.depth_image[cy_safe, cx_safe]

                # Filter invalid depth readings (0.0 or NaN or too far)
                if 0.1 < Z_metric < 2.0:
                    # 3D Deprojection Math: (u - cx) * Z / fx
                    X_metric = (cx_pixel - self.cx) * Z_metric / self.fx
                    Y_metric = (cy_pixel - self.cy) * Z_metric / self.fy
                    
                    # Publish Results
                    self.publish_point(X_metric, Y_metric, Z_metric, msg.header)
                    self.publish_marker(X_metric, Y_metric, Z_metric)

                    # Draw Box on Image
                    cv2.rectangle(color_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(color_img, (cx_pixel, cy_pixel), 5, (0, 0, 255), -1)
                    cv2.putText(color_img, f"X:{X_metric:.2f} Y:{Y_metric:.2f} Z:{Z_metric:.2f}", 
                               (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if self.show_debug:
            cv2.imshow("Stone Tracker", color_img)
            cv2.waitKey(1)

    def publish_point(self, x, y, z, header):
        pt = PointStamped()
        pt.header = header
        pt.header.frame_id = self.camera_frame # Uses the Optical Frame
        pt.point.x = x
        pt.point.y = y
        pt.point.z = z
        self.position_pub.publish(pt)

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = self.camera_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "stone"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0 
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

if __name__ == "__main__":
    try:
        StoneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()