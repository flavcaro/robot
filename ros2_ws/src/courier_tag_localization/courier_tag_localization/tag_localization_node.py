#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool
import cv2
import numpy as np

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    from dt_apriltags import Detector
except ImportError:
    Detector = None


class TagLocalizationNode(Node):
    def __init__(self):
        super().__init__('tag_localization_node')
        
        # Check dependencies
        if Detector is None:
            self.get_logger().warn("⚠️  dt-apriltags not found. Install: pip install dt-apriltags")
            return
        
        if CvBridge is None:
            self.get_logger().warn("⚠️  cv_bridge not found")
            return
        
        # AprilTag detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        
        # Camera parameters (adjusted for simulation)
        self.camera_params = [320, 320, 320, 240]  # fx, fy, cx, cy
        self.tag_size = 0.3  # 30cm tags
        
        # Detection thresholds for efficiency
        self.detection_distance_threshold = 1.5  # meters - only detect tags within 1.5m
        self.detection_angle_threshold = 0.8    # radians - only detect tags in front
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.tag_id_pub = self.create_publisher(Int32, '/detected_tag_id', 10)
        self.at_start_pub = self.create_publisher(Bool, '/at_start_tag', 10)
        self.at_goal_pub = self.create_publisher(Bool, '/at_goal_tag', 10)
        self.tag_pose_pub = self.create_publisher(PoseStamped, '/tag_pose', 10)
        
        # Known tag poses (world coordinates) and their meanings
        self.known_tags = {
            0: {
                'name': 'START',
                'position': (0.5, 0.5, 0.0),
                'color': 'GREEN',
                'action': 'departure'
            },
            1: {
                'name': 'GOAL', 
                'position': (5.5, 5.5, 0.0),
                'color': 'BLUE',
                'action': 'grab_package'
            }
        }
        
        # State tracking
        self.last_detected_tag = None
        self.detection_count = {}  # Track consecutive detections for reliability
        self.required_detections = 3  # Need 3 consecutive detections to confirm
        
        self.get_logger().info("✅ AprilTag Localization Node started (Efficient Mode)")
        self.get_logger().info(f"   Detection range: {self.detection_distance_threshold}m")
        self.get_logger().info(f"   Known tags:")
        for tag_id, info in self.known_tags.items():
            self.get_logger().info(f"      Tag {tag_id}: {info['name']} ({info['color']}) - Action: {info['action']}")
    
    def is_tag_in_view(self, tag_pose_t):
        """Check if tag is within detection range and angle"""
        # Calculate distance
        distance = float(np.linalg.norm(tag_pose_t))
        
        # Check if within distance threshold
        if distance > self.detection_distance_threshold:
            return False, distance
        
        # Check if tag is roughly in front (z-axis positive means forward)
        z_distance = float(tag_pose_t[2][0])
        if z_distance < 0.1:  # Tag is behind or too close
            return False, distance
            
        return True, distance
    
    def confirm_detection(self, tag_id):
        """Require multiple consecutive detections to confirm"""
        if tag_id not in self.detection_count:
            self.detection_count[tag_id] = 0
        
        self.detection_count[tag_id] += 1
        
        # Reset counts for other tags
        for other_id in list(self.detection_count.keys()):
            if other_id != tag_id:
                self.detection_count[other_id] = 0
        
        return self.detection_count[tag_id] >= self.required_detections
    
    def image_callback(self, msg):
        """Process camera images and detect AprilTags efficiently"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to grayscale for detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect AprilTags
            tags = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self.camera_params,
                tag_size=self.tag_size
            )
            
            # Process detected tags
            for tag in tags:
                if tag.tag_id not in self.known_tags:
                    continue
                
                # Check if tag is in valid detection range
                in_view, distance = self.is_tag_in_view(tag.pose_t)
                
                if not in_view:
                    continue
                
                # Confirm detection with multiple frames
                if not self.confirm_detection(tag.tag_id):
                    continue
                
                tag_info = self.known_tags[tag.tag_id]
                
                # Only log if this is a new detection
                if self.last_detected_tag != tag.tag_id:
                    self.get_logger().info(
                        f"🏷️  Detected {tag_info['color']} Tag {tag.tag_id} ({tag_info['name']}) "
                        f"at {distance:.2f}m - Action: {tag_info['action']}"
                    )
                    self.last_detected_tag = tag.tag_id
                
                # Publish detected tag ID
                tag_id_msg = Int32()
                tag_id_msg.data = tag.tag_id
                self.tag_id_pub.publish(tag_id_msg)
                
                # Publish specific location flags
                if tag.tag_id == 0:  # START tag
                    at_start_msg = Bool()
                    at_start_msg.data = True
                    self.at_start_pub.publish(at_start_msg)
                    self.get_logger().info("✅ Confirmed at START location", throttle_duration_sec=2.0)
                
                elif tag.tag_id == 1:  # GOAL tag
                    at_goal_msg = Bool()
                    at_goal_msg.data = True
                    self.at_goal_pub.publish(at_goal_msg)
                    self.get_logger().info("✅ Confirmed at GOAL location - Ready to grab!", throttle_duration_sec=2.0)
                
                # Publish tag pose for localization
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera'
                pose_msg.pose.position.x = float(tag.pose_t[0][0])
                pose_msg.pose.position.y = float(tag.pose_t[1][0])
                pose_msg.pose.position.z = float(tag.pose_t[2][0])
                
                # Convert rotation matrix to quaternion (simplified)
                pose_msg.pose.orientation.w = 1.0
                
                self.tag_pose_pub.publish(pose_msg)
            
            # Reset last detected tag if no tags found
            if len(tags) == 0:
                self.last_detected_tag = None
                # Decay detection counts
                for tag_id in self.detection_count:
                    self.detection_count[tag_id] = max(0, self.detection_count[tag_id] - 1)
                    
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = TagLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/tag_pose', 10)
        
        # Known tag poses (world coordinates)
        self.known_tags = {
            0: (0.5, 0.5, 0.0),    # Start (green)
            1: (5.5, 5.5, 0.0),    # Goal (blue)
        }
        
        self.get_logger().info("✅ AprilTag Localization Node started")
        self.get_logger().info(f"   Known tags: {list(self.known_tags.keys())}")
    
    def image_callback(self, msg):
        """Process camera images and detect AprilTags"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect AprilTags
            tags = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self.camera_params,
                tag_size=self.tag_size
            )
            
            for tag in tags:
                if tag.tag_id in self.known_tags:
                    self.get_logger().info(
                        f"🏷️  Detected tag {tag.tag_id} at distance {tag.pose_t[2][0]:.2f}m",
                        throttle_duration_sec=2.0
                    )
                    
                    # Publish tag pose
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = 'camera'
                    pose_msg.pose.position.x = float(tag.pose_t[0][0])
                    pose_msg.pose.position.y = float(tag.pose_t[1][0])
                    pose_msg.pose.position.z = float(tag.pose_t[2][0])
                    
                    self.pose_pub.publish(pose_msg)
                    
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = TagLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
