#!/usr/bin/env python3
"""
Debug script to visualize and verify coordinate frame alignment
This script publishes visualization markers to help debug coordinate frame issues
"""
import rclpy
from rclpy.node import Node
import tf2_ros
import math
from geometry_msgs.msg import TransformStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf_transformations

class FrameAlignmentDebug(Node):
    def __init__(self):
        super().__init__('frame_alignment_debug')
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/frame_debug_markers', 10)
        
        # Timer for periodic updates
        self.timer = self.create_timer(1.0, self.publish_debug_markers)
        
        self.get_logger().info("Frame alignment debug node started")
    
    def create_axis_marker(self, frame_id, marker_id, color, scale=1.0):
        """Create an axis marker for a given frame"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frame_axes"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Arrow from origin along X-axis (red for forward direction)
        marker.points = []
        from geometry_msgs.msg import Point
        p1 = Point(); p1.x = 0.0; p1.y = 0.0; p1.z = 0.0
        p2 = Point(); p2.x = scale; p2.y = 0.0; p2.z = 0.0
        marker.points = [p1, p2]
        
        marker.scale.x = 0.1 * scale  # shaft diameter
        marker.scale.y = 0.2 * scale  # head diameter
        marker.scale.z = 0.0
        
        marker.color = color
        marker.color.a = 1.0
        
        return marker
    
    def create_text_marker(self, frame_id, marker_id, text, offset_z=0.5):
        """Create a text marker for frame identification"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frame_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = offset_z
        marker.pose.orientation.w = 1.0
        
        marker.scale.z = 0.3  # text size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.text = text
        return marker
    
    def get_transform_info(self, parent_frame, child_frame):
        """Get transform information between two frames"""
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            )
            
            # Extract yaw angle from quaternion
            q = transform.transform.rotation
            yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            yaw_deg = math.degrees(yaw)
            
            return {
                'translation': transform.transform.translation,
                'rotation': transform.transform.rotation,
                'yaw_rad': yaw,
                'yaw_deg': yaw_deg,
                'valid': True
            }
        except Exception as e:
            return {'valid': False, 'error': str(e)}
    
    def publish_debug_markers(self):
        """Publish debug markers and frame information"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # Define frames to visualize
        frames_to_debug = [
            ('map', ColorRGBA(r=1.0, g=0.0, b=0.0)),     # Red for map
            ('map_raw', ColorRGBA(r=1.0, g=0.5, b=0.0)), # Orange for map_raw
            ('odom', ColorRGBA(r=0.0, g=1.0, b=0.0)),    # Green for odom
            ('base_footprint', ColorRGBA(r=0.0, g=0.0, b=1.0)),  # Blue for base_footprint
        ]
        
        # Create axis markers for each frame
        for frame_name, color in frames_to_debug:
            try:
                # X-axis arrow (forward direction)
                x_marker = self.create_axis_marker(frame_name, marker_id, color, 2.0)
                marker_array.markers.append(x_marker)
                marker_id += 1
                
                # Frame label
                text_marker = self.create_text_marker(frame_name, marker_id, frame_name)
                marker_array.markers.append(text_marker)
                marker_id += 1
                
            except Exception as e:
                self.get_logger().warn(f"Could not create marker for frame {frame_name}: {e}")
        
        # Publish markers
        self.marker_pub.publish(marker_array)
        
        # Log frame relationship information
        relationships = [
            ('map', 'map_raw'),
            ('map', 'odom'),
            ('odom', 'base_footprint'),
            ('map_raw', 'base_footprint'),
        ]
        
        for parent, child in relationships:
            info = self.get_transform_info(parent, child)
            if info['valid']:
                self.get_logger().info(
                    f"{parent} -> {child}: "
                    f"yaw={info['yaw_deg']:.1f}°, "
                    f"x={info['translation'].x:.2f}, "
                    f"y={info['translation'].y:.2f}"
                )
            else:
                self.get_logger().warn(f"No transform: {parent} -> {child}: {info.get('error', 'Unknown')}")

def main():
    rclpy.init()
    node = FrameAlignmentDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()