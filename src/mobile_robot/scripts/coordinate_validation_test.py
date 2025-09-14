#!/usr/bin/env python3
"""
Test script to validate coordinate frame transformations and GPS alignment
"""
import rclpy
from rclpy.node import Node
import tf2_ros
import math
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf_transformations

class CoordinateValidationTest(Node):
    def __init__(self):
        super().__init__('coordinate_validation_test')
        
        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered_map', self.odom_callback, 10)
        
        # Data storage
        self.latest_gps = None
        self.latest_odom = None
        
        # Test timer
        self.test_timer = self.create_timer(2.0, self.run_validation_tests)
        
        self.get_logger().info("Coordinate validation test started")
    
    def gps_callback(self, msg):
        self.latest_gps = msg
    
    def odom_callback(self, msg):
        self.latest_odom = msg
    
    def gps_to_meters(self, lat, lon, lat0=37.2889339, lon0=127.1076245):
        """Convert GPS coordinates to meters relative to origin"""
        # Approximate conversion for small distances
        lat_rad = math.radians(lat0)
        m_per_deg_lat = 111132.92 - 559.82 * math.cos(2*lat_rad) + 1.175 * math.cos(4*lat_rad)
        m_per_deg_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3*lat_rad)
        
        dx = (lon - lon0) * m_per_deg_lon  # East
        dy = (lat - lat0) * m_per_deg_lat  # North
        
        return dx, dy
    
    def run_validation_tests(self):
        """Run coordinate validation tests"""
        if not self.latest_gps or not self.latest_odom:
            self.get_logger().warn("Waiting for GPS and odometry data...")
            return
        
        # Test 1: GPS vs Odometry position comparison
        gps_x, gps_y = self.gps_to_meters(self.latest_gps.latitude, self.latest_gps.longitude)
        odom_x = self.latest_odom.pose.pose.position.x
        odom_y = self.latest_odom.pose.pose.position.y
        
        pos_error = math.sqrt((gps_x - odom_x)**2 + (gps_y - odom_y)**2)
        
        self.get_logger().info(f"Position Comparison:")
        self.get_logger().info(f"  GPS:  ({gps_x:.2f}, {gps_y:.2f}) m")
        self.get_logger().info(f"  Odom: ({odom_x:.2f}, {odom_y:.2f}) m")
        self.get_logger().info(f"  Error: {pos_error:.2f} m")
        
        # Test 2: Coordinate frame orientations
        try:
            # Check map to base_footprint transform
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            q = transform.transform.rotation
            yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            yaw_deg = math.degrees(yaw)
            
            self.get_logger().info(f"Vehicle Orientation:")
            self.get_logger().info(f"  Map -> base_footprint yaw: {yaw_deg:.1f}°")
            
            # Check if vehicle is facing northwest (should be around 140-160°)
            expected_yaw_range = (120, 180)  # Allow some tolerance
            if expected_yaw_range[0] <= yaw_deg <= expected_yaw_range[1]:
                self.get_logger().info(f"  ✓ Vehicle orientation is correct (northwest)")
            else:
                self.get_logger().warn(f"  ✗ Vehicle orientation may be incorrect (expected {expected_yaw_range}°)")
                
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")
        
        # Test 3: Frame chain validation
        frame_chain = ['utm', 'map_raw', 'map', 'odom', 'base_footprint']
        self.get_logger().info("Frame Chain Validation:")
        
        for i in range(len(frame_chain) - 1):
            parent = frame_chain[i]
            child = frame_chain[i + 1]
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                self.get_logger().info(f"  ✓ {parent} -> {child}: OK")
            except Exception as e:
                self.get_logger().warn(f"  ✗ {parent} -> {child}: {e}")
        
        # Test 4: GPS accuracy validation
        gps_std_xy = math.sqrt(self.latest_gps.position_covariance[0])  # std from covariance
        if gps_std_xy <= 0.2:  # Should be around 0.1m
            self.get_logger().info(f"  ✓ GPS accuracy: {gps_std_xy:.3f} m (good)")
        else:
            self.get_logger().warn(f"  ✗ GPS accuracy: {gps_std_xy:.3f} m (poor)")
        
        # Overall assessment
        if pos_error < 0.5:  # Within 50cm
            self.get_logger().info("✓ Overall coordinate alignment: GOOD")
        elif pos_error < 1.0:  # Within 1m
            self.get_logger().info("⚠ Overall coordinate alignment: ACCEPTABLE")
        else:
            self.get_logger().warn("✗ Overall coordinate alignment: POOR")
        
        self.get_logger().info("-" * 50)

def main():
    rclpy.init()
    node = CoordinateValidationTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()