#!/usr/bin/env python3
"""
Wheel odometry calibration and diagnostic tool
This script helps diagnose wheel odometry accuracy issues
"""
import rclpy
from rclpy.node import Node
import math
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose
import tf_transformations

class WheelOdometryDiagnostic(Node):
    def __init__(self):
        super().__init__('wheel_odom_diagnostic')
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.world_pose_sub = self.create_subscription(Pose, '/henes_t870/world_pose', self.world_pose_callback, 10)
        
        # Publisher for test commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Data storage
        self.latest_odom = None
        self.latest_joints = None
        self.latest_world_pose = None
        self.start_time = None
        self.start_odom = None
        self.start_world_pose = None
        
        # Vehicle parameters (from URDF)
        self.wheel_radius = 0.135  # meters
        self.wheel_base = 0.71     # meters
        self.track_width = 0.70    # meters
        
        # Test state
        self.test_running = False
        self.test_distance = 0.0
        
        # Timer for diagnostics
        self.diagnostic_timer = self.create_timer(2.0, self.run_diagnostics)
        
        self.get_logger().info("Wheel odometry diagnostic tool started")
        self.get_logger().info(f"Vehicle params: wheel_radius={self.wheel_radius}m, wheel_base={self.wheel_base}m")
    
    def odom_callback(self, msg):
        self.latest_odom = msg
    
    def joint_callback(self, msg):
        self.latest_joints = msg
    
    def world_pose_callback(self, msg):
        self.latest_world_pose = msg
    
    def calculate_wheel_distance(self, joint_states):
        """Calculate distance traveled based on wheel rotations"""
        if not joint_states or not joint_states.name:
            return None
        
        # Find rear wheel joints (main drive wheels)
        rear_wheel_names = ['wheel1_joint', 'wheel2_joint']  # RR, RL
        wheel_positions = {}
        
        for i, name in enumerate(joint_states.name):
            if name in rear_wheel_names and i < len(joint_states.position):
                wheel_positions[name] = joint_states.position[i]
        
        if len(wheel_positions) < 2:
            return None
        
        # Calculate average wheel rotation
        avg_rotation = sum(wheel_positions.values()) / len(wheel_positions)
        distance = avg_rotation * self.wheel_radius
        
        return distance, wheel_positions
    
    def pose_distance(self, pose1, pose2):
        """Calculate 2D distance between two poses"""
        if not pose1 or not pose2:
            return None
        
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def pose_yaw(self, pose):
        """Extract yaw from pose quaternion"""
        if not pose:
            return None
        
        q = pose.orientation
        return tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    
    def run_diagnostics(self):
        """Run wheel odometry diagnostics"""
        if not self.latest_odom or not self.latest_world_pose:
            self.get_logger().warn("Waiting for odometry and world pose data...")
            return
        
        # Compare odometry vs ground truth
        odom_pos = self.latest_odom.pose.pose.position
        world_pos = self.latest_world_pose.position
        
        pos_error = math.sqrt(
            (odom_pos.x - world_pos.x)**2 + 
            (odom_pos.y - world_pos.y)**2
        )
        
        # Compare orientations
        odom_yaw = self.pose_yaw(self.latest_odom.pose.pose)
        world_yaw = self.pose_yaw(self.latest_world_pose)
        
        if odom_yaw is not None and world_yaw is not None:
            yaw_error = abs(odom_yaw - world_yaw)
            # Normalize to [-pi, pi]
            while yaw_error > math.pi:
                yaw_error -= 2*math.pi
            yaw_error = abs(yaw_error)
            yaw_error_deg = math.degrees(yaw_error)
        else:
            yaw_error_deg = None
        
        # Wheel-based distance calculation
        wheel_info = None
        if self.latest_joints:
            wheel_info = self.calculate_wheel_distance(self.latest_joints)
        
        # Log diagnostics
        self.get_logger().info("=== Wheel Odometry Diagnostics ===")
        self.get_logger().info(f"Position Error: {pos_error:.3f} m")
        if yaw_error_deg is not None:
            self.get_logger().info(f"Orientation Error: {yaw_error_deg:.1f}°")
        
        self.get_logger().info(f"Odometry: ({odom_pos.x:.2f}, {odom_pos.y:.2f})")
        self.get_logger().info(f"Ground Truth: ({world_pos.x:.2f}, {world_pos.y:.2f})")
        
        if wheel_info:
            wheel_distance, wheel_positions = wheel_info
            self.get_logger().info(f"Wheel Distance: {wheel_distance:.2f} m")
            self.get_logger().info(f"Wheel Positions: {wheel_positions}")
        
        # Check odometry covariance
        odom_cov = self.latest_odom.pose.covariance
        if odom_cov and len(odom_cov) >= 36:
            pos_uncertainty = math.sqrt(odom_cov[0] + odom_cov[7])  # x + y variance
            self.get_logger().info(f"Odometry Uncertainty: {pos_uncertainty:.3f} m")
        
        # Assessment
        if pos_error < 0.1:
            status = "EXCELLENT"
        elif pos_error < 0.5:
            status = "GOOD"
        elif pos_error < 1.0:
            status = "ACCEPTABLE"
        else:
            status = "POOR"
        
        self.get_logger().info(f"Overall Assessment: {status}")
        
        # Recommendations
        if pos_error > 0.5:
            self.get_logger().warn("Recommendations:")
            self.get_logger().warn("- Check wheel slip parameters in Gazebo")
            self.get_logger().warn("- Verify wheel radius calibration")
            self.get_logger().warn("- Consider increasing GPS weight in EKF")
            self.get_logger().warn("- Check for joint encoder drift")
        
        self.get_logger().info("-" * 40)
    
    def start_calibration_test(self, distance=2.0):
        """Start a calibration test by driving forward"""
        self.get_logger().info(f"Starting calibration test: driving {distance}m forward")
        
        # Record starting positions
        self.start_time = time.time()
        self.start_odom = self.latest_odom.pose.pose if self.latest_odom else None
        self.start_world_pose = self.latest_world_pose
        self.test_distance = distance
        self.test_running = True
        
        # Send forward command
        cmd = Twist()
        cmd.linear.x = 1.0  # 1 m/s forward
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        # Schedule stop
        self.stop_timer = self.create_timer(distance / 1.0, self.stop_calibration_test)
    
    def stop_calibration_test(self):
        """Stop the calibration test and analyze results"""
        # Stop the vehicle
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.test_running = False
        
        if self.stop_timer:
            self.stop_timer.destroy()
        
        # Analyze results
        if self.start_odom and self.start_world_pose and self.latest_odom and self.latest_world_pose:
            odom_distance = self.pose_distance(self.start_odom, self.latest_odom.pose.pose)
            world_distance = self.pose_distance(self.start_world_pose, self.latest_world_pose)
            
            if odom_distance and world_distance:
                error = abs(odom_distance - world_distance)
                error_percent = (error / world_distance) * 100 if world_distance > 0 else 0
                
                self.get_logger().info("=== Calibration Test Results ===")
                self.get_logger().info(f"Expected distance: {self.test_distance:.2f} m")
                self.get_logger().info(f"Odometry distance: {odom_distance:.2f} m")
                self.get_logger().info(f"Ground truth distance: {world_distance:.2f} m")
                self.get_logger().info(f"Error: {error:.3f} m ({error_percent:.1f}%)")
                
                if error_percent < 2.0:
                    self.get_logger().info("✓ Wheel odometry calibration: EXCELLENT")
                elif error_percent < 5.0:
                    self.get_logger().info("✓ Wheel odometry calibration: GOOD")
                elif error_percent < 10.0:
                    self.get_logger().info("⚠ Wheel odometry calibration: ACCEPTABLE")
                else:
                    self.get_logger().warn("✗ Wheel odometry calibration: POOR")
                    
                    # Calculate correction factor
                    correction_factor = world_distance / odom_distance if odom_distance > 0 else 1.0
                    corrected_radius = self.wheel_radius * correction_factor
                    self.get_logger().warn(f"Suggested wheel radius correction: {corrected_radius:.4f} m")

def main():
    rclpy.init()
    node = WheelOdometryDiagnostic()
    
    # Run diagnostics for a while, then optionally run calibration test
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()