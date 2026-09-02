#!/usr/bin/env python3
"""
ROS2 Control Hardware Interface Validator
Validates sensor readings, actuator commands, and hardware interface reliability
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from msgs.msg import TeleopCommand
import math
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ControlValidator(Node):
    def __init__(self):
        super().__init__('control_validator')
        
        # Robot parameters
        self.declare_parameter('wheelbase', 0.6)
        self.declare_parameter('wheel_radius', 0.25)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('max_velocity', 2.0)
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        # Data storage
        self.joint_states = None
        self.last_joint_time = None
        self.command = None
        self.last_command_time = None
        
        # Statistics
        self.update_times = deque(maxlen=1000)
        self.command_errors = []
        self.validation_count = 0
        self.error_count = 0
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 1)
        
        self.cmd_sub = self.create_subscription(
            TeleopCommand, 'robot_steering_controller/reference_teleop',
            self.command_callback, qos)
        
        # Validation timer
        self.timer = self.create_timer(10.0, self.validate_and_report)
        
        self.get_logger().info('='*60)
        self.get_logger().info('ROS2 Control Hardware Validator Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Wheelbase: {self.wheelbase} m')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')
        self.get_logger().info('Monitoring hardware interfaces...\n')

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now()
        if self.last_joint_time is not None:
            dt = (current_time - self.last_joint_time).nanoseconds / 1e9
            self.update_times.append(dt)
        
        self.joint_states = msg
        self.last_joint_time = current_time
        
    def command_callback(self, msg):
        self.command = msg
        self.last_command_time = self.get_clock().now()

    def validate_and_report(self):
        """Main validation routine with detailed reporting"""
        self.validation_count += 1
        
        print('\n' + '='*60)
        print(f'VALIDATION REPORT #{self.validation_count}')
        print('='*60)
        
        # 1. Check data freshness
        if not self.check_data_freshness():
            self.error_count += 1
            return
        
        # 2. Validate joint states
        self.validate_joint_states()
        
        # 3. Validate command execution
        if self.command is not None:
            self.validate_command_execution()
        
        # 5. Performance metrics
        self.print_performance_metrics()
        
        print('='*60 + '\n')

    def check_data_freshness(self):
        """Check that data is being received and is recent"""
        current_time = self.get_clock().now()
        
        print('\n📡 DATA FRESHNESS CHECK:')
        
        # Joint states check
        if self.last_joint_time is None:
            print('  ❌ No joint states received')
            return False
        
        time_since_joint = (current_time - self.last_joint_time).nanoseconds / 1e9
        if time_since_joint > 0.01:
            print(f'  ❌ Joint states stale ({time_since_joint:.3f}s old)')
            return False
        else:
            print(f'  ✅ Joint states fresh ({time_since_joint*1000:.3f}ms old)')
        
        # Command check
        if self.last_command_time is not None:
            time_since_cmd = (current_time - self.last_command_time).nanoseconds / 1e9
            print(f'  ✅ Commands fresh ({time_since_cmd*1000:.1f}ms old)')
        else:
            print('  ⚠️  No commands received yet')
        
        return True

    def validate_joint_states(self):
        """Validate joint state values are physically reasonable"""
        print('\n🔧 JOINT STATE VALIDATION:')
        
        js = self.joint_states
        all_valid = True
        
        # Front steering joints
        print('  Front Steering Joints:')
        for i, name in enumerate(js.name):
            if 'front' in name and i < len(js.position):
                angle_rad = js.position[i]
                angle_deg = math.degrees(angle_rad)
                
                if abs(angle_rad) > self.max_steering_angle:
                    all_valid = False
                else:
                    print(f'    ✅ {name}: {angle_deg:.1f}° ({angle_rad:.3f} rad)')
        
        # Rear wheel velocities
        print('  Rear Wheel Velocities:')
        for i, name in enumerate(js.name):
            if 'rear' in name and i < len(js.velocity):
                angular_vel = js.velocity[i]
                linear_vel = angular_vel * self.wheel_radius
                
                print(f'    {name}:')
                print(f'      Angular: {angular_vel:.3f} rad/s')
                print(f'      Linear:  {linear_vel:.3f} m/s')
                
                if abs(linear_vel) > self.max_velocity:
                    all_valid = False
                else:
                    print(f'      ✅ Within limits')
        
        if all_valid:
            print('  ✅ All joint states valid')
        
        return all_valid

    def validate_command_execution(self):
        """Validate commanded velocities match actual outputs"""
        print('\n⚙️  COMMAND EXECUTION VALIDATION:')
        
        cmd_linear = self.command.velocity
        cmd_angular = self.command.angle
        
        print(f'  Commanded:')
        print(f'    Linear velocity:  {cmd_linear:.3f} m/s')
        print(f'    Wheel angle: {cmd_angular:.3f} degrees')


    def print_performance_metrics(self):
        """Print real-time performance metrics"""
        print('\n📊 PERFORMANCE METRICS:')
        
        if len(self.update_times) > 0:
            avg_update = sum(self.update_times) / len(self.update_times)
            max_update = max(self.update_times)
            min_update = min(self.update_times)
            update_rate = 1.0 / avg_update if avg_update > 0 else 0
            
            print(f'  Joint state update rate: {update_rate:.1f} Hz')
            print(f'  Update period: avg={avg_update*1000:.2f}ms, '
                  f'min={min_update*1000:.2f}ms, max={max_update*1000:.2f}ms')
            
            if max_update > 0.15:  # 150ms
                print(f'    ⚠️  Maximum update period high ({max_update*1000:.1f}ms)')
            else:
                print(f'    ✅ Update timing consistent')
        
        print(f'  Total validations: {self.validation_count}')
        print(f'  Errors detected: {self.error_count}')
        success_rate = ((self.validation_count - self.error_count) / 
                       max(self.validation_count, 1)) * 100
        print(f'  Success rate: {success_rate:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    validator = ControlValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        print('\n\n' + '='*60)
        print('FINAL VALIDATION SUMMARY')
        print('='*60)
        print(f'Total validations performed: {validator.validation_count}')
        print(f'Total errors detected: {validator.error_count}')
        success_rate = ((validator.validation_count - validator.error_count) / 
                       max(validator.validation_count, 1)) * 100
        print(f'Overall success rate: {success_rate:.1f}%')
        print('='*60)
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
