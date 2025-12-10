#!/usr/bin/env python3
"""
Drone'u belirtilen yüksekliğe kaldırır ve hover'da tutar.
Nav2 navigasyonu başlamadan önce çalıştırılmalı.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)
import sys


class TakeoffNode(Node):
    def __init__(self, target_altitude: float = 2.0):
        super().__init__('takeoff_node')
        
        self.target_altitude = target_altitude
        
        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.position_callback, qos)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.status_callback, qos)
        
        # State
        self.current_position = None
        self.vehicle_status = None
        self.is_armed = False
        self.is_offboard = False
        self.takeoff_complete = False
        self.setpoint_counter = 0
        
        # Timer - 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f'Takeoff node started. Target altitude: {target_altitude}m')
    
    def position_callback(self, msg):
        self.current_position = {
            'x': msg.x,
            'y': msg.y,
            'z': msg.z  # NED: negative is up
        }
    
    def status_callback(self, msg):
        self.vehicle_status = msg
        self.is_armed = (msg.arming_state == 2)
        self.is_offboard = (msg.nav_state == 14)
    
    def control_loop(self):
        # Publish offboard control mode
        self.publish_offboard_control_mode()
        
        if self.takeoff_complete:
            self.get_logger().info('Takeoff complete! Hovering...', throttle_duration_sec=5.0)
            self.publish_hover_setpoint()
            return
        
        # Wait for position data
        if self.current_position is None:
            self.get_logger().info('Waiting for position data...', throttle_duration_sec=2.0)
            return
        
        # Send setpoints before arming
        if self.setpoint_counter < 15:
            self.publish_takeoff_setpoint()
            self.setpoint_counter += 1
            
            if self.setpoint_counter == 10:
                self.engage_offboard_mode()
                self.arm()
            return
        
        # Takeoff
        self.publish_takeoff_setpoint()
        
        # Check if reached target altitude (NED: z is negative when up)
        current_alt = -self.current_position['z']
        if current_alt >= self.target_altitude * 0.95:
            self.takeoff_complete = True
            self.get_logger().info(f'Reached target altitude: {current_alt:.2f}m')
    
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_takeoff_setpoint(self):
        msg = TrajectorySetpoint()
        # NED coordinates: x=North, y=East, z=Down
        msg.position = [
            self.current_position['x'] if self.current_position else 0.0,
            self.current_position['y'] if self.current_position else 0.0,
            -self.target_altitude  # Negative because NED
        ]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)
    
    def publish_hover_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [
            self.current_position['x'],
            self.current_position['y'],
            -self.target_altitude
        ]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)
    
    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('Arm command sent')
    
    def engage_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('Offboard mode command sent')


def main(args=None):
    rclpy.init(args=args)
    
    # Default altitude or from command line
    altitude = 2.0
    if len(sys.argv) > 1:
        try:
            altitude = float(sys.argv[1])
        except ValueError:
            pass
    
    node = TakeoffNode(target_altitude=altitude)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()