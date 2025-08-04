#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleAttitude, VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool

def nav_state_to_string(nav_state):
    return {
        0: "Manual",
        1: "Altitude",
        2: "Position",
        3: "Mission",
        4: "Hold",
        5: "RTL",
        6: "Position Slow",
        10: "Acro",
        12: "Descend",
        13: "Terminate",
        14: "Offboard",
        15: "Stabilized",
        17: "Takeoff",
        18: "Land",
        19: "Follow",
        20: "Precision Land",
        21: "Orbit",
        22: "VTOL Takeoff",
    }.get(nav_state, f"Unknown ({nav_state})")

def arming_state_to_string(arming_state):
    return {
        1: "Disarmed",
        2: "Armed"
    }.get(arming_state, f"Unknown ({arming_state})")

class OffboardControl(Node):

    def __init__(self):
        super().__init__('vel_ctrl')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)
        self.offboard_velocity_sub = self.create_subscription(Twist, '/offboard_velocity_cmd', self.offboard_velocity_callback, qos_profile)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        self.arm_sub = self.create_subscription(Bool, '/arm_message', self.arm_message_callback, qos_profile)
        self.stop_offboard_sub = self.create_subscription(Bool, '/stop_offboard', self.stop_offboard_callback, qos_profile)

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.arm_timer_ = self.create_timer(0.1, self.state_machine)
        self.timer = self.create_timer(0.02, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.yaw = 0.0
        self.trueYaw = 0.0
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        self.current_state = "IDLE"
        self.last_state = self.current_state
        self.allow_offboard_switch = True 
        self.landing_initiated = False

    def stop_offboard_callback(self, msg):
        if msg.data:
            self.allow_offboard_switch = False
            self.offboardMode = False
            self.landing_initiated = True
            self.get_logger().info("Offboard mode deactivation requested by GUI.")
            self.arm_message = False

    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Sent arm by button press. Arm Message: {self.arm_message}")
        if msg.data:
            self.allow_offboard_switch = True
            self.landing_initiated = False

    def state_machine(self):
        match self.current_state:
            case "IDLE":
                self.offboardMode = False
                if self.arm_message == False and self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 3.)
                if self.flightCheck and self.arm_message:
                    self.current_state = "ARMING"
                    self.get_logger().info("Switch from idle to arming")

            case "ARMING":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Arming, Flight Check Failed")
                elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 3:
                    self.current_state = "TAKEOFF"
                    self.get_logger().info("Switch from arming into takeoff")
                self.arm()

            case "TAKEOFF":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Takeoff, Flight Check Failed")
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                    self.current_state = "LOITER"
                    self.get_logger().info("Taking off, switch to hold mode after")
                self.arm()
                self.take_off()

            case "LOITER":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Loiter, Flight Check Failed")
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.current_state = "OFFBOARD"
                    self.get_logger().info("Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if not self.flightCheck or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                    self.current_state = "IDLE"
                    self.get_logger().info("Offboard, Flight Check Failed")
                elif self.landing_initiated or self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                    self.get_logger().info("Landing detected, switching to LANDING state")
                    self.offboardMode = False
                    self.allow_offboard_switch = False
                    self.current_state = "LANDING"
                elif self.allow_offboard_switch:
                    self.state_offboard()

            case "LANDING":
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                if self.arm_state == VehicleStatus.ARMING_STATE_DISARMED:
                    self.get_logger().info("Landing complete, returning to IDLE")
                    self.landing_initiated = False
                    self.current_state = "IDLE"

        if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm_message = False

        if self.last_state != self.current_state:
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    def state_offboard(self):
        if self.allow_offboard_switch and not self.landing_initiated:
            self.myCnt = 0
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.offboardMode = True

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)
        self.get_logger().info("Takeoff command sent")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def vehicle_status_callback(self, msg):
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"Mode switched, current flight mode: {nav_state_to_string(msg.nav_state)}")
        if msg.arming_state != self.arm_state:
            self.get_logger().info(f"Arm changed, ARM STATUS: {arming_state_to_string(msg.arming_state)}")
        if msg.failsafe != self.failsafe:
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        if msg.pre_flight_checks_pass != self.flightCheck:
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    def offboard_velocity_callback(self, msg):
        self.velocity.x = -msg.linear.x
        self.velocity.y = -msg.linear.y
        self.velocity.z = -msg.linear.z
        self.yaw = msg.angular.z

    def attitude_callback(self, msg):
        q = msg.q
        self.trueYaw = -np.arctan2(2.0 * (q[3] * q[0] + q[1] * q[2]), 1.0 - 2.0 * (q[0]**2 + q[1]**2))

    def cmdloop_callback(self):
        if (self.offboardMode and self.current_state == "OFFBOARD" and not self.landing_initiated and self.allow_offboard_switch):
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)

            cos_yaw = np.cos(self.trueYaw)
            sin_yaw = np.sin(self.trueYaw)
            velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
            velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.velocity[0] = velocity_world_x
            trajectory_msg.velocity[1] = velocity_world_y
            trajectory_msg.velocity[2] = self.velocity.z
            trajectory_msg.position[0] = float('nan')
            trajectory_msg.position[1] = float('nan')
            trajectory_msg.position[2] = float('nan')
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = self.yaw

            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    print("Hi hihihihh")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()