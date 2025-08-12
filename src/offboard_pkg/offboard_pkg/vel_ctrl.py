#!/usr/bin/env python3
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
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        # self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos) # real drone  
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos) # for sitl
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos)
        self.offboard_velocity_sub = self.create_subscription(Twist, '/offboard_velocity_cmd', self.offboard_velocity_callback, qos)
        self.arm_request_sub = self.create_subscription(Bool, '/arm_request', self.arm_request_callback, qos)
        self.rtl_request_sub = self.create_subscription(Bool, '/rtl_request', self.rtl_request_callback, qos)
        self.land_request_sub = self.create_subscription(Bool, '/land_request', self.land_request_callback, qos)
        self.stop_offboard_sub = self.create_subscription(Bool, '/stop_offboard', self.stop_offboard_callback, qos)
        self.position_request_sub = self.create_subscription(Bool, '/position_request', self.position_request_callback, qos)
        self.offboard_request_sub = self.create_subscription(Bool, '/offboard_request', self.offboard_request_callback, qos)

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Timers
        self.arm_timer_ = self.create_timer(0.1, self.state_machine)
        self.timer = self.create_timer(0.02, self.cmdloop_callback)

        # State Variables
        self.nav_state = None
        self.arm_state = None
        self.velocity = Vector3()
        self.yaw = 0.0
        self.trueYaw = 0.0
        self.current_state = "POSITION"
        self.last_state = self.current_state
        self.flightCheck = False
        self.failsafe = False
        self.arm_message = False
        self.offboardMode = False
        self.allow_offboard_switch = True 
        self.offboard_request = False
        self.position_request = False
        self.landing_initiated = False
        self.rtl_request = False
        self.fsm_paused = False
        self.rc_override = False

    # --- Callbacks ---
    def position_request_callback(self, msg):
        self.fsm_paused = False
        self.position_request = msg.data
    
    def land_request_callback(self, msg):
        self.fsm_paused = False
        if msg.data:
            self.landing_initiated = True
        else:
            self.landing_request = False
    
    def rtl_request_callback(self, msg):
        self.rtl_request = msg.data
        self.fsm_paused = False
        if msg.data:
            self.get_logger().info("RTL requested by GUI.")
    
    def offboard_request_callback(self, msg):
        self.offboard_request = msg.data
        self.fsm_paused = False
        if msg.data:
            self.get_logger().info("Offboard mode activation requested by GUI.")
            if self.fsm_paused:
                self.fsm_paused = False
    
    def stop_offboard_callback(self, msg):
        self.fsm_paused = False
        if msg.data:
            self.allow_offboard_switch = False
            self.offboardMode = False
            self.get_logger().info("Offboard mode deactivation requested by GUI.")
            self.arm_message = False

    def arm_request_callback(self, msg):
        self.arm_message = msg.data
        self.fsm_paused = False
        if msg.data:
            self.rc_override = False
            self.allow_offboard_switch = True
            self.landing_initiated = False

    def vehicle_status_callback(self, msg):
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"Mode switched, current flight mode: {nav_state_to_string(msg.nav_state)}")
            if msg.nav_state not in [
                VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
                VehicleStatus.NAVIGATION_STATE_POSCTL,
                VehicleStatus.NAVIGATION_STATE_AUTO_RTL,
                VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF,
                VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            ]:
                self.fsm_paused = True
            else:
                self.fsm_paused = False
        
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

    def _should_transition_to_landing(self):
        """Check if conditions are met to transition to landing state"""
        return (self.rtl_request or self.landing_initiated or self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND)
    
    def _transition_to_landing(self):
        """Execute transition to landing state"""
        self.get_logger().info("Switch to landing state")
        self.offboardMode = False
        self.allow_offboard_switch = False
        self.current_state = "LANDING"

    def _transistion_to_offboard(self):
        """Execute transition to offboard state"""
        self.allow_offboard_switch = True
        self.state_offboard()
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.current_state = "OFFBOARD"
            self.get_logger().info("User requested Offboard, swtiching to OFFBOARD")

    def state_machine(self):
        if not self.fsm_paused and not self.rc_override:
            match self.current_state:
                case "POSITION":
                        self.offboardMode = False
                        externally_armed = self.arm_state == VehicleStatus.ARMING_STATE_ARMED and not self.arm_message
                        if self.arm_message == False and self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
                            self.position()
                        elif self._should_transition_to_landing():
                            self._transition_to_landing()
                        elif self.offboard_request and self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
                            self._transistion_to_offboard()
                        elif self.flightCheck and (self.arm_message or externally_armed) and not self.position_request:
                            self.current_state = "ARM_AND_TAKEOFF"
                            self.get_logger().info("Arm and Takeoff requested")
                        elif self.position_request:
                            self.position()
                        
                case "ARM_AND_TAKEOFF":
                    if not self.flightCheck:
                        self.current_state = "POSITION"
                        self.get_logger().info("Arming, Flight Check Failed")
                    elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                        self.current_state = "HOLD"
                        self.get_logger().info("Takeoff complete, switching to HOLD")
                    elif self.landing_initiated:
                        self.get_logger().info("Landing initiated, switching to LANDING state")
                        self.current_state = "LANDING"
                    self.arm()
                    self.take_off()

                case "HOLD":
                    if self._should_transition_to_landing():
                            self._transition_to_landing()
                    elif self.offboard_request:
                        self._transistion_to_offboard()

                case "OFFBOARD":
                    if not self.flightCheck or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe:
                        self.offboardMode = False
                        self.allow_offboard_switch = False
                        self.current_state = "POSITION"
                        self.get_logger().info("Flight Check Failed, switching to POSITION")
                    elif self._should_transition_to_landing():
                            self._transition_to_landing()
                    elif self.position_request:
                        self.offboardMode = False
                        self.allow_offboard_switch = False
                        self.get_logger().info("Position requested, switching to Position state")
                        self.current_state = "POSITION"
                    elif self.allow_offboard_switch:
                        self.state_offboard()
                        if self.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL and not self.position_request:
                            self.get_logger().info("Position mode detected, RC override, switching out of offboard mode")
                            self.rc_override = True
                            self.offboardMode = False
                            self.allow_offboard_switch = False
                            self.position_request = True
                            self.current_state = "POSITION"

                case "LANDING":
                    if self.rtl_request:
                        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
                    else:
                        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    if self.arm_state == VehicleStatus.ARMING_STATE_DISARMED:
                        self.rtl_request = False
                        self.get_logger().info("Landing complete, returning to POSITION")
                        self.landing_initiated = False
                        self.current_state = "POSITION"

            if self.last_state != self.current_state:
                self.last_state = self.current_state
                self.get_logger().info(f"Current state: {self.current_state}")

    def state_offboard(self):
        if self.allow_offboard_switch and not self.landing_initiated:
            self.myCnt = 0
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.offboardMode = True

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def take_off(self):
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=5.0)
            self.get_logger().info("Takeoff command sent")
    
    def position(self):
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_POSCTL:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 3.)

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

    def offboard_velocity_callback(self, msg):
        self.velocity.x = -msg.linear.x
        self.velocity.y = -msg.linear.y
        self.velocity.z = -msg.linear.z
        self.yaw = msg.angular.z

    def attitude_callback(self, msg):
        q = msg.q
        self.trueYaw = -np.arctan2(2.0 * (q[3] * q[0] + q[1] * q[2]), 1.0 - 2.0 * (q[0]**2 + q[1]**2))

    def cmdloop_callback(self):
        if (self.offboardMode and self.allow_offboard_switch and not self.landing_initiated):
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
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()