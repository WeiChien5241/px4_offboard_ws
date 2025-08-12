#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QSlider, QLabel, QVBoxLayout, QWidget, QMessageBox, QGridLayout, QLineEdit, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QPen, QBrush, QFont
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleCommand, VehicleStatus, VehicleLandDetected
from std_msgs.msg import Bool
import math

def nav_state_to_string(nav_state):
    """Convert navigation state number to string representation"""
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
    """Convert arming state number to string representation"""
    return {
        1: "Disarmed",
        2: "Armed"
    }.get(arming_state, f"Unknown ({arming_state})")

class ClickableCanvas(QWidget):
    """Canvas for displaying and interacting with the drone's field of view"""
    # Parameters for the actual resolution, display resolution, and field of view
    ACTUAL_WIDTH = 1920
    ACTUAL_HEIGHT = 1080
    DISPLAY_WIDTH = 640
    DISPLAY_HEIGHT = 360
    HFOV_DEG = 60
    VFOV_DEG = 45

    def __init__(self, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT):
        super().__init__()
        self.display_height = height
        self.display_width = width
        self.setFixedSize(width, height)

        self.disp_x = None
        self.disp_y = None
        self.actual_x = None
        self.actual_y = None
        self.center_x = self.display_width // 2
        self.center_y = self.display_height // 2
    
    # Gets coordinate from left mouse click
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.disp_x = event.x()
            self.disp_y = event.y()
            self.actual_x = int(self.disp_x * self.ACTUAL_WIDTH / self.display_width)
            self.actual_y = int(self.disp_y * self.ACTUAL_HEIGHT / self.display_height)
            self.update()

            window = self.window()
            if hasattr(window, 'on_canvas_click'):
                window.on_canvas_click()
    
    # Calculates angles from pixel coordinates based on the actual resolution and field of view
    def get_angles_from_pixel(self, x, y):
        offset_x = x - self.ACTUAL_WIDTH // 2
        offset_y = y - self.ACTUAL_HEIGHT // 2
        angle_xy = (offset_x / (self.ACTUAL_WIDTH / 2)) * (self.HFOV_DEG / 2)
        angle_z = -(offset_y / (self.ACTUAL_HEIGHT / 2)) * (self.VFOV_DEG / 2)
        return angle_xy, angle_z

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.setPen(QPen(Qt.lightGray, 1))
        grid_size = 20
        for x in range(0, self.display_width, grid_size):
            painter.drawLine(x, 0, x, self.display_height)
        for y in range(0, self.display_height, grid_size):
            painter.drawLine(0, y, self.display_width, y)

        painter.setPen(QPen(Qt.red, 2))
        painter.drawLine(self.center_x, 0, self.center_x, self.display_height)
        painter.drawLine(0, self.center_y, self.display_width, self.center_y)

        painter.setBrush(QBrush(Qt.red))
        painter.drawEllipse(self.center_x - 3, self.center_y - 3, 6, 6)

        painter.setFont(QFont("Arial", 10))
        painter.setPen(QPen(Qt.black, 1))
        
        # Horizontal angle labels
        painter.drawText(10, self.center_y - 10, f"-{self.HFOV_DEG/2}°")
        painter.drawText(self.display_width - 40, self.center_y - 10, f"+{self.HFOV_DEG/2}°")
        
        # Vertical angle labels
        painter.drawText(self.center_x + 10, 20, f"+{self.VFOV_DEG/2}°")
        painter.drawText(self.center_x + 10, self.display_height - 10, f"-{self.VFOV_DEG/2}°")

        # Draw center label
        painter.drawText(self.center_x + 10, self.center_y + 20, "Center (0°, 0°)")

        # Draw the target point if it exists
        if self.disp_x is not None and self.disp_y is not None:
            painter.setBrush(QBrush(Qt.blue))
            painter.setPen(QPen(Qt.blue, 2))
            painter.drawEllipse(self.disp_x - 5, self.disp_y - 5, 10, 10)
            angle_xy, angle_z = self.get_angles_from_pixel(self.actual_x, self.actual_y)
            painter.drawText(self.disp_x + 10, self.disp_y - 10, f"({self.actual_x}, {self.actual_y})")
            painter.drawText(self.disp_x + 10, self.disp_y + 10, f"XY: {angle_xy:.1f}°, Z: {angle_z:.1f}°")

def velocity_from_angles(vel, angle_xy, angle_z):
    """Calculates the velocity components from given angle and velocity"""
    v_pitch = vel * math.cos(math.radians(angle_z)) * math.cos(math.radians(angle_xy))
    v_roll = vel * math.cos(math.radians(angle_z)) * math.sin(math.radians(angle_xy))
    v_throttle = vel * math.sin(math.radians(angle_z))
    return v_pitch, v_roll, v_throttle

class DroneGUIControl(Node):
    def __init__(self):
        super().__init__('gui_control')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.vel_pub = self.create_publisher(Twist, 'offboard_velocity_cmd', qos)
        self.arm_request_pub = self.create_publisher(Bool, '/arm_request', qos)
        self.rtl_request_pub = self.create_publisher(Bool, '/rtl_request', qos)
        self.land_request_pub = self.create_publisher(Bool, '/land_request', qos)
        self.stop_offboard_pub = self.create_publisher(Bool, '/stop_offboard', qos)
        self.position_request_pub = self.create_publisher(Bool, '/position_request', qos)
        self.offboard_request_pub = self.create_publisher(Bool, '/offboard_request', qos)
        
        # Subscribers
        #self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos) # real drone
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_callback, qos) # sitl
        self.land_detected_sub = self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.land_detected_callback, qos)

        # Timers
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Variables
        self.current_status = VehicleStatus()
        self.twist = Twist()
        self.is_landed = True 
        self.window = None
        self.is_in_offboard = False
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.param7 = param7
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    # --- Callbacks ---
    def timer_callback(self):
        """Publishes velocity command constantly"""
        self.vel_pub.publish(self.twist)

    def status_callback(self, msg):
        self.current_status = msg
        self.update_gui_state()

    def land_detected_callback(self, msg):
        self.is_landed = msg.landed
        self.update_gui_state()

    def update_gui_state(self):
        """Update GUI button states based on current vehicle status"""
        if not self.window:
            return

        is_armed = self.current_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        is_landing = self.current_status.nav_state in [ VehicleStatus.NAVIGATION_STATE_AUTO_LAND, VehicleStatus.NAVIGATION_STATE_AUTO_RTL] 
        self.is_in_offboard = self.current_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if is_armed and not self.is_landed:
            # Armed but not landed
            self.window.arm_button.setText("Armed")
            self.window.arm_button.setEnabled(False)
            if is_landing:
                self.window.land_button.setText("Landing...")
                self.window.land_button.setEnabled(False)
                self.window.rtl_button.setEnabled(False)
                self.window.offboard_button.setEnabled(False)
            else:
                self.window.land_button.setText("Land and Disarm")
                self.window.land_button.setEnabled(True)
                self.window.rtl_button.setEnabled(True)
                self.window.offboard_button.setEnabled(True)
                if self.is_in_offboard:
                    self.window.offboard_button.setText("Switch to Position Mode")
                else:
                    self.window.offboard_button.setText("Switch to Offboard Mode")
        elif not is_armed and self.is_landed:
            # Drone is disarmed and landed - ready for next flight
            self.window.arm_button.setText("Arm and Takeoff")
            self.window.arm_button.setEnabled(True)
            self.window.land_button.setText("Land and Disarm")
            self.window.land_button.setEnabled(False)
            self.window.rtl_button.setEnabled(False)
            self.window.offboard_button.setEnabled(False)
        else:
            # Default state
            self.window.arm_button.setText("Arm and Takeoff")
            self.window.arm_button.setEnabled(not is_armed)
            self.window.land_button.setEnabled(False)
            self.window.rtl_button.setEnabled(False)
            self.window.offboard_button.setEnabled(False)

    # --- Button logics ---
    def arm_drone(self):
        self.arm_request_pub.publish(Bool(data=True))
        self.get_logger().info("Arm command sent, switch to arming")
    
    def landing_sequence(self, command):
        self.get_logger().info(f"{command} command sent, switching to {command} mode")
        self.arm_request_pub.publish(Bool(data=False))
        if self.is_in_offboard:
            self.offboard_request_pub.publish(Bool(data=False))
            self.stop_offboard_pub.publish(Bool(data=True))

    def land_drone(self):
        self.land_request_pub.publish(Bool(data=True))
        self.landing_sequence("Land")

    def rtl_drone(self):
        self.rtl_request_pub.publish(Bool(data=True))
        self.landing_sequence("Return to Launch (RTL)")
    
    def offboard_switch(self):
        if self.is_in_offboard:
            self.stop_offboard_pub.publish(Bool(data=True))
            self.position_request_pub.publish(Bool(data=True))
            self.offboard_request_pub.publish(Bool(data=False))
        else:
            self.stop_offboard_pub.publish(Bool(data=False))
            self.position_request_pub.publish(Bool(data=False))
            self.offboard_request_pub.publish(Bool(data=True))

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("PX4 Drone Control GUI")
        self.setGeometry(500, 200, 500, 500)
        
        layout = QVBoxLayout()
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        # --- Mode Buttons ---
        mode_button_layout = QGridLayout()
        
        self.arm_button = QPushButton("Arm and Takeoff")
        self.arm_button.clicked.connect(self.arm_logic)
        mode_button_layout.addWidget(self.arm_button, 0, 0)

        self.land_button = QPushButton("Land and Disarm")
        self.land_button.clicked.connect(self.land_logic)
        self.land_button.setEnabled(False)
        mode_button_layout.addWidget(self.land_button, 0, 1)

        self.offboard_button = QPushButton("Offboard Mode")
        self.offboard_button.clicked.connect(self.offboard_logic)
        self.offboard_button.setEnabled(False)
        mode_button_layout.addWidget(self.offboard_button, 1, 0)

        self.rtl_button = QPushButton("Return to Launch (RTL)")
        self.rtl_button.clicked.connect(self.return_to_launch)
        self.rtl_button.setEnabled(False)
        mode_button_layout.addWidget(self.rtl_button, 1, 1)

        layout.addLayout(mode_button_layout)

        # --- Canvas ---
        canvas_layout = QVBoxLayout()
        canvas_layout.addWidget(QLabel("Clickable canvas - Click to set target coordinate:"))

        self.canvas = ClickableCanvas(ClickableCanvas.DISPLAY_WIDTH, ClickableCanvas.DISPLAY_HEIGHT)
        canvas_layout.addWidget(self.canvas, alignment=Qt.AlignCenter)

        canvas_controls_layout = QHBoxLayout()

        self.canvas_velocity = QLineEdit(self)
        self.canvas_velocity.setPlaceholderText("Velocity (m/s)")
        canvas_controls_layout.addWidget(self.canvas_velocity)

        self.canvas_duration = QLineEdit(self)
        self.canvas_duration.setPlaceholderText("Duration (s)")
        canvas_controls_layout.addWidget(self.canvas_duration)

        self.canvas_execute_button = QPushButton("Execute Canvas Target")
        self.canvas_execute_button.clicked.connect(self.execute_canvas_target)
        self.canvas_execute_button.setEnabled(False)
        canvas_controls_layout.addWidget(self.canvas_execute_button)
        
        self.canvas_clear_button = QPushButton("Clear Target")
        self.canvas_clear_button.clicked.connect(self.clear_canvas_target)
        canvas_controls_layout.addWidget(self.canvas_clear_button)

        canvas_layout.addLayout(canvas_controls_layout)

        self.velocity_display = QLabel("Velocity: x=0.00, y=0.00, z=0.00")
        self.velocity_display.setAlignment(Qt.AlignCenter)
        self.velocity_display.setStyleSheet("""
            border: 2px solid black;
            border-radius: 6px;
            background: #f0f8ff;
            padding: 3px;
            font-weight: bold;
            font-size: 12px;
        """)
        canvas_layout.addWidget(self.velocity_display)

        layout.addLayout(canvas_layout)

        # -- RC Layout ---
        rc_layout = QGridLayout()

        throttle_layout = QVBoxLayout()
        self.throttle_slider = QSlider(Qt.Vertical)
        self.throttle_slider.setRange(-100, 100)
        self.throttle_slider.setFixedHeight(200)
        self.throttle_slider.valueChanged.connect(self.update_velocity)
        self.throttle_slider.sliderReleased.connect(self.reset_sliders)
        throttle_layout.addWidget(QLabel("Z Velocity (m/s)"), alignment=Qt.AlignCenter)
        throttle_layout.addWidget(self.throttle_slider, alignment=Qt.AlignCenter)
        rc_layout.addLayout(throttle_layout, 0, 0)

        yaw_layout = QVBoxLayout()
        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setRange(-100, 100)
        self.yaw_slider.setFixedWidth(200)
        self.yaw_slider.valueChanged.connect(self.update_velocity)
        self.yaw_slider.sliderReleased.connect(self.reset_sliders)
        yaw_layout.addWidget(QLabel("Yaw Rate (rad/s)"), alignment=Qt.AlignCenter)
        yaw_layout.addWidget(self.yaw_slider, alignment=Qt.AlignCenter)
        rc_layout.addLayout(yaw_layout, 1, 0)
        
        pitch_layout = QVBoxLayout()
        self.pitch_slider = QSlider(Qt.Vertical)
        self.pitch_slider.setRange(-100, 100)
        self.pitch_slider.setFixedHeight(200)
        self.pitch_slider.valueChanged.connect(self.update_velocity)
        self.pitch_slider.sliderReleased.connect(self.reset_sliders)
        pitch_layout.addWidget(QLabel("X Velocity (m/s)"), alignment=Qt.AlignCenter)
        pitch_layout.addWidget(self.pitch_slider, alignment=Qt.AlignCenter)
        rc_layout.addLayout(pitch_layout, 0, 1)

        roll_layout = QVBoxLayout()
        self.roll_slider = QSlider(Qt.Horizontal)
        self.roll_slider.setRange(-100, 100)
        self.roll_slider.setFixedWidth(200)
        self.roll_slider.valueChanged.connect(self.update_velocity)
        self.roll_slider.sliderReleased.connect(self.reset_sliders)
        roll_layout.addWidget(QLabel("Y Velocity (m/s)"), alignment=Qt.AlignCenter)
        roll_layout.addWidget(self.roll_slider, alignment=Qt.AlignCenter)
        rc_layout.addLayout(roll_layout, 1, 1)

        layout.addLayout(rc_layout)
        
    def on_canvas_click(self):
        self.canvas_execute_button.setEnabled(True)

    def clear_canvas_target(self):
        self.canvas.click_x = None
        self.canvas.click_y = None
        self.canvas.update()
        self.canvas_execute_button.setEnabled(False)

    def execute_canvas_target(self):
        try:
            canvas_vel = float(self.canvas_velocity.text())
            canvas_duration_time = float(self.canvas_duration.text())
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid velocity and duration values!")
            return

        target_x = self.canvas.actual_x
        target_y = self.canvas.actual_y
        
        angle_xy, angle_z = self.canvas.get_angles_from_pixel(target_x, target_y)
        v_pitch, v_roll, v_throttle = velocity_from_angles(canvas_vel, angle_xy, angle_z)

        self.node.twist.linear.x = v_pitch
        self.node.twist.linear.y = v_roll
        self.node.twist.linear.z = v_throttle
        self.node.twist.angular.z = 0.0

        self.node.get_logger().info(f"Canvas target velocity set: x={v_pitch:.2f}, y={v_roll:.2f}, z={v_throttle:.2f}")
        self.velocity_display.setText(f"Velocity: x={v_pitch:.2f}, y={v_roll:.2f}, z={v_throttle:.2f}")

        QTimer.singleShot(int(canvas_duration_time * 1000), self.stop_velocity)

    # def coords_submit(self):
        try:
            target_x_coord = float(self.x_coord.text())
            target_y_coord = float(self.y_coord.text())
            coord_vel = float(self.coords_velocity.text())
            coord_duration_time = float(self.coords_duration.text())
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid coordinate, velocity, and duration values!")
            return
        
        self.x_coord.clear()
        self.y_coord.clear()
        self.coords_velocity.clear()
        self.coords_duration.clear()

        angle_xy, angle_z = self.canvas.get_angles_from_pixel(target_x_coord, target_y_coord)
        v_pitch, v_roll, v_throttle = velocity_from_angles(coord_vel, angle_xy, angle_z)

        self.node.twist.linear.x = v_pitch
        self.node.twist.linear.y = v_roll
        self.node.twist.linear.z = v_throttle
        self.node.twist.angular.z = 0.0

        self.node.get_logger().info(f"Velocity set: x={v_pitch:.2f}, y={v_roll:.2f}, z={v_throttle:.2f}")

        QTimer.singleShot(int(coord_duration_time * 1000), self.stop_velocity)

    # def submit(self):
        try:
            xy_deg = float(self.xy_angle.text())
            z_deg = float(self.z_angle.text())
            vel = float(self.velocity.text())
            duration_time = float(self.duration.text())
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid angle, velocity, and duration values!")
            return

        self.xy_angle.clear()
        self.z_angle.clear()
        self.velocity.clear()
        self.duration.clear()
    
        v_pitch, v_roll, v_throttle = velocity_from_angles(vel, xy_deg, z_deg)

        self.node.twist.linear.x = v_pitch
        self.node.twist.linear.y = v_roll
        self.node.twist.linear.z = v_throttle
        self.node.twist.angular.z = 0.0

        self.node.get_logger().info(f"Velocity set: x={v_pitch:.2f}, y={v_roll:.2f}, z={v_throttle:.2f}")

        QTimer.singleShot(int(duration_time * 1000), self.stop_velocity)
    
    def stop_velocity(self):
        self.node.twist.linear.x = 0.0
        self.node.twist.linear.y = 0.0
        self.node.twist.linear.z = 0.0
        self.node.twist.angular.z = 0.0
        self.node.get_logger().info("Velocity stopped")

    def reset_sliders(self):
        self.pitch_slider.setValue(0)
        self.throttle_slider.setValue(0)
        self.roll_slider.setValue(0)
        self.yaw_slider.setValue(0)
        self.update_velocity()
    
    def update_velocity(self):
        self.node.twist.linear.x = self.pitch_slider.value() / 50.0
        self.node.twist.linear.y = self.roll_slider.value() / 50.0
        self.node.twist.linear.z = self.throttle_slider.value() / 100.0
        self.node.twist.angular.z = self.yaw_slider.value() / 100.0

    def arm_logic(self):
        self.node.arm_drone()

    def land_logic(self):
        self.node.land_drone()

    def return_to_launch(self):
        self.node.rtl_drone()
        self.node.get_logger().info("Return to Launch (RTL) command sent")
    
    def offboard_logic(self):
        self.node.offboard_switch()

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = DroneGUIControl()
    window = MainWindow(node)
    window.show()
    node.window = window

    # Integrate ROS 2 with Qt loop
    from PyQt5.QtCore import QTimer
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(10)

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()