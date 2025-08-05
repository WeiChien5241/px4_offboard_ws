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
import time

# added dictionary to convert state number to actual state
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

# same logic for arm number to arm state
def arming_state_to_string(arming_state):
    return {
        1: "Disarmed",
        2: "Armed"
    }.get(arming_state, f"Unknown ({arming_state})")

class ClickableCanvas(QWidget):
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

        if self.disp_x is not None and self.disp_y is not None:
            painter.setBrush(QBrush(Qt.blue))
            painter.setPen(QPen(Qt.blue, 2))
            painter.drawEllipse(self.disp_x - 5, self.disp_y - 5, 10, 10)
            angle_xy, angle_z = self.get_angles_from_pixel(self.actual_x, self.actual_y)
            painter.drawText(self.disp_x + 10, self.disp_y - 10, f"({self.actual_x}, {self.actual_y})")
            painter.drawText(self.disp_x + 10, self.disp_y + 10, f"XY: {angle_xy:.1f}°, Z: {angle_z:.1f}°")

def velocity_from_angles(vel, angle_xy, angle_z):
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

        self.vel_pub = self.create_publisher(Twist, 'offboard_velocity_cmd', qos)
        self.arm_pub = self.create_publisher(Bool, '/arm_message', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.hold_request_pub = self.create_publisher(Bool, '/hold_request', qos)
        self.rtl_request_pub = self.create_publisher(Bool, '/rtl_request', qos)
        # added publisher to stop offboard when switching to land
        self.stop_offboard_pub = self.create_publisher(Bool, '/stop_offboard', qos)

        # if using real drone, use vehicle status without v1
        #self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos)
        # if using SITL, use vehicle status with v1
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_callback, qos)
        # Add subscription for land detection
        self.land_detected_sub = self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.land_detected_callback, qos)

        self.current_status = VehicleStatus()
        self.is_landed = True 
        self.window = None

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.twist = Twist()
    
    def timer_callback(self):
        self.vel_pub.publish(self.twist)
    
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

    def status_callback(self, msg):
        self.current_status = msg
        self.update_gui_state()

    def land_detected_callback(self, msg):
        self.is_landed = msg.landed
        self.update_gui_state()

    # handles the functionality of enabling the button or disabling the button
    def update_gui_state(self):
        """Update GUI button states based on current vehicle status"""
        if not self.window:
            return

        is_armed = self.current_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        is_disarmed = self.current_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED

        if is_armed and not self.is_landed:
            # Drone is armed and airborne - enable land button, disable arm button
            self.window.arm_button.setText("Armed")
            self.window.arm_button.setEnabled(False)
            self.window.hold_button.setEnabled(True)
            if self.current_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND or self.current_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
                self.window.land_button.setText("Landing...")
                self.window.land_button.setEnabled(False)
                self.window.rtl_button.setEnabled(False)
            else:
                self.window.land_button.setText("Land and Disarm")
                self.window.land_button.setEnabled(True)
                self.window.rtl_button.setEnabled(True)
        elif is_disarmed and self.is_landed:
            # Drone is disarmed and landed - ready for next flight
            self.window.arm_button.setText("Arm and Takeoff")
            self.window.arm_button.setEnabled(True)
            self.window.land_button.setText("Land and Disarm")
            self.window.land_button.setEnabled(False)
            self.window.rtl_button.setEnabled(False)
            self.window.hold_button.setEnabled(False)
        else:
            # Default state
            self.window.arm_button.setText("Arm and Takeoff")
            self.window.arm_button.setEnabled(is_disarmed)
            self.window.land_button.setEnabled(False)
            self.window.rtl_button.setEnabled(False)
            self.window.hold_button.setEnabled(False)

    def arm_drone(self, arm):
        if arm:
            if self.current_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Drone is already armed")
                return
            self.arm_pub.publish(Bool(data=True))
            self.get_logger().info("Sending arm message")
    
    def land_drone(self):
        self.stop_offboard_pub.publish(Bool(data=True))
        self.get_logger().info("Land command sent, switching to Land mode")
        self.arm_pub.publish(Bool(data=False))

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("PX4 Drone Control GUI")
        self.setGeometry(500, 200, 500, 500)
        
        # Layout
        layout = QVBoxLayout()
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        mode_button_layout = QGridLayout()
        
        self.arm_button = QPushButton("Arm and Takeoff")
        self.arm_button.clicked.connect(self.arm_logic)
        mode_button_layout.addWidget(self.arm_button, 0, 0)

        self.land_button = QPushButton("Land and Disarm")
        self.land_button.clicked.connect(self.land_logic)
        self.land_button.setEnabled(False)
        mode_button_layout.addWidget(self.land_button, 0, 1)

        self.hold_button = QPushButton("Hold Mode")
        self.hold_button.clicked.connect(self.hold_logic)
        self.hold_button.setEnabled(False)
        mode_button_layout.addWidget(self.hold_button, 1, 0)

        self.rtl_button = QPushButton("Return to Launch (RTL)")
        self.rtl_button.clicked.connect(self.return_to_launch)
        self.rtl_button.setEnabled(False)
        mode_button_layout.addWidget(self.rtl_button, 1, 1)

        layout.addLayout(mode_button_layout)

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

        # textbox_layout = QGridLayout()

        # self.xy_angle = QLineEdit(self)
        # self.xy_angle.setPlaceholderText("Enter angle (X)")
        # textbox_layout.addWidget(self.xy_angle, 0, 0)

        # self.z_angle = QLineEdit(self)
        # self.z_angle.setPlaceholderText("Enter angle (Z)")
        # textbox_layout.addWidget(self.z_angle, 0, 1)

        # self.velocity = QLineEdit(self)
        # self.velocity.setPlaceholderText("Enter velocity")
        # textbox_layout.addWidget(self.velocity, 0, 2)

        # self.duration = QLineEdit(self)
        # self.duration.setPlaceholderText("Enter duration")
        # textbox_layout.addWidget(self.duration, 0, 3)

        # self.xy_angle.setToolTip("Angle in XY plane (0°=forward, 90°=right, 180°=back)")
        # self.z_angle.setToolTip("Angle above XY plane (0°=flat, 90°=straight up)")
        # self.velocity.setToolTip("Speed magnitude in m/s")
        # self.duration.setToolTip("Duration in s")
        
        # layout.addLayout(textbox_layout)

        # self.submit_button = QPushButton("Submit")
        # self.submit_button.clicked.connect(self.submit)
        # layout.addWidget(self.submit_button)

        # coords_textbox_layout = QGridLayout()

        # self.x_coord = QLineEdit(self)
        # self.x_coord.setPlaceholderText("Enter coordinate (X)")
        # coords_textbox_layout.addWidget(self.x_coord, 0, 0)

        # self.y_coord = QLineEdit(self)
        # self.y_coord.setPlaceholderText("Enter coordinate (Y)")
        # coords_textbox_layout.addWidget(self.y_coord, 0, 1)

        # self.coords_velocity = QLineEdit(self)
        # self.coords_velocity.setPlaceholderText("Enter velocity")
        # coords_textbox_layout.addWidget(self.coords_velocity, 0, 2)

        # self.coords_duration = QLineEdit(self)
        # self.coords_duration.setPlaceholderText("Enter duration")
        # coords_textbox_layout.addWidget(self.coords_duration, 0, 3)

        # self.x_coord.setToolTip("Coordinate in X (0-640)")
        # self.y_coord.setToolTip("Coordinate in Y (0-480)")
        # self.coords_velocity.setToolTip("Speed magnitude in m/s")
        # self.coords_duration.setToolTip("Duration in s")
        
        # layout.addLayout(coords_textbox_layout)

        # self.coords_submit_button = QPushButton("Submit")
        # self.coords_submit_button.clicked.connect(self.coords_submit)
        # layout.addWidget(self.coords_submit_button)
        
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
        # self.node.twist.linear.y = self.pitch_slider.value() / 100.0
        # self.node.twist.linear.x = -(self.roll_slider.value() / 100.0)
        self.node.twist.linear.x = self.pitch_slider.value() / 50.0
        self.node.twist.linear.y = self.roll_slider.value() / 50.0
        self.node.twist.linear.z = self.throttle_slider.value() / 100.0
        self.node.twist.angular.z = self.yaw_slider.value() / 100.0

    def arm_logic(self):
        if self.node.current_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            QMessageBox.warning(self, "Arming Error", "Drone is already armed!")
        else:
            self.node.arm_drone(True)
            self.node.get_logger().info("Arm button pressed, arming...")

    def land_logic(self):
        self.node.land_drone()

    def return_to_launch(self):
        self.node.rtl_request_pub.publish(Bool(data=True))
        self.node.get_logger().info("Return to Launch (RTL) command sent")
    
    def hold_logic(self):
        self.node.hold_request_pub.publish(Bool(data=True))
        self.node.get_logger().info("Switch to Hold Mode command sent")

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