#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QSlider, QLabel, QVBoxLayout, QWidget, QMessageBox, QGridLayout, QLineEdit, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QPen, QBrush, QFont
import math
import asyncio
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.telemetry import FlightMode, LandedState
import threading

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
    
    def mousePressEvent(self, event):
        """Gets coordinate from left mouse click"""
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
        """Calculates angles from pixel coordinates based on the actual resolution and field of view"""
        offset_x = x - self.ACTUAL_WIDTH // 2
        offset_y = y - self.ACTUAL_HEIGHT // 2
        angle_xy = (offset_x / (self.ACTUAL_WIDTH / 2)) * (self.HFOV_DEG / 2)
        angle_z = -(offset_y / (self.ACTUAL_HEIGHT / 2)) * (self.VFOV_DEG / 2)
        return angle_xy, angle_z

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw grid
        painter.setPen(QPen(Qt.lightGray, 1))
        grid_size = 20
        for x in range(0, self.display_width, grid_size):
            painter.drawLine(x, 0, x, self.display_height)
        for y in range(0, self.display_height, grid_size):
            painter.drawLine(0, y, self.display_width, y)

        # Draw center lines
        painter.setPen(QPen(Qt.red, 2))
        painter.drawLine(self.center_x, 0, self.center_x, self.display_height)
        painter.drawLine(0, self.center_y, self.display_width, self.center_y)

        painter.setBrush(QBrush(Qt.red))
        painter.drawEllipse(self.center_x - 3, self.center_y - 3, 6, 6)

        painter.setFont(QFont("Arial", 10))
        painter.setPen(QPen(Qt.black, 1))
        
        # Angle labels
        painter.drawText(10, self.center_y - 10, f"-{self.HFOV_DEG/2}°")
        painter.drawText(self.display_width - 40, self.center_y - 10, f"+{self.HFOV_DEG/2}°")
        painter.drawText(self.center_x + 10, 20, f"+{self.VFOV_DEG/2}°")
        painter.drawText(self.center_x + 10, self.display_height - 10, f"-{self.VFOV_DEG/2}°")
        painter.drawText(self.center_x + 10, self.center_y + 20, "Center (0°, 0°)")

        # Draw target point
        if self.disp_x is not None and self.disp_y is not None:
            painter.setBrush(QBrush(Qt.blue))
            painter.setPen(QPen(Qt.blue, 2))
            painter.drawEllipse(self.disp_x - 5, self.disp_y - 5, 10, 10)
            angle_xy, angle_z = self.get_angles_from_pixel(self.actual_x, self.actual_y)
            painter.drawText(self.disp_x + 10, self.disp_y - 10, f"({self.actual_x}, {self.actual_y})")
            painter.drawText(self.disp_x + 10, self.disp_y + 10, f"XY: {angle_xy:.1f}°, Z: {angle_z:.1f}°")

def velocity_from_angles(vel, angle_xy, angle_z):
    """Convert velocity magnitude and angles to velocity components"""
    v_forward = vel * math.cos(math.radians(angle_z)) * math.cos(math.radians(angle_xy))
    v_right = vel * math.cos(math.radians(angle_z)) * math.sin(math.radians(angle_xy))
    v_down = -(vel * math.sin(math.radians(angle_z)))
    return v_forward, v_right, v_down

class DroneControl:
    """MAVSDK-based drone control system with state machine"""
    
    def __init__(self):
        self.drone = System()
        self.current_state = "HOLD"
        self.last_state = self.current_state
        
        # Status variables
        self.is_armed = False
        self.is_landed = True
        self.flight_mode = None
        self.health_ok = False
        
        # Control variables
        self.velocity = [0.0, 0.0, 0.0]  # [forward, right, down]
        self.yaw_rate = 0.0
        self.offboard_active = False
        
        # Request flags
        self.arm_requested = False
        self.land_requested = False
        self.rtl_requested = False
        self.offboard_requested = False
        self.hold_requested = False
        
        # Other flags
        self.fsm_paused = False
        self.allow_offboard_switch = True
        self.rc_override = False
        self.land_command_sent = False
        
        self.window = None
        self.loop = None
        
    async def connect(self, connection_string="udp://:14540"):
        """Connect to the drone"""
        print(f"Connecting to drone at {connection_string}...")
        await self.drone.connect(system_address=connection_string)

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break

        print("Waiting for global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                self.health_ok = True
                break

    async def monitor_telemetry(self):
        """Monitor drone telemetry and update status"""
        # Monitor armed state
        async def monitor_armed():
            async for armed in self.drone.telemetry.armed():
                if self.is_armed != armed:
                    print(f"Armed state changed: {armed}")
                    self.is_armed = armed
                    QTimer.singleShot(0, self.window.update_gui_state)

        # Monitor landed state
        async def monitor_landed():
            async for landed_state in self.drone.telemetry.landed_state():
                is_landed = (landed_state == LandedState.ON_GROUND)
                if self.is_landed != is_landed:
                    print(f"Landed state changed: {is_landed}")
                    self.is_landed = is_landed
                    QTimer.singleShot(0, self.window.update_gui_state)

        # Monitor flight mode
        async def monitor_flight_mode():
            async for mode in self.drone.telemetry.flight_mode():
                if self.flight_mode != mode:
                    print(f"Flight mode changed: {mode}")
                    # Check for RC override
                    if mode in [FlightMode.POSCTL, FlightMode.MANUAL] and self.current_state == "OFFBOARD":
                        self.rc_override = True
                        self.hold_requested = True
                        print("RC override detected")
                    
                    self.flight_mode = mode
                    QTimer.singleShot(0, self.window.update_gui_state)

        # Run all monitoring tasks concurrently
        await asyncio.gather(
            monitor_armed(),
            monitor_landed(),
            monitor_flight_mode()
        )

    async def state_machine(self):
        """Main state machine for drone control"""
        while True:
            if not self.fsm_paused and not self.rc_override:
                if self.current_state == "HOLD":
                    self.offboard_active = False
                    if (not self.is_armed and not self.arm_requested) or self.hold_requested:
                        if self.flight_mode != FlightMode.HOLD:
                            await self.drone.action.hold()
                    elif self._should_transition_to_landing():
                        await self._transition_to_landing()
                    elif self.offboard_requested and self.is_armed:
                        await self._transition_to_offboard()
                    elif self.health_ok and self.arm_requested and self.is_landed and not self.hold_requested:
                        self.current_state = "ARM_AND_TAKEOFF"
                        print("Transitioning to ARM_AND_TAKEOFF")

                elif self.current_state == "ARM_AND_TAKEOFF":
                    if not self.health_ok:
                        self.current_state = "HOLD"
                        print("Health check failed, returning to HOLD")
                        return
                    if self.flight_mode == FlightMode.HOLD and self.is_armed and not self.is_landed:
                        self.current_state = "HOLD"
                        self.arm_requested = False
                        print("Takeoff complete, transitioning to HOLD")
                    elif self._should_transition_to_landing():
                        await self._transition_to_landing()
                    else:
                        if not self.is_armed:
                            await self.drone.action.arm()
                        await self.drone.action.set_takeoff_altitude(5.0)
                        if not self.flight_mode ==  FlightMode.TAKEOFF:
                            await self.drone.action.takeoff()

                elif self.current_state == "OFFBOARD":
                    if not self.health_ok or not self.is_armed:
                        self.offboard_active = False
                        self.allow_offboard_switch = False
                        self.current_state = "HOLD"
                        print("Safety check failed, returning to HOLD")
                    elif self._should_transition_to_landing():
                        await self._transition_to_landing()
                    elif self.hold_requested:
                        self.offboard_active = False
                        self.allow_offboard_switch = False
                        self.current_state = "HOLD"
                        print("HOLD requested, returning to HOLD")
                    elif self.allow_offboard_switch:
                        await self._maintain_offboard()

                elif self.current_state == "LANDING":
                    if self.rtl_requested:
                        if not self.flight_mode == FlightMode.RETURN_TO_LAUNCH and not self.land_command_sent:
                            await self.drone.action.return_to_launch()
                            print("RTL command sent")
                            self.land_command_sent = True
                    elif self.land_requested:
                        if not self.flight_mode == FlightMode.LAND and not self.land_command_sent:
                            await self.drone.action.land()
                            print("Land command sent")
                            self.land_command_sent = True
                    if not self.is_armed and self.is_landed:
                        self.rtl_requested = False
                        self.land_requested = False
                        self.land_command_sent = False
                        self.current_state = "HOLD"
                        print("Landing complete, returning to HOLD")
            
            if self.last_state != self.current_state:
                print(f"State changed: {self.last_state} -> {self.current_state}")
                self.last_state = self.current_state
            
            await asyncio.sleep(0.1)  # 10 Hz

    def _should_transition_to_landing(self):
        """Check if should transition to landing state"""
        return (self.rtl_requested or self.land_requested or 
                self.flight_mode in [FlightMode.LAND, FlightMode.RETURN_TO_LAUNCH])

    async def _transition_to_landing(self):
        """Transition to landing state"""
        print("Transitioning to LANDING")
        self.offboard_active = False
        self.allow_offboard_switch = False
        self.current_state = "LANDING"

    async def _transition_to_offboard(self):
        """Transition to offboard state"""
        if self.allow_offboard_switch:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            try:
                await self.drone.offboard.start()
                self.offboard_active = True
                self.current_state = "OFFBOARD"
                print("Transitioning to OFFBOARD")
            except OffboardError as e:
                print(f"Failed to start offboard: {e}")

    async def _maintain_offboard(self):
        """Maintain offboard control and send velocity commands"""
        if self.offboard_active:
            try:
                await self.drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(
                        self.velocity[0],  # forward
                        self.velocity[1],  # right
                        self.velocity[2],  # down
                        self.yaw_rate      # yaw rate
                    )
                )
            except OffboardError as e:
                print(f"Velocity command failed: {e}")

    # Control command methods
    def arm_drone(self):
        self.arm_requested = True
        self.fsm_paused = False
        self.rc_override = False
        self.allow_offboard_switch = True
        self.land_requested = False
        print("Arm command requested")

    def land_drone(self):
        self.land_requested = True
        self.fsm_paused = False
        self.arm_requested = False
        if self.current_state == "OFFBOARD":
            self.offboard_requested = False
            self.offboard_active = False
        print("Land command requested")

    def rtl_drone(self):
        self.rtl_requested = True
        self.fsm_paused = False
        self.arm_requested = False
        if self.current_state == "OFFBOARD":
            self.offboard_requested = False
            self.offboard_active = False
        print("RTL command requested")

    def offboard_switch(self):
        if self.current_state == "OFFBOARD":
            self.offboard_active = False
            self.hold_requested = True
            self.offboard_requested = False
            self.allow_offboard_switch = True
        else:
            self.offboard_active = False
            self.hold_requested = False
            self.offboard_requested = True
            self.allow_offboard_switch = True
        self.fsm_paused = False
        print(f"Offboard switch requested: {self.offboard_requested}")

class MainWindow(QMainWindow):
    def __init__(self, drone_control):
        super().__init__()
        self.drone_control = drone_control
        self.drone_control.window = self
        self.setWindowTitle("PX4 Drone Control GUI (MAVSDK)")
        self.setGeometry(500, 200, 500, 500)
        
        layout = QVBoxLayout()
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        # Mode Buttons
        mode_button_layout = QGridLayout()
        
        self.arm_button = QPushButton("Arm and Takeoff")
        self.arm_button.clicked.connect(self.arm_logic)
        mode_button_layout.addWidget(self.arm_button, 0, 0)

        self.land_button = QPushButton("Land and Disarm")
        self.land_button.clicked.connect(self.land_logic)
        self.land_button.setEnabled(False)
        mode_button_layout.addWidget(self.land_button, 0, 1)

        self.offboard_button = QPushButton("Switch to Offboard Mode")
        self.offboard_button.clicked.connect(self.offboard_logic)
        self.offboard_button.setEnabled(False)
        mode_button_layout.addWidget(self.offboard_button, 1, 0)

        self.rtl_button = QPushButton("Return to Launch (RTL)")
        self.rtl_button.clicked.connect(self.return_to_launch)
        self.rtl_button.setEnabled(False)
        mode_button_layout.addWidget(self.rtl_button, 1, 1)

        layout.addLayout(mode_button_layout)

        # Canvas Section
        canvas_layout = QVBoxLayout()
        canvas_layout.addWidget(QLabel("Clickable Canvas - Click to set target coordinate:"))

        self.canvas = ClickableCanvas()
        canvas_layout.addWidget(self.canvas, alignment=Qt.AlignCenter)

        canvas_controls_layout = QHBoxLayout()

        self.canvas_velocity = QLineEdit()
        self.canvas_velocity.setPlaceholderText("Velocity (m/s)")
        canvas_controls_layout.addWidget(self.canvas_velocity)

        self.canvas_duration = QLineEdit()
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
            padding: 5px;
            font-weight: bold;
            font-size: 12px;
        """)
        canvas_layout.addWidget(self.velocity_display)

        layout.addLayout(canvas_layout)

        # Manual Control Sliders
        rc_layout = QGridLayout()

        # Throttle (Z) control
        throttle_layout = QVBoxLayout()
        self.throttle_slider = QSlider(Qt.Vertical)
        self.throttle_slider.setRange(-100, 100)
        self.throttle_slider.setFixedHeight(200)
        self.throttle_slider.valueChanged.connect(self.update_velocity)
        self.throttle_slider.sliderReleased.connect(self.reset_sliders)
        throttle_layout.addWidget(QLabel("Z Velocity (m/s)"), alignment=Qt.AlignCenter)
        throttle_layout.addWidget(self.throttle_slider, alignment=Qt.AlignCenter)
        rc_layout.addLayout(throttle_layout, 0, 0)

        # Yaw control
        yaw_layout = QVBoxLayout()
        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setRange(-100, 100)
        self.yaw_slider.setFixedWidth(200)
        self.yaw_slider.valueChanged.connect(self.update_velocity)
        self.yaw_slider.sliderReleased.connect(self.reset_sliders)
        yaw_layout.addWidget(QLabel("Yaw Rate (rad/s)"), alignment=Qt.AlignCenter)
        yaw_layout.addWidget(self.yaw_slider, alignment=Qt.AlignCenter)
        rc_layout.addLayout(yaw_layout, 1, 0)
        
        # Pitch (X) control
        pitch_layout = QVBoxLayout()
        self.pitch_slider = QSlider(Qt.Vertical)
        self.pitch_slider.setRange(-100, 100)
        self.pitch_slider.setFixedHeight(200)
        self.pitch_slider.valueChanged.connect(self.update_velocity)
        self.pitch_slider.sliderReleased.connect(self.reset_sliders)
        pitch_layout.addWidget(QLabel("X Velocity (m/s)"), alignment=Qt.AlignCenter)
        pitch_layout.addWidget(self.pitch_slider, alignment=Qt.AlignCenter)
        rc_layout.addLayout(pitch_layout, 0, 1)

        # Roll (Y) control
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

    def update_gui_state(self):
        """Update GUI button states based on drone status"""
        is_armed = self.drone_control.is_armed
        is_landed = self.drone_control.is_landed
        flight_mode = self.drone_control.flight_mode
        current_state = self.drone_control.current_state

        if is_armed and not is_landed:
            self.arm_button.setText("Armed")
            self.arm_button.setEnabled(False)
            if flight_mode == FlightMode.LAND:
                self.land_button.setText("Landing...")
                self.land_button.setEnabled(False)
                self.rtl_button.setEnabled(False)
                self.offboard_button.setEnabled(False)
            else:
                self.land_button.setText("Land and Disarm")
                self.land_button.setEnabled(True)
                self.rtl_button.setEnabled(True)
                self.offboard_button.setEnabled(True)
                if current_state == "OFFBOARD":
                    self.offboard_button.setText("Switch to Hold Mode")
                else:
                    self.offboard_button.setText("Switch to Offboard Mode")
        elif not is_armed and is_landed:
            self.arm_button.setText("Arm and Takeoff")
            self.arm_button.setEnabled(True)
            self.land_button.setText("Land and Disarm")
            self.land_button.setEnabled(False)
            self.rtl_button.setEnabled(False)
            self.offboard_button.setEnabled(False)
        else:
            self.arm_button.setText("Arm and Takeoff")
            self.arm_button.setEnabled(not is_armed)
            self.land_button.setEnabled(False)
            self.rtl_button.setEnabled(False)
            self.offboard_button.setEnabled(False)

    def on_canvas_click(self):
        self.canvas_execute_button.setEnabled(True)

    def clear_canvas_target(self):
        self.canvas.disp_x = None
        self.canvas.disp_y = None
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
        v_forward, v_right, v_down = velocity_from_angles(canvas_vel, angle_xy, angle_z)

        self.drone_control.velocity = [v_forward, v_right, v_down]
        self.drone_control.yaw_rate = 0.0

        print(f"Canvas target velocity set: forward={v_forward:.2f}, right={v_right:.2f}, down={v_down:.2f}")
        self.velocity_display.setText(f"Velocity: x={v_forward:.2f}, y={v_right:.2f}, z={v_down:.2f}")

        QTimer.singleShot(int(canvas_duration_time * 1000), self.stop_velocity)

    def stop_velocity(self):
        self.drone_control.velocity = [0.0, 0.0, 0.0]
        self.drone_control.yaw_rate = 0.0
        self.velocity_display.setText("Velocity: x=0.00, y=0.00, z=0.00")
        print("Velocity stopped")

    def reset_sliders(self):
        self.pitch_slider.setValue(0)
        self.throttle_slider.setValue(0)
        self.roll_slider.setValue(0)
        self.yaw_slider.setValue(0)
        self.update_velocity()
    
    def update_velocity(self):
        self.drone_control.velocity = [
            self.pitch_slider.value() / 50.0,     # forward
            self.roll_slider.value() / 50.0,      # right
            -self.throttle_slider.value() / 100.0  # down (negative for up)
        ]
        self.drone_control.yaw_rate = self.yaw_slider.value()

    def arm_logic(self):
        self.drone_control.arm_drone()

    def land_logic(self):
        self.drone_control.land_drone()

    def return_to_launch(self):
        self.drone_control.rtl_drone()

    def offboard_logic(self):
        self.drone_control.offboard_switch()

def run_asyncio_loop(loop):
    """Run asyncio event loop in separate thread"""
    asyncio.set_event_loop(loop)
    loop.run_forever()

async def start_drone_control(drone_control):
    """Start all drone control tasks"""
    await drone_control.connect()
    await asyncio.gather(
        drone_control.monitor_telemetry(),
        drone_control.state_machine()
    )

def main():
    print("Starting MAVSDK Drone Control GUI...")
    
    # Create PyQt application
    app = QApplication(sys.argv)
    
    # Create asyncio event loop for drone control
    loop = asyncio.new_event_loop()
    
    # Create drone control system
    drone_control = DroneControl()
    drone_control.loop = loop
    
    # Create and show main window
    window = MainWindow(drone_control)
    window.show()
    
    # Start asyncio loop in separate thread
    thread = threading.Thread(target=run_asyncio_loop, args=(loop,), daemon=True)
    thread.start()
    
    # Start drone control tasks
    asyncio.run_coroutine_threadsafe(start_drone_control(drone_control), loop)
    
    print("GUI started. Connect to drone and start controlling!")
    
    # Run PyQt event loop
    try:
        app.exec_()
    finally:
        loop.call_soon_threadsafe(loop.stop)

if __name__ == '__main__':
    main()