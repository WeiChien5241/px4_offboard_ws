#!/usr/bin/env python3
import sys
import math
import asyncio
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QSlider, QLabel, QVBoxLayout, QWidget, QMessageBox, QGridLayout, QLineEdit
from PyQt5.QtCore import Qt, QTimer
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.telemetry import FlightMode
import threading

def velocity_from_angles(vel, angle_xy, angle_z):
    v_forward = vel * math.cos(math.radians(angle_z)) * math.cos(math.radians(angle_xy))
    v_right = vel * math.cos(math.radians(angle_z)) * math.sin(math.radians(angle_xy))
    v_down = vel * math.sin(math.radians(angle_z))
    return v_forward, v_right, v_down

class DroneControl:
    def __init__(self):
        self.drone = System()
        self.current_state = "IDLE"
        self.is_armed = False
        self.is_landed = True
        self.flight_mode = None
        self.velocity = [0.0, 0.0, 0.0]  # [forward, right, down]
        self.yaw_rate = 0.0
        self.true_yaw = 0.0
        self.offboard_active = False
        self.landing_initiated = False
        self.window = None
        self.loop = asyncio.get_event_loop()

    async def connect(self):
        await self.drone.connect(system_address="udp://:14540")

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break

        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                print("-- Global position estimate OK")
                break

    async def monitor_armed(self):
        async for armed in self.drone.telemetry.armed():
            self.is_armed = armed
            self.update_gui_state()

    async def monitor_flight_mode(self):
        async for mode in self.drone.telemetry.flight_mode():
            self.flight_mode = mode
            # self.is_landed = mode in [FlightMode.HOLD, FlightMode.LAND]
            self.update_gui_state()

    async def monitor_attitude(self):
        async for attitude in self.drone.telemetry.attitude_euler():
            self.true_yaw = -math.radians(attitude.yaw_deg)

    async def state_machine(self):
        """Run state machine for drone control."""
        while True:
            if self.current_state == "IDLE":
                if self.window and self.window.arm_requested:
                    self.current_state = "ARMING"
                    print(("Switch from idle to arming"))

            elif self.current_state == "ARMING":
                if self.is_armed:
                    self.current_state = "TAKEOFF"
                    print("Switch from arming into takeoff")
                await self.arm_drone()

            elif self.current_state == "TAKEOFF":
                if self.flight_mode == FlightMode.TAKEOFF:
                    self.current_state = "LOITER"
                    print("Switching to loiter mode")
                await self.takeoff()

            elif self.current_state == "LOITER":
                if self.flight_mode == FlightMode.HOLD:
                    self.current_state = "OFFBOARD"
                    self.offboard_active = True
                    print("Switching to offboard mode")

            elif self.current_state == "OFFBOARD":
                if self.landing_initiated or self.flight_mode == FlightMode.LAND:
                    print("Landing detected, switching to LANDING state")
                    self.current_state = "LANDING"
                    self.offboard_active = False
                elif self.offboard_active:
                    await self.start_offboard()
                    await self.send_velocity()

            elif self.current_state == "LANDING":
                await self.drone.action.land()
                if not self.is_armed:
                    print("Landing complete, returning to IDLE")
                    self.current_state = "IDLE"
                    self.landing_initiated = False
                    
            await asyncio.sleep(0.02)  # 50 Hz

    async def arm_drone(self):
        if not self.is_armed:
            try:
                await self.drone.action.arm()
                print("Arming drone")
            except Exception as e:
                print(f"Arming failed: {e}")
                self.current_state = "IDLE"

    async def takeoff(self):
        if self.is_armed and self.flight_mode != FlightMode.TAKEOFF:
            try:
                await self.drone.action.set_takeoff_altitude(5.0)
                await self.drone.action.takeoff()
                print("Takeoff command sent")
            except Exception as e:
                print(f"Takeoff failed: {e}")
                self.current_state = "IDLE"

    async def start_offboard(self):
        try:
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
        except OffboardError as e:
            print(f"Starting offboard failed: {e}")
            self.current_state = "IDLE"

    async def send_velocity(self):
        """Send velocity commands in body frame."""
        cos_yaw = np.cos(self.true_yaw)
        sin_yaw = np.sin(self.true_yaw)
        # Convert world-frame (NED) to body-frame (FRD)
        vel_forward = self.velocity[0] * cos_yaw - self.velocity[1] * sin_yaw
        vel_right = self.velocity[0] * sin_yaw + self.velocity[1] * cos_yaw
        vel_down = self.velocity[2]
        try:
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(vel_forward, vel_right, vel_down, self.yaw_rate)
            )
        except OffboardError as e:
            print(f"Velocity command failed: {e}")

    def update_gui_state(self):
        """Update GUI button states based on drone status."""
        if not self.window:
            return
        if self.is_armed and not self.is_landed:
            self.window.arm_button.setText("Armed")
            self.window.arm_button.setEnabled(False)
            if self.flight_mode == FlightMode.LAND:
                self.window.land_button.setText("Landing...")
                self.window.land_button.setEnabled(False)
            else:
                self.window.land_button.setText("Land and Disarm")
                self.window.land_button.setEnabled(True)
        elif not self.is_armed and self.is_landed:
            self.window.arm_button.setText("Arm and Takeoff")
            self.window.arm_button.setEnabled(True)
            self.window.land_button.setText("Land and Disarm")
            self.window.land_button.setEnabled(False)
        else:
            self.window.arm_button.setText("Arm and Takeoff")
            self.window.arm_button.setEnabled(not self.is_armed)
            self.window.land_button.setEnabled(False)

class MainWindow(QMainWindow):
    def __init__(self, drone_control):
        super().__init__()
        self.drone_control = drone_control
        self.drone_control.window = self
        self.arm_requested = False
        self.setWindowTitle("PX4 Drone Control GUI")
        self.setGeometry(500, 200, 500, 500)

        layout = QVBoxLayout()
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        self.arm_button = QPushButton("Arm and Takeoff")
        self.arm_button.clicked.connect(self.arm_logic)
        layout.addWidget(self.arm_button)

        self.land_button = QPushButton("Land and Disarm")
        self.land_button.clicked.connect(self.land_logic)
        self.land_button.setEnabled(False)
        layout.addWidget(self.land_button)

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

        textbox_layout = QGridLayout()

        self.xy_angle = QLineEdit(self)
        self.xy_angle.setPlaceholderText("Enter angle (X)")
        textbox_layout.addWidget(self.xy_angle, 0, 0)

        self.z_angle = QLineEdit(self)
        self.z_angle.setPlaceholderText("Enter angle (Z)")
        textbox_layout.addWidget(self.z_angle, 0, 1)

        self.velocity = QLineEdit(self)
        self.velocity.setPlaceholderText("Enter velocity")
        textbox_layout.addWidget(self.velocity, 0, 2)
        
        self.duration = QLineEdit(self)
        self.duration.setPlaceholderText("Enter duration")
        textbox_layout.addWidget(self.duration, 0, 3)
        
        self.xy_angle.setToolTip("Angle in XY plane (0°=forward, 90°=right, 180°=back)")
        self.z_angle.setToolTip("Angle above XY plane (0°=flat, 90°=straight up)")
        self.velocity.setToolTip("Speed magnitude in m/s")
        self.duration.setToolTip("Duration in s")
        
        layout.addLayout(textbox_layout)

        self.submit_button = QPushButton("Submit")
        self.submit_button.clicked.connect(self.submit)
        layout.addWidget(self.submit_button)

    def submit(self):
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

        v_forward, v_right, v_down = velocity_from_angles(vel, xy_deg, z_deg)
        self.drone_control.velocity = [v_forward, v_right, v_down]
        self.drone_control.yaw_rate = 0.0
        print(f"Velocity set: forward={v_forward:.2f}, right={v_right:.2f}, down={v_down:.2f}")

        QTimer.singleShot(int(duration_time * 1000), self.stop_velocity)

    def stop_velocity(self):
        self.drone_control.velocity = [0.0, 0.0, 0.0]
        self.drone_control.yaw_rate = 0.0
        print("Velocity stopped")

    def reset_sliders(self):
        self.pitch_slider.setValue(0)
        self.throttle_slider.setValue(0)
        self.roll_slider.setValue(0)
        self.yaw_slider.setValue(0)
        self.update_velocity()

    def update_velocity(self):
        self.drone_control.velocity = [
            (self.roll_slider.value() / 50.0),  # X (forward)
            -(self.pitch_slider.value() / 50.0),   # Y (right)
            -(self.throttle_slider.value() / 100.0)  # Z (down)
        ]
        self.drone_control.yaw_rate = self.yaw_slider.value() / 100.0

    def arm_logic(self):
        if self.drone_control.is_armed:
            QMessageBox.warning(self, "Arming Error", "Drone is already armed!")
        else:
            self.arm_requested = True
            print("Arm request sent")

    def land_logic(self):
        self.drone_control.landing_initiated = True
        print("Land command sent")

def run_asyncio_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()

async def start_drone_control(drone_control):
    await drone_control.connect()
    await asyncio.gather(
        drone_control.monitor_armed(),
        drone_control.monitor_flight_mode(),
        drone_control.monitor_attitude(),
        drone_control.state_machine()
    )

def main():
    app = QApplication(sys.argv)
    loop = asyncio.new_event_loop()
    drone_control = DroneControl()
    window = MainWindow(drone_control)

    # Run asyncio loop in a separate thread
    thread = threading.Thread(target=run_asyncio_loop, args=(loop,), daemon=True)
    thread.start()

    # Start drone control tasks
    asyncio.run_coroutine_threadsafe(start_drone_control(drone_control), loop)

    window.show()
    app.exec_()

if __name__ == '__main__':
    main()