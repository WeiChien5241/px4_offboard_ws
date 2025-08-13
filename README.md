# Offboard Example with PX4 (ROS2 & MAVSDK)

## Overview

---

This repo provides an example of using offboard mode in PX4 with a simple gui. The code is written in Python and includes a version using ROS2 and the other with MAVSDK. 

## SITL Demo

---

To view the demo in SITL, check out the recording here: https://youtu.be/KBl8BWIlt5U?si=wn5qFk6GNARu-g41

## Note

---

To use this package, make sure the following have been installed. 

- Laptop:
    - QGC for connecting with the drone (SITL and real drone)
- RPi running on 24.04, ROS2 Jazzy
    - uXRCE-DDS Agent

This guide has several assumptions:

- If using ROS2, assumed agent and client has already been connected
- Assumes connection between QGC and drone is established
- Assumes ROS2 functionality is enabled in companion computer.

If you need help with setting up your drone, please visit this guide for more info:

 [PX4 101 Guide](https://www.notion.so/PX4-101-Guide-222f130fcb60800c8afffc4887eca054?pvs=21)  for more info

## How to setup

---

This repo is a workspace package so it contains the src folder already. To be able to use this workspace follow:

1. Go to your home directory and do:
    
    ```powershell
    git clone https://github.com/WeiChien5241/px4_offboard_ws.git
    ```
    
2. Clone px4_msgs inside the src folder so that PX4 can communicate with ROS2
    1. Make sure the px4_msg matches the version of your px4 firmware, this can be checked via QGC, in vehicle configuration. The code has been tested with v1.15.
    
    ```powershell
    cd px4_offboard_ws/src
    git clone https://github.com/PX4/px4_msgs.git
    ```
    
    b. To check your version, go to px4_msgs folder and type
    
    ```powershell
    git branch
    ```
    
    If not in matching version, fetch all branches and switch to the desired branch
    
    ```powershell
    git fetch origin
    git checkout release/1.15
    git branch
    ```
    
3.  Build and source your workspace
    
    ```powershell
    cd ..
    colcon build
    cd ~/
    nano .bashrc # or use gedit .bashrc
    source ~/px4_offboard/install/setup.bash
    ```
    
4. Now you can go back to your workspace and run the code

## Running the code

---

You can test the code with the ROS2 version or the MAVSDK version

1. ROS2:
    1. build and source
        
        ```powershell
        cd px4_offboard_ws
        colcon build
        source install/setup.bash
        ```
        
    2. run the code (you will need two terminals)
        - Start vel_ctrl, the file that actually controls the drone
            
            ```powershell
            ros2 run offboard_pkg vel_ctrl
            ```
            
        - Start gui_control, which pops out a GUI window
            
            ```powershell
            ros2 run offboard_pkg gui_control 
            ```
            
2. MAVSDK
    1. Go to the location:
        
        ```powershell
        cd px4_offboard_ws/src/offboard_pkg/offboard_pkg
        ```
        
    2. Run the file:
        
        This file combines the vel_ctrl and gui_control file
        
        ```powershell
        ./sdk_gui_control.py
        ```
        

## Explanation

---

### How to control the drone:

- Offboard mode needs to be activated to be able to use the sliders and the canvas
- To use offboard mode, arm and takeoff the drone, once in the air, you can switch to offboard mode
- You can use sliders to control the drone or the canvas
- Think of the canvas as the 2D POV of the drone, in which the center is the drone
    - Click on the canvas to specify a point, input velocity and duration to make the drone fly
    - To make the drone fly backwards, make the velocity to be negative
- To exit offboard mode, click on the button again to enter position mode
- You can land the drone with the land or RTL button

### Slider and Canvas Mechanism:

Both of the controls uses trajectory setpoint to send velocity data to the drone. 

The sliders are able to control the roll, pitch, yaw and throttle of the drone, basically simulating an RC controller but using a GUI to do so. Once the sliders are released, they go back to their original position. 

The canvas simulates a drone’s POV if the drone has a camera. It assumes the center is the drone. By using the FOV of a camera and the resolution of the image, we are able to calculate the angles of the drone relative to the point specified. If assuming a Forward Right Up coordinate system, where X is to the front, Y to the right, Z to the up. The angles refer to the angle relative to the XY starting from X, and angle relative to XZ, starting from X. Using these two angles, we are able to calculate the three vector velocities based on the total velocity given using simple trig calculations. The calculations can be seen inside the code. 

More information regarding FOV and how it works can be seen here:

- https://www.edmundoptics.com/knowledge-center/application-notes/imaging/understanding-focal-length-and-field-of-view/?srsltid=AfmBOoqemFy01YILm0pvmasb5QDCxgTmRiOgG3zhFDYQ6EXrPSH-i8cr
- https://www.e-consystems.com/calculator/calculate-lens-fov-and-focal-length.asp#
- https://blog.arducam.com/fov-calculator/
- https://wavelength-oe.com/optical-calculators/field-of-view/