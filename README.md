To use this package, make sure the following have been installed. If need help with those, please visit: [PX4 101 Guide](https://www.notion.so/PX4-101-Guide-222f130fcb60800c8afffc4887eca054?pvs=21)  for more info

This repo is a workspace package so it contains the src folder already. To be able to use this workspace follow:

1. go to your workspace directory and do:
    
    ```powershell
    git clone https://github.com/WeiChien5241/px4_offboard.git
    ```
    
2. clone px4_msgs inside the src folder so that PX4 can communicate with ROS2
    1. make sure the px4_msg matches the version of your px4 firmware, this can be checked via QGC, in vehicle configuration. The code has been tested with v1.15.
    
    ```powershell
    cd px4_offboard/src
    git clone https://github.com/PX4/px4_msgs.git
    ```
    
3. go back to the workspace level and build and source your workspace
    
    ```powershell
    cd ..
    colcon build
    cd ~/
    nano .bashrc # or use gedit .bashrc
    source ~/px4_offboard/install/setup.bash
    ```
    
4. Now you can go back to your workspace and run the code
    1. build and source again just to make sure, use two terminals
    
    ```powershell
    cd px4_offboard
    colcon build
    source install/setup.bash
    ros2 run offboard_pkg vel_ctrl
    ros2 run offboard_pkg gui_control 
    ```
    
5. now you can control the drone
    1. gui control:
        1. pops a gui open
        2. to be able to use the sliders, offboard mode needs to be activated
        3. press arm and takeoff to make the drone fly
        4. click offboard once takeoff is completed and enters hold mode
        5. once in offboard, sliders are activated, so is the canvas and the textboxes
        6. if want to exit offboard, press the button again to enter hold mode
        7. two landing options are provided, land or return to land