# Individual-project
## Build Instructions

### Prerequisites
- [List required tools/languages]

### usefell command reminders

1. ros2 pkg create car_control --build-type ament_python --dependencies rclpy chassis_control
2. colcon build
3. source instll/setub.bash
4. ros2 interface show chassis_control/msg/SetVelocity
5. ros2 bag record -a -o name -d 60
6. ros2 bag play name --loop
7. scp -r -v theo@theo-desktop.local:~/bags ./  # copy bags from raspberry pi to local machine
8. du -h my_bag # check bag size
9. ros2 topic bw /out/compressedDepth # check topic bandwidth
10. ros2 topic hz /out/compressedDepth # check topic frequency
11. 



### Installation Instructions

### Steps
1. Clone the repository
    ```bash
    git clone <repository-url>
    cd Individual-project
    ```

2. Install dependencies
    ```bash
    [dependency installation command]
    ```

3. Build the project
    ```bash
    colcon build
    ```

4. Source the setup file
    ```bash
    source install/setup.bash
    ```

### Running the Project

## vision

Run the camera node using the following command:

```bash
ros2 run vision camera_node
```

In another terminal, run the following command to republish the camera feed as compressed images:

```bash
ros2 run image_transport republish raw compressed --ros-args --remap in:=/camera/image_raw --remap out:=/camera/image/compressed
```

subscribe to topic out/compressed to view the compressed images. You can use rqt_image_view or any other ROS image viewer to visualize the feed.

In another terminal, run the following command to view the camera feed:

```bash
ros2 run rqt_image_view rqt_image_view
```
This can be on a separate machine on the same network, just make sure to set the ROS_DOMAIN_ID environment variable to match the one used by the camera node.

## remote control

the following command will allow you to control the car using an Xbox controller. Make sure your Xbox controller is connected and properly configured on your system.

```bash
ros2 run car_control xbox_drive 
```

## Project Description
[Add a brief description of what this project does]

## License
[Specify your license]