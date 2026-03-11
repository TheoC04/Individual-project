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

4. Run the project
    ```bash
    [run command]
    ```

## Project Description
[Add a brief description of what this project does]

## License
[Specify your license]