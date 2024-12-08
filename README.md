# Camera Saver

Camera Saver is a ROS 2 package providing tools for saving image streams from the ULTIMATE project's rosbag topics.

## Features

- Save multiple image streams from ROS 2 topics and convert them into one jpg.
- The name of each saved image will correspond to the simulation time the image was taken (from a chosen camera).
- Produce a .csv file with names of files and corresponding UNIX timestamps of each image topic.

(all camera stream timestamps differ, hence one camera timestamp was chosen to unify annotation )

## Installation

1. Clone the repository:
    ```sh
    cd ~/ros2_ws/src
    git clone https://github.com/kackac2000/camera_saver.git
    ```

2. Build the ROS 2 package:
    ```sh
    cd ~/ros2_ws
    colcon build --packages-select camera_saver
    ```

## Usage

1. Source the workspace:
    ```sh
    source install/setup.bash
    ```

2. Run the node to save images:
    ```sh
    ros2 run camera_saver image_saver_node
    ```
    
3. Play the rosbag:
    ```sh
    ros2 bag play <your_ros_bag/>
    ```

All images will be saved in the *saved_images_from_Ultimate_dataset* folder, with the additional .csv file listing all the files and their corresponding cameras' timestamps(UNIX).

    
