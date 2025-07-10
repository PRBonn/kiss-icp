# KISS-ICP ROS 2 Wrapper

### How to build

You should not need any extra dependency, just clone and build:

```sh
git clone https://github.com/PRBonn/kiss-icp
colcon build
source ./install/setup.bash
```

### How to run

The only required argument to provide is the **topic name** so KISS-ICP knows which PointCloud2 to process:

```sh
ros2 launch kiss_icp odometry.launch.py bagfile:=<path_to_rosbag> topic:=<topic_name>
```

You can optionally launch the node with any bagfile, and play the bagfiles on a different shell:

```sh
ros2 launch kiss_icp odometry.launch.py topic:=<topic_name>
```

and then,

```sh
ros2 bag play <path>*.bag
```

A preconfigured Rviz2 window is launched by default. To disable it, you can set the `visualize` launch argument to `false`:
```sh
ros2 launch kiss_icp odometry.launch.py topic:=<topic_name> visualize:=false
```

### Configuration

The parameters for the KISS-ICP algorithm itself are written in a yaml configuration file. They are meant to be set before launching the node. An **example** yaml file is given in config/config.yaml. The file parsed by default is located in the share directory of the kiss_icp package (in your workspace that would be `install/kiss_icp/share/kiss_icp/config/config.yaml`), but any file path can be provided with the `config_file` launch argument, e.g.: `ros2 launch kiss_icp odometry.launch.py config_file:=/path/to/my/config_file.yaml`.

The ROS-related parameters can be set via command-line when launching the node:
* `base_frame`
* `lidar_odom_frame`
* `publish_odom_tf`
* `invert_odom_tf`
* `position_covariance`
* `orientation_covariance`

You can set them like so:
```sh
ros2 launch kiss_icp odometry.launch.py topic:=<topic_name> <parameter_name>:=<parameter_value>
```
For example:
```sh
ros2 launch kiss_icp odometry.launch.py topic:=/lidar/points base_frame:=lidar_cloud
```
Note that if you set the ROS-related parameters in the yaml configuration file and via command-line, the values in the yaml file will be selected in the end.

## Out of source builds

Good news! If you don't have git or you don't need to change the core KISS-ICP library, you can just
copy paste this folder into your ROS2 workspace and build as usual. The build system will fetch
the latest stable release for you.

## Looking how to run KITTI on ROS along with KISS-ICP?

I believe you could use the python API instead, to avoid converting all the data to a format that is
not the original one. If you still insist in doing it, I've created a separate repository for this,
just clone and build [this package](https://github.com/nachovizzo/kiss_icp_kitti)
