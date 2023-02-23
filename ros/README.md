# KISS-ICP ROS1 node

This node is just an application example on how to use the KISS-ICP C++ API. It's still considered
work in progress and we are very happy to receive any contribution from the comunity 👼

https://user-images.githubusercontent.com/21349875/214578180-b1d2431c-8fff-440e-aa6e-99a1d85989b5.mp4

## How to build

You should not need any extra dependency, just clone and build:

```sh
cd ~/catkin_ws/
git clone https://github.com/PRBonn/kiss-icp 
catkin build
source devel/setup.bash
```

## How to run

Example with ouster-bagfiles (newer college, etc)

```sh
roslaunch kiss_icp odometry.launch bagfile:=<path_to_rosbag> topic:=/os1_cloud_node/points
```

You can optionally launch the node with any bagfile, and play the bagfiles on a different shell:

```sh
roslaunch kiss_icp odometry.launch topic:=/os1_cloud_node/points
```

and then,

```sh
rosbag play <path>*.bag
```

## Special case: how to run on KITTI

TODO: add external repo to avoid extra uglyness

## How to run the evaluation

This is WIP, any suggestion on how this should be done is more than welcome! Just open an issue with
your proposal.

