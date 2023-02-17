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

In first instance, if you really need to run the experiments, just `$ pip install kiss-icp` and use
the python API, it's definitely easier. If for whatever reason you insist in using the
`kitti2rosbag` dataset, then we have provided a helper node that should correct all the mistakes
present in the dataset. To launch it just do:

```sh
roslaunch kiss_icp kitti.launch bagfile:=2011_09_30_drive_0027_sync.bag
```

## How to run the evaluation

This is WIP, any suggestion on how this should be done is more than welcome! Just open an issue with
your proposal.

