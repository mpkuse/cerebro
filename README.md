# VINS-Mono with Cerebro
This is the cerebro module for vins-mono. The aim of this project
is better loop detection. The cerebro node connects to vins-mono
(with ros interface). The DataManager class handles
all the incoming data. Visualization handles all the visualization.
Cerebro class handles the loop closure intelligence part.
This is a multi-threaded object oriented implementation.

[![IMAGE ALT TEXT](http://img.youtube.com/vi/KDRo9LpL6Hs/0.jpg)](http://www.youtube.com/watch?v=KDRo9LpL6Hs "Video Title")



[![IMAGE ALT TEXT](http://img.youtube.com/vi/XvoCrLFq99I/0.jpg)](http://www.youtube.com/watch?v=XvoCrLFq99I "Video Title")

[![IMAGE ALT TEXT](http://img.youtube.com/vi/MMLyNtDNsZE/0.jpg)](http://www.youtube.com/watch?v=MMLyNtDNsZE "Video Title")

For more demonstration, have a look at my [youtube playlist](https://www.youtube.com/playlist?list=PLWyydx20vdPzs5VVhZu0TGsReT7U17Fxp)


## Datasets
I have setup with
- [ETHZ EuroC Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [TUM Visual Inertial Dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)
- UPenn - [Penncosyvio](https://github.com/daniilidis-group/penncosyvio)
- [ADVIO](https://github.com/AaltoVision/ADVIO) ARKIT, Tango logging.
- Blackbox4 (dji imu+color bluefox)
- point_grey (microstain imu+point grey color camera)
- HK Mall (using mynt stereo cam)

## How to run - Docker
I highly recommend the already deployed packages with docker.
Run the roscore on your host pc and all the packages run inside
of docker container. rviz runs on the host pc.

I assume you have a PC with a graphics card and cuda9 working smoothly.
```
$(host) export ROS_HOSTNAME=`hostname`
$(host) roscore
# assume that host has the ip address 172.17.0.1 in docker-network aka docker0
$(host) docker run --runtime=nvidia -it \
        -v /media/mpkuse/Bulk_Data/:/Bulk_Data \ #TODO-remove
        -v /home/mpkuse/docker_ws_slam:/app \    #TODO-remove
        --add-host `hostname`:172.17.0.1 \
        --env ROS_MASTER_URI=http://`hostname`:11311/ \
        --env CUDA_VISIBLE_DEVICES=0 \
        --hostname happy_go \
        --name happy_go  \
        mpkuse/kusevisionkit:ros-kinetic-vins bash
$(host) roscd cerebro; rviz -d ./rviz/good-viz.rviz
```

If you are unfamiliar with docker, you may want to read [my blog post](https://kusemanohar.wordpress.com/2018/10/03/docker-for-computer-vision-researchers/)
on using docker for computer vision researchers.
You might want to have a look at my test ros-package to ensure things work with docker [docker_ros_test](https://github.com/mpkuse/docker_ros_test).


## How to compile (from scratch)
You will need a) vins-mono (with modification for reset by mpkuse), b) cerebro
and c) solve_keyframe_pose_graph.

### Dependencies
- ROS Kinetic
- Eigen
- Ceres
- OpenCV 3
- [Theia-sfm](http://theia-sfm.org/) (Optional)

### Get Vins-mono working
```
cd ~/catkin_ws/src
#git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
git clone https://github.com/mpkuse/VINS-Mono
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

Make sure your vins-mono can compile correctly. See vins-mono github repo
for the latest information on prerequisites and compilation instructions.
For compatibility i recommend using my fork of vins-mono. Some minor
modifications have been made by me for working with kidnap cases.
My changes are limited to the file `vins_estimator/src/estimator_node.cpp`.

### Cerebro
```
cd catkin_ws/src/
git clone https://github.com/mpkuse/cerebro
cd ../
catkin_make
```

This have 2 exectables. a) ros server that takes as input an image and
returns a image descriptor. b) cerebro_node, this
finds the loop candidates and computes the relative poses. I have also
included my trained model (about 3mb) in this package. If you wish
to train your own model, you may find [my learning code here](https://github.com/mpkuse/cartwheel_train).

### Pose Graph Solver
Use my pose graph solver. The differences between this implementation and the
original from VINS-mono is that mine can handle kidnap cases and it
uses a switch-constraint formulation of the graph problem.

```
cd catkin_ws/src/
git clone https://github.com/mpkuse/solve_keyframe_pose_graph
cd ../
catkin_make
```

### vins_mono_debug_pkg (optional, needed only if you wish to debug vins-mono)
With cerebro node it is possible to live run the vins and make it log all the
details to file for firther analysis/debugging. This might be useful
for researchers and other Ph.D. student to help VINS-mono improve further.
see [github/mpkuse/vins_mono](https://github.com/mpkuse/vins_mono_debug_pkg).

## How to run
```
roslaunch cerebro mynteye.launch
```
