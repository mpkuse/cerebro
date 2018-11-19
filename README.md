# VINS-Mono with Cerebro
This is the cerebro module for vins-mono. The aim of this project
is better loop detection. The cerebro node connects to vins-mono
(with ros interface). The DataManager class handles
all the incoming data. Visualization handles all the visualization.
Cerebro class handles the loop closure intelligence part.
This is a multi-threaded object oriented implementation.

## Datasets
I have setup with
- [ETHZ EuroC Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [TUM Visual Inertial Dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)
- UPenn - [Penncosyvio](https://github.com/daniilidis-group/penncosyvio)
- [ADVIO](https://github.com/AaltoVision/ADVIO) ARKIT, Tango logging.
- Blackbox4 (dji imu+color bluefox)
- point_grey (microstain imu+point grey color camera)
- HK Mall (using mynt stereo cam)

## How to compile

### Get Vins-mono working
```
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

Make sure your vins-mono can compile correctly. See vins-mono github repo
for the latest information on prerequisites and compilation instructions.

### Cerebro
```
cd catkin_ws/src/
git clone https://github.com/mpkuse/cerebro
cd ../
catkin_make
```

### vins_mono_debug_pkg
With cerebro node it is possible to live run the vins and make it log all the
details to file for firther analysis/debugging. This might be useful
for researchers and other Ph.D. student to help VINS-mono improve further.
see [github/mpkuse/vins_mono](https://github.com/mpkuse/vins_mono_debug_pkg).

## How to run
```
roslaunch cerebro tum.launch
```

## How to run - Docker
You can use my docker image which has all the prerequisites already installed.
TODO 
