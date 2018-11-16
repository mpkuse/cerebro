# Camodocal

Original Implementation : https://github.com/hengli/camodocal
My (Manohar Kuse, mpkuse@connect.ust.hk) Adaptation : https://github.com/mpkuse/camera_model

## CMakefile.txt
If you put this folder under `src/utils/` and CMakeLists.txt is in `./`
you will need to add the following to you CMake files

```
include_directories(src/utils/camodocal/include)
FILE(GLOB CamodocalCameraModelSources
        src/utils/camodocal/src/chessboard/Chessboard.cc
        src/utils/camodocal/src/calib/CameraCalibration.cc
        src/utils/camodocal/src/camera_models/Camera.cc
        src/utils/camodocal/src/camera_models/CameraFactory.cc
        src/utils/camodocal/src/camera_models/CostFunctionFactory.cc
        src/utils/camodocal/src/camera_models/PinholeCamera.cc
        src/utils/camodocal/src/camera_models/CataCamera.cc
        src/utils/camodocal/src/camera_models/EquidistantCamera.cc
        src/utils/camodocal/src/camera_models/ScaramuzzaCamera.cc
        src/utils/camodocal/src/sparse_graph/Transform.cc
        src/utils/camodocal/src/gpl/gpl.cc
        src/utils/camodocal/src/gpl/EigenQuaternionParameterization.cc
    )
```

Dependencies if not already added:
```
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

```
