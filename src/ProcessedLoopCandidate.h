#pragma once

/** Stores info on the loop candidates. Also contains geometric info like
    number of pf-matches, how it was calculated, and the relative pose
**/


#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>


//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

// ros
// #include <ros/ros.h>
// #include <ros/package.h>

// ros msg
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PoseWithCovariance.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/PointCloud.h>
// #include <geometry_msgs/Point32.h>
// #include <sensor_msgs/Image.h>
// // #include <nav_msgs/Path.h>
// #include <geometry_msgs/Point.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

#include "DataNode.h"
#include "utils/PoseManipUtils.h"
// #include "utils/MiscUtils.h"
#include "utils/TermColor.h"

#include "../utils/nlohmann/json.hpp"
using json = nlohmann::json;

// Publishing message
#include <cerebro/LoopEdge.h>

class ProcessedLoopCandidate
{
public:
    ProcessedLoopCandidate( int idx_from_raw_candidates_list, DataNode * node_1, DataNode * node_2 );
    ProcessedLoopCandidate() { idx_from_raw_candidates_list=-1; opX_b_T_a.clear(); debug_images.clear(); }

    bool makeLoopEdgeMsg(  cerebro::LoopEdge& msg );

    // Looks at all the candidate relative poses and decides if they are all consistent.
    // Returns true if consistent, returns false if not consistent
    bool makeLoopEdgeMsgWithConsistencyCheck( cerebro::LoopEdge& msg );

    bool asJson( json& out_json );

// private:
    DataNode * node_1;
    DataNode * node_2;

    int idx_from_datamanager_1;
    int idx_from_datamanager_2;

    int idx_from_raw_candidates_list;

    int pf_matches; //pf==>point-features


    int _3d2d_n_pfvalid_depth;
    Matrix4d _3d2d__2T1; //used 3d points from 1st view, 2d points from 2nd view.
    bool isSet_3d2d__2T1=false; //< final_pose_available
    float _3d2d__2T1__ransac_confidence;


    // std::mutex _mutex;

    // All computed poses
    vector<Matrix4d> opX_b_T_a; // b_T_a is same as 2T1
    vector<float> opX_goodness;
    vector<string> opX_b_T_a_name;
    vector<string> opX_b_T_a_debugmsg;

    // debug images
    vector<cv::Mat> debug_images;
    vector<string> debug_images_titles;



};
