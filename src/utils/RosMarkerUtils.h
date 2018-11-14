#pragma once

//
// This provides utilities creating ros markers
//
// Author : Manohar Kuse <mpkuse@connect.ust.hk>
// Notes: http://wiki.ros.org/rviz/DisplayTypes/Marker

#include <iostream>
#include <string>


#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


using namespace std;
using namespace Eigen;


class RosMarkerUtils
{
public:
    ////////////////// INIT /////////////////////////


    /// Initialize a camera with a few lines. You need to set the `ns` and `id` before publishing.
    static void init_camera_marker( visualization_msgs::Marker& marker, float cam_size );
    static void init_text_marker( visualization_msgs::Marker &marker );
    static void init_line_strip_marker( visualization_msgs::Marker &marker );
    static void init_line_marker( visualization_msgs::Marker &marker );
    static void init_line_marker( visualization_msgs::Marker &marker, const Vector3d& p1, const Vector3d& p2 );
    static void init_points_marker( visualization_msgs::Marker &marker );

    //////////////// SET //////////////////////////
    static void setpose_to_marker( const Matrix4d& w_T_c, visualization_msgs::Marker& marker );
    static void setposition_to_marker( const Vector3d& w_t_c, visualization_msgs::Marker& marker );
    static void setposition_to_marker( const Vector4d& w_t_c, visualization_msgs::Marker& marker );
    static void setposition_to_marker( float x, float y, float z, visualization_msgs::Marker& marker );
    static void setcolor_to_marker( float r, float g, float b, visualization_msgs::Marker& marker  );
    static void setcolor_to_marker( float r, float g, float b, float a, visualization_msgs::Marker& marker  );

    //////////////// Add Points ////////////////////
    static void add_point_to_marker( float x, float y, float z, visualization_msgs::Marker& marker, bool clear_prev_points=true );
    static void add_point_to_marker( const Vector3d& X, visualization_msgs::Marker& marker, bool clear_prev_points=true );
    static void add_point_to_marker( const Vector4d& X, visualization_msgs::Marker& marker, bool clear_prev_points=true );
    static void add_points_to_marker( const MatrixXd& X, visualization_msgs::Marker& marker, bool clear_prev_points=true ); //X : 3xN or 4xN.

};
