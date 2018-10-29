#pragma once
//
// This provides utilities for pose manipulation.
//
// Author : Manohar Kuse <mpkuse@connect.ust.hk>
//

#include <iostream>
#include <string>


#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>


using namespace std;
using namespace Eigen;

class PoseManipUtils
{
public:
    static void raw_to_eigenmat( const double * quat, const double * t, Matrix4d& dstT );
    static void eigenmat_to_raw( const Matrix4d& T, double * quat, double * t);
    static void rawyprt_to_eigenmat( const double * ypr, const double * t, Matrix4d& dstT );
    static void eigenmat_to_rawyprt( const Matrix4d& T, double * ypr, double * t);
    static Vector3d R2ypr( const Matrix3d& R);
    static Matrix3d ypr2R( const Vector3d& ypr);
    static void prettyprintPoseMatrix( const Matrix4d& M );
    static void prettyprintPoseMatrix( const Matrix4d& M, string& return_string );

    static string prettyprintMatrix4d( const Matrix4d& M );
    static string prettyprintMatrix4d_YPR( const Matrix4d& M );
    static string prettyprintMatrix4d_t( const Matrix4d& M );


    static void raw_xyzw_to_eigenmat( const double * quat, const double * t, Matrix4d& dstT );
    static void eigenmat_to_raw_xyzw( const Matrix4d& T, double * quat, double * t);

    static void geometry_msgs_Pose_to_eigenmat( const geometry_msgs::Pose& pose, Matrix4d& dstT );

};
