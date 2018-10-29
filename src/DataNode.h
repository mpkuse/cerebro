#pragma once

/** This class holds data at 1 instance of time.

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 27th Oct, 2018

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
#include <ros/ros.h>
#include <ros/package.h>

// ros msg
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/Image.h>
// #include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

#include "utils/PoseManipUtils.h"
#include "utils/MiscUtils.h"

class DataNode
{
public:
    DataNode( ros::Time stamp ): stamp(stamp)  { is_key_frame = false; }

    void setImageFromMsg( const sensor_msgs::ImageConstPtr msg );
    void setPoseFromMsg( const nav_msgs::OdometryConstPtr msg );

    void setPointCloudFromMsg( const sensor_msgs::PointCloudConstPtr msg ); // uses msg->points[ \forall i].x, .y, .z
    void setUnVnFromMsg( const sensor_msgs::PointCloudConstPtr msg );
    void setUVFromMsg( const sensor_msgs::PointCloudConstPtr msg );
    void setTrackedFeatIdsFromMsg( const sensor_msgs::PointCloudConstPtr msg );

    void setAsKeyFrame() { is_key_frame = true; }
    void unsetAsKeyFrame() { is_key_frame = false; }

    // TODO : Make the getters and setters thread-safe.
    bool isKeyFrame() { return (bool)is_key_frame; }
    bool isImageAvailable() { return m_image; }
    bool isPoseAvailable() { return m_wTc; }
    bool isPtCldAvailable() { return m_ptcld; }
    bool isUnVnAvailable() { return m_unvn; }
    bool isUVAvailable() { return m_uv; }
    bool isFeatIdsAvailable() { return m_tracked_feat_ids; }


    const cv::Mat& getImage();
    const Matrix4d& getPose();
    const MatrixXd& getPoseCovariance(); //6x6 matrix
    const MatrixXd& getPointCloud(); // returns a 4xN matrix
    const MatrixXd& getUnVn(); // returns a 3xN matrix
    const MatrixXd& getUV(); // returns a 3xN matrix
    const VectorXi& getFeatIds(); // return a N-vector
    int nPts();
    const ros::Time getT();


    void prettyPrint();


private:
    const ros::Time stamp;
    std::mutex m;

    // bool is_key_frame = false;
    std::atomic<bool> is_key_frame;

    // Raw Image
    cv::Mat image;
    ros::Time t_image;
    bool m_image=false; // TODO better make this atomic<bool>


    // Pose (odometry pose from vins estimator)
    Matrix4d wTc;
    MatrixXd wTc_covariance; // 6x6
    ros::Time t_wTc;
    bool m_wTc=false;


    // point cloud (3d data)
    MatrixXd ptcld;
    ros::Time t_ptcld;
    bool m_ptcld=false;


    // unvn - imaged points in normalized cords
    MatrixXd unvn;
    ros::Time t_unvn;
    bool m_unvn=false;


    // uv - imaged points in observed cords.
    MatrixXd uv;
    ros::Time t_uv;
    bool m_uv=false;


    // tracked feat ids
    VectorXi tracked_feat_ids;
    ros::Time t_tracked_feat_ids;
    bool m_tracked_feat_ids = false;

};
