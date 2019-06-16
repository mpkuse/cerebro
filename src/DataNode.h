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
#include "utils/TermColor.h"
class DataNode
{
public:
    DataNode( ros::Time stamp ): stamp(stamp)
    {
        is_key_frame = false;
        // m_image=false; 
        m_wTc=false;
    }

    #if 0
    void setImageFromMsg( const sensor_msgs::ImageConstPtr msg ); ///< this sets the primary image
    void setImageFromMsg( const sensor_msgs::ImageConstPtr msg, short cam_id ); ///< this sets additional image. multiple additional images by Node is possible by using ids
    #endif

    void setPoseFromMsg( const nav_msgs::OdometryConstPtr msg );

    void setPointCloudFromMsg( const sensor_msgs::PointCloudConstPtr msg ); // uses msg->points[ \forall i].x, .y, .z
    void setUnVnFromMsg( const sensor_msgs::PointCloudConstPtr msg );
    void setUVFromMsg( const sensor_msgs::PointCloudConstPtr msg );
    void setTrackedFeatIdsFromMsg( const sensor_msgs::PointCloudConstPtr msg );

    void setAsKeyFrame() { is_key_frame = true; }
    void unsetAsKeyFrame() { is_key_frame = false; }

    // This is intended to indicate the number of tracked features from /feature_tracker.
    // TODO: In the future if need be implement to save the tracked points. For now I am not retaining those
    void setNumberOfSuccessfullyTrackedFeatures( int n );
    int getNumberOfSuccessfullyTrackedFeatures() const;


    bool isKeyFrame()  const{ return (bool)is_key_frame; }
    #if 0
    bool isImageAvailable() const { return m_image; }
    bool isImageAvailable(short cam_id) const { return ((t_all_images.count(cam_id)>0)?true:false); }
    #endif
    bool isPoseAvailable() const { return m_wTc; }
    bool isPtCldAvailable() const { return m_ptcld; }
    bool isUnVnAvailable()  const{ return m_unvn; }
    bool isUVAvailable() const { return m_uv; }
    bool isFeatIdsAvailable() const { return m_tracked_feat_ids; }

    #if 0
    const cv::Mat& getImage() const; //< this will give out the default image.
    const cv::Mat& getImage(short cam_id) const ; // this will give out the image of the cam_id. cam_id=0 will have issues. if you want the default camera image do not pass any argument, which will result in the above call.
    #endif
    const Matrix4d& getPose() const ;
    const MatrixXd& getPoseCovariance() const ; //6x6 matrix
    const MatrixXd& getPointCloud() const ; // returns a 4xN matrix
    const MatrixXd& getUnVn() const ; // returns a 3xN matrix
    const MatrixXd& getUV() const ; // returns a 3xN matrix
    const VectorXi& getFeatIds() const; // return a N-vector
    int nPts() const;

    const ros::Time getT() const;
    #if 0
    const ros::Time getT_image() const;
    const ros::Time getT_image(short cam_id) const;
    #endif
    const ros::Time getT_pose() const;
    const ros::Time getT_ptcld() const;
    const ros::Time getT_unvn() const ;
    const ros::Time getT_uv() const ;

    // Whole Image descriptors setter and getter
    void setWholeImageDescriptor( VectorXd vec );
    // const VectorXd& getWholeImageDescriptor();
    const VectorXd getWholeImageDescriptor() const; //don't know which one is more suitated to me. :(
    bool isWholeImageDescriptorAvailable() const { return m_img_desc; }


    void prettyPrint();
    #if 0
    void deallocate_all_images();
    void print_image_cvinfo();
    #endif


private:
    const ros::Time stamp;
    mutable std::mutex m;

    // bool is_key_frame = false;
    std::atomic<bool> is_key_frame;

    #if 0
    // Raw Image
    cv::Mat image;
    ros::Time t_image;
    // bool m_image=false; // TODO better make this atomic<bool>
    std::atomic<bool> m_image;

    // Additional Raw Images
    std::map<short,cv::Mat> all_images;
    std::map<short,ros::Time> t_all_images;
    #endif


    // Pose (odometry pose from vins estimator)
    Matrix4d wTc;
    MatrixXd wTc_covariance; // 6x6
    ros::Time t_wTc;
    // bool m_wTc=false;
    std::atomic<bool> m_wTc;


    // point cloud (3d data)
    MatrixXd ptcld;
    ros::Time t_ptcld;
    std::atomic<bool> m_ptcld;
    std::atomic<bool> m_ptcld_zero_pts;


    // unvn - imaged points in normalized cords
    MatrixXd unvn;
    ros::Time t_unvn;
    std::atomic<bool> m_unvn;
    std::atomic<bool> m_unvn_zero_pts;


    // uv - imaged points in observed cords.
    MatrixXd uv;
    ros::Time t_uv;
    std::atomic<bool> m_uv;
    std::atomic<bool> m_uv_zero_pts;


    // tracked feat ids
    VectorXi tracked_feat_ids;
    ros::Time t_tracked_feat_ids;
    std::atomic<bool> m_tracked_feat_ids;
    std::atomic<bool> m_tracked_feat_ids_zero_pts;

    int numberOfSuccessfullyTrackedFeatures = -1;

    // Whole Image Descriptor
    VectorXd img_desc;
    bool m_img_desc = false;

};
