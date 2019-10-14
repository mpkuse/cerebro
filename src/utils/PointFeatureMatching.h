#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>
#include <memory> //for std::shared_ptr


//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;

#include "GMSMatcher/gms_matcher.h"
#include "ElapsedTime.h"
#include "MiscUtils.h"
#include "TermColor.h"
#include "CameraGeometry.h"



// this will hold numbers to indicate the performance. Specific for each type of matcher
// TODO: ideal for inheritance structure
class PointFeatureMatchingSummary
{
public:
    string feature_detector_type; // ORB, FAST, SURF etc
    string feature_descriptor_type; // ORB, SURF
    string matcher_type; // BFMatcher, FLANN
    int n_keypoints; // number of detected keypoints
    int n_descriptor_dimension;  // the dimension of the descriptor

    int n_keypoints_pass_ratio_test; // number of points that pass the ratio test
    int n_keypoints_pass_f_test; //number of points that pass the fundamental matrix test
    int n_total_usable_features; // after eliminating features which dont pass the ratio test and the f-test, how many remain?

    // 0 : no printing
    // 1 : minimal
    // 2 : more
    // 5 : elaborate
    void prettyPrint( int debug_level )
    {
        if( debug_level <= 0)
            return ;

        cout << TermColor::GREEN() << "PointFeatureMatchingSummary\n";
        if( debug_level > 2 ) {
        cout << "\tfeature_detector_type: " << feature_detector_type << "\t";
        cout << "\tfeature_descriptor_type: " << feature_descriptor_type << "\t";
        cout << "\tmatcher_type: " << matcher_type << "\t";
        cout << "\tn_descriptor_dimension: " << n_descriptor_dimension << "\t";
        cout << endl;
        }
        cout << "\tn_keypoints: " << n_keypoints << "\t";
        cout << "\tn_keypoints_pass_ratio_test: " << n_keypoints_pass_ratio_test << "\t";
        cout << "\tn_keypoints_pass_f_test: " << n_keypoints_pass_f_test << "\t";
        if( debug_level > 2 ) {
        cout << "\tn_total_usable_features: " << n_total_usable_features << "\t";}
        cout << TermColor::RESET() << endl;
    }
};


class StaticPointFeatureMatching
{
public:

    static void gms_point_feature_matches( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                                MatrixXd& u, MatrixXd& ud, int n_orb_feat=5000 ); //< n_orb_feat has to be a few thousands atleast for spatial consistency checks.


    // u : 3xN. (x,y) or (colID,rowID)
    static void point_feature_matches( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                    MatrixXd&u, MatrixXd& ud,
                PointFeatureMatchingSummary& summary  );

    // Given the point feature matches and the 3d image (from disparity map) will return
    // the valid world points and corresponding points.
    // [Input]
    //      uv: 2xN matrix of point-feature in image-a. In image co-ordinates (not normalized image cords)
    //      _3dImage_uv : 3d image from disparity map of image-a. sizeof( _3dImage_uv) === WxHx3
    //      uv_d: 2xN matrix of point-feature in image-b. Note that uv<-->uv_d are correspondences so should of equal sizes
    // [Output]
    //      feature_position_uv : a subset of uv but normalized_image_cordinates
    //      feature_position_uv_d : a subset of uv_d. results in normalized_image_cordinates
    //      world_point : 3d points of uv.
    // [Note]
    //      feature_position_uv \subset uv. Selects points which have valid depths.
    //      size of output is same for all 3
    //      world points are of uv and in co-ordinate system of camera center of uv (or image-a).
    static bool make_3d_2d_collection__using__pfmatches_and_disparity( std::shared_ptr<StereoGeometry> stereogeom,
                const MatrixXd& uv, const cv::Mat& _3dImage_uv,     const MatrixXd& uv_d,
                                std::vector<Eigen::Vector2d>& feature_position_uv, std::vector<Eigen::Vector2d>& feature_position_uv_d,
                                std::vector<Eigen::Vector3d>& world_point );


    // given pf-matches uv<-->ud_d and their _3dImages. returns the 3d point correspondences at points where it is valid
    // uv_X: the 3d points are in frame of ref of camera-uv
    // uvd_Y: these 3d points are in frame of ref of camera-uvd
    static bool make_3d_3d_collection__using__pfmatches_and_disparity(
        const MatrixXd& uv, const cv::Mat& _3dImage_uv,
        const MatrixXd& uv_d, const cv::Mat& _3dImage_uv_d,
        vector<Vector3d>& uv_X, vector<Vector3d>& uvd_Y
    );



    // given pf-matches uv<-->ud_d and their depth images and camera, returns the 3d point correspondences at points where it is valid
    // uv_X: the 3d points are in frame of ref of camera-uv
    // uvd_Y: these 3d points are in frame of ref of camera-uvd
    // depth_image_uv, depth_image_uvd: assumes either CV_16uc1 or CV_32FC1, depth values expressed in millimeters.
    static bool make_3d_3d_collection__using__pfmatches_and_depthimage(
        camodocal::CameraPtr _camera,
        const MatrixXd& uv, const cv::Mat& depth_image_uv,
        const MatrixXd& uv_d, const cv::Mat& depth_image_uvd,
        // vector<Vector3d>& uv_X, vector<Vector3d>& uvd_Y
        MatrixXd& uv_X, MatrixXd& uvd_Y, vector<bool>& valids
    );



    static bool image_correspondence_with_depth(
        const cv::Mat& im_a, const cv::Mat& depth_a,
        const cv::Mat& im_b, const cv::Mat& depth_b,
        MatrixXd& uv_a, MatrixXd& uv_b,
        MatrixXd& aX, MatrixXd& bX, vector<bool>& valids
    );



private:
        // Give a set of knn=2 raw matches, eliminates matches based on Lowe's Ratio test.
    // Also returns the point features set.
    // Cite: Lowe, David G. "Distinctive image features from scale-invariant keypoints." International journal of computer vision 60.2 (2004): 91-110.
        static void lowe_ratio_test( const vector<cv::KeyPoint>& keypoints1, const vector<cv::KeyPoint>& keypoints2 ,
                          const std::vector< std::vector< cv::DMatch > >& matches_raw,
                          vector<cv::Point2f>& pts_1, vector<cv::Point2f>& pts_2, float threshold=0.85 );

};
