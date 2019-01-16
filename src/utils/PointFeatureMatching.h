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


class StaticPointFeatureMatching
{
public:

    static void gms_point_feature_matches( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                                MatrixXd& u, MatrixXd& ud, int n_orb_feat=5000 ); //< n_orb_feat has to be a few thousands atleast for spatial consistency checks.


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

};
