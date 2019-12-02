#pragma once


#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>
#include <memory> //for std::shared_ptr
#include <map>
using namespace std;

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Eigen3
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

// ceres
#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <ceres/loss_function.h>

// Camodocal
#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"

#include "utils/PoseManipUtils.h"
#include "utils/MiscUtils.h"
#include "utils/TermColor.h"
#include "utils/ElapsedTime.h"

class EdgeAlignment
{
public:
    EdgeAlignment(const camodocal::CameraPtr _cam, const cv::Mat _im_ref, const cv::Mat _im_curr, cv::Mat _depth_curr);

    bool solve( const Matrix4d& initial_guess____ref_T_curr, Matrix4d& optimized__ref_T_curr );

    void set_make_representation_image()   {  make_representation_image = true;  }
    void unset_make_representation_image() {  make_representation_image = false; }
    const cv::Mat get_representation_image() {
        if( is_representation_image_ready == false ) {
            cout << "[EdgeAlignment::get_representation_image] THROW You are requesting to the representation image. It was not created. To create it do set_make_representation_image() before you call the solve()\n";
            throw "ERROR";
        }
        return representation_image;

    }

private:
    const camodocal::CameraPtr cam ;
    const cv::Mat im_ref;               // distance transform will be made with edgemap of this image
    const cv::Mat im_curr, depth_curr; //3d points will be made from curr

    // Representation image
    bool make_representation_image = false;
    cv::Mat representation_image;
    bool is_representation_image_ready = false;


private:
    // helpers

    // Input an image, will output the distance transform of its edge map.
    // Steps in this a) Create Edge map b) threshold edgemap c) distance transform
    //      input : The input image, can be either rgb or gray. To compute edge map will be converted to gray
    //      distance transform[output] : same dimensions as input type=CV_32FC1
    void get_distance_transform( const cv::Mat& input, cv::Mat& out_distance_transform, cv::Mat& out_edge_map );


    // Given input image and depth map (either from RGBD or stereo), returns the 3D points at edges. Uses camera from the class
    // The output 3d points are in co-ordinate frame of the camera.
    MatrixXd get_cX_at_edge_pts( const cv::Mat im, const cv::Mat depth_map   );


    // Reprojects the 3D points a_X (in frame-of-ref of imA) using the transformation b_T_a (pose of a in frame-of-ref of b).
    // b_u are the 2d points. uses the class-global `cam` in this.
    Eigen::MatrixXd reproject( const Eigen::MatrixXd& a_X, const Eigen::Matrix4d& b_T_a );






};




class EAResidue {
public:
    EAResidue(
        const double fx, const double fy, const double cx, const double cy,
        const Eigen::Vector4d& __a_X,
        const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>>& __interpolated_a
    ): fx(fx), fy(fy), cx(cx), cy(cy),  a_X(__a_X), interp_a(__interpolated_a)
    {
        // cout << "---\n";
        // cout << "EAResidue.a_X: "<< a_X << endl;
        // cout << "fx=" << fx << "fy=" << fy << "cx=" << cx << "cy=" << cy << endl;

    }

    EAResidue(
        const double fx, const double fy, const double cx, const double cy,
        const double a_Xx, const double a_Xy, const double a_Xz,
        const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>>& __interpolated_a
    ): fx(fx), fy(fy), cx(cx), cy(cy),   a_Xx(a_Xx),a_Xy(a_Xy),a_Xz(a_Xz), interp_a(__interpolated_a)
    {}

    template <typename T>
    bool operator()( const T* const quat, const T* const t, T* residue ) const {
        // b_quat_a, b_t_a to b_T_a
        // Eigen::Quaternion<T> eigen_q( quat[0], quat[1], quat[2], quat[3] );
        Eigen::Quaternion<T> eigen_q( quat[3], quat[0], quat[1], quat[2] );
        Eigen::Matrix<T,4,4> b_T_a = Eigen::Matrix<T,4,4>::Zero();
        b_T_a.topLeftCorner(3,3) = eigen_q.toRotationMatrix();
        b_T_a(0,3) = t[0];
        b_T_a(1,3) = t[1];
        b_T_a(2,3) = t[2];
        b_T_a(3,3) =  T(1.0);



        // transform a_X
        Eigen::Matrix<T,4,1> b_X;
        Eigen::Matrix<T,4,1> templaye_a_X;
        // templaye_a_X << T(a_X(0)),T(a_X(1)),T(a_X(2)),T(1.0);
        // cout << "{{{{{{{{}}}}}}}}" << a_X << endl;
        // templaye_a_X(0) = T(a_X(0));
        // templaye_a_X(1) = T(a_X(1));
        // templaye_a_X(2) = T(2.0); //T(a_X(2));
        // templaye_a_X(3) = T(a_X(3));

        // cout << "{{{{{{{{}}}}}}}}" << a_Xx << ","<< a_Xy << ","<< a_Xz << "," << endl;
        templaye_a_X(0) = T(a_Xx);
        templaye_a_X(1) = T(a_Xy);
        templaye_a_X(2) = T(a_Xz);
        templaye_a_X(3) = T(1.0);
        b_X = b_T_a *templaye_a_X;


        // Perspective-Projection and scaling with K.
        if( b_X(2) < T(0.001) && b_X(2) > T(-0.001) )
            return false;
        T _u = T(fx) * b_X(0)/b_X(2) + T(cx);
        T _v = T(fy) * b_X(1)/b_X(2) + T(cy);

        // if( _u < T(0) || _u > T(640) || _v < T(0) || _v > T(240) )
            // return false;

        interp_a.Evaluate( _u, _v, &residue[0] );  //original

        #if 0
        // switch constraint
        T lambda = T(1.0);
        T delta = residue[0] * residue[0];
        T s = lambda / ( T(1.0) + delta );
        residue[0] *= s;
        residue[1] = lambda * ( T(1.0) - s );
        #endif


        return true;
    }

    static ceres::CostFunction* Create(
        const double fx, const double fy, const double cx, const double cy,
        //const Eigen::Vector4d __a_X,
        //const Eigen::Ref<const VectorXd>& __a_X,
        const double a_Xx, const double a_Xy, const double a_Xz,
        const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>>& __interpolated_a  )
    {
        return( new ceres::AutoDiffCostFunction<EAResidue,1,4,3>
            (
                new EAResidue( fx, fy, cx, cy, a_Xx,a_Xy,a_Xz,__interpolated_a)
            )
            );
    }

private:
    const Eigen::Vector4d a_X; // a_X
    const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>> interp_a; //
    // const Eigen::Matrix3d& K;
    double fx, fy, cx, cy;
    double a_Xx, a_Xy, a_Xz;

};
