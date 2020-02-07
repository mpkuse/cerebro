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

    // Casts the edge alignment problem in opt variables imu_yaw, imu_tx, imu_ty, imu_tz.
    // Params:
    //      initial_guess____ref_T_curr: takes as input (initial guess) ref_T_curr (for the cameras)
    //      imu_T_cam                  : pose of camera as observed by imu frame (usually you get this from VINS system)
    //      vio_w_T_a, vio_w_T_b       : They are the pose of cameras as given out by the vio system.
    //                                   These are used to get initial estimates of pitch and rolls.
    //      optimized__ref_T_curr [out]: relative poses of curr in ref frame. These are camera poses
    //
    bool solve4DOF( const Matrix4d& initial_guess____ref_T_curr,
        const Matrix4d& imu_T_cam,const Matrix4d& vio_w_T_ref, const Matrix4d& vio_w_T_curr,
        Matrix4d& optimized__ref_T_curr  );



#if 0
    // Casts the edge alignment problem in opt variables imu_yaw, imu_tx, imu_ty, imu_tz.
    // Params:
    //      initial_guess____ref_T_curr: takes as input (initial guess) ref_T_curr (for the cameras)
    //      imu_T_cam                  : pose of camera as observed by imu frame (usually you get this from VINS system)
    //      vio_w_T_a, vio_w_T_b       : They are the pose of cameras as given out by the vio system.
    //                                   These are used to get initial estimates of pitch and rolls.
    //      optimized__ref_T_curr [out]: relative poses of curr in ref frame. These are camera poses
    //
    bool solve4DOF_with_point_feature_matching( const Matrix4d& initial_guess____ref_T_curr,
        const MatrixXd& uv_ref, const MatrixXd _3dpts_curr,
        const Matrix4d& imu_T_cam,const Matrix4d& vio_w_T_ref, const Matrix4d& vio_w_T_curr,
        Matrix4d& optimized__ref_T_curr  );
        #endif

    void set_make_representation_image()   {  make_representation_image = true;  }
    void unset_make_representation_image() {  make_representation_image = false; }
    const cv::Mat get_representation_image() {
        if( is_representation_image_ready == false ) {
            cout << "[EdgeAlignment::get_representation_image] THROW You are requesting to the representation image. It was not created. To create it do set_make_representation_image() before you call the solve()\n";
            throw "ERROR";
        }
        return representation_image;

    }

    // save the things needed to recreate this to disk.
    //  will save im_ref, im_curr, depth_curr and the initial_guess____ref_T_curr
    bool save_to_disk( const string PREFIX, const Matrix4d& initial_guess____ref_T_curr ) const;


    const ceres::Solver::Summary getCeresSummary() const { return out_summary; }
private:
    ceres::Solver::Summary out_summary;



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
    //          out_edgemap: same dimension as im but CV_8UC1 mask 255 at edges 0 at other points
    //          im_curr_uv : selected points as 3xN. Essentially uv points where cX (the returned params) was made
    MatrixXd get_cX_at_edge_pts( const cv::Mat im, const cv::Mat depth_map,
        cv::Mat& out_edgemap, cv::Mat& out_selected_pts_edgemap  );


    // Reprojects the 3D points a_X (in frame-of-ref of imA) using the transformation b_T_a (pose of a in frame-of-ref of b).
    // b_u are the 2d points. uses the class-global `cam` in this.
    Eigen::MatrixXd reproject( const Eigen::MatrixXd& a_X, const Eigen::Matrix4d& b_T_a );






};




class EAResidue {
public:
    /*
    EAResidue(
        const double fx, const double fy, const double cx, const double cy,
        const Eigen::Vector4d& __a_X,
        const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>>& __interpolated_a
    ): fx(fx), fy(fy), cx(cx), cy(cy),  a_X(__a_X), interp_a(__interpolated_a)
    {
        // cout << "---\n";
        // cout << "EAResidue.a_X: "<< a_X << endl;
        // cout << "fx=" << fx << "fy=" << fy << "cx=" << cx << "cy=" << cy << endl;

    }*/

    EAResidue(
        const double& fx, const double& fy, const double& cx, const double& cy,
        // const double a_Xx, const double a_Xy, const double a_Xz,
        // const Eigen::Vector4d& __a_X,
        const Eigen::Ref<const VectorXd>& __a_X,
        const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>>& __interpolated_a
    ):
        fx(fx), fy(fy), cx(cx), cy(cy),
        // a_Xx(a_Xx),a_Xy(a_Xy),a_Xz(a_Xz),
        a_X(__a_X),
        interp_a(__interpolated_a)
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
        // cout << "{{{{{{{{}}}}}}}}" << a_Xx << ","<< a_Xy << ","<< a_Xz << "," << endl;
        // templaye_a_X(0) = T(a_Xx);
        // templaye_a_X(1) = T(a_Xy);
        // templaye_a_X(2) = T(a_Xz);
        // templaye_a_X(3) = T(1.0);
        templaye_a_X = a_X.cast<T>();
        b_X = b_T_a *templaye_a_X;


        // Perspective-Projection and scaling with K.
        // if( b_X(2) < T(0.001) && b_X(2) > T(-0.001) ) {
        //     residue[0] = T(1.0);
        //     return true;
        // }
        // if( b_X(2) < T(0.0) ) {
        //     residue[0] = T(1.0);
        //     return true;
        // }

        T _u = T(fx) * b_X(0)/b_X(2) + T(cx);
        T _v = T(fy) * b_X(1)/b_X(2) + T(cy);

        // NEED TO DISCUSS THIS ISSUE WITH Sameer Agarwal (dev of ceres-solver Google)
        // if( _u < T(0) || _u > T(640) || _v < T(0) || _v > T(480) ) {
        //     residue[0] = T(.1);
        //     return true;
        // }

        interp_a.Evaluate( _u, _v, &residue[0] );  //original

        #if 0
        // switch constraint - if u enable this besure to also use `<EAResidue,2,4,3>` instead of <EAResidue,1,4,3> om create()
        T lambda = T(.5);
        T delta = residue[0] * residue[0];
        T s = lambda / ( T(1.0) + delta );
        residue[0] *= s;
        residue[1] = lambda * ( T(1.0) - s );
        #endif


        return true;
    }

    static ceres::CostFunction* Create(
        const double& fx, const double& fy, const double& cx, const double& cy,
        //const Eigen::Vector4d __a_X,
        const Eigen::Ref<const VectorXd>& __a_X,
        // const double a_Xx, const double a_Xy, const double a_Xz,
        const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>>& __interpolated_a  )
    {
        return( new ceres::AutoDiffCostFunction<EAResidue,1,4,3>
            (
                // new EAResidue( fx, fy, cx, cy, a_Xx,a_Xy,a_Xz,__interpolated_a)
                new EAResidue( fx, fy, cx, cy,
                    // a_Xx,a_Xy,a_Xz,
                    __a_X,
                    __interpolated_a)
            )
            );
    }

private:
    const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>>& interp_a; //
    // const Eigen::Matrix3d& K;
    const double& fx, fy, cx, cy;
    // double a_Xx, a_Xy, a_Xz;
    // const Eigen::Vector4d a_X; // a_X
    const Eigen::Ref<const VectorXd>& a_X;

};

// templated helpers
template <typename T>
Eigen::Matrix<T,3,3> ypr2R( const T& yaw_deg, const T& pitch_deg, const T& roll_deg )
{
  T y = yaw_deg   / T(180.0) * T(M_PI);
  T p = pitch_deg / T(180.0) * T(M_PI);
  T r = roll_deg  / T(180.0) * T(M_PI);


  // Eigen::Matrix<double, 3, 3> Rz;
  Eigen::Matrix<T,3,3> Rz;
  Rz << cos(y), -sin(y), T(0.0),
      sin(y), cos(y), T(0.0),
      T(0.0), T(0.0), T(1.0);

  // Eigen::Matrix<double, 3, 3> Ry;
  Eigen::Matrix<T,3,3> Ry;
  Ry << cos(p), T(0.), sin(p),
          T(0.), T(1.), T(0.),
          -sin(p), T(0.), cos(p);


  // Eigen::Matrix<double, 3, 3> Rx;
  Eigen::Matrix<T,3,3> Rx;
  Rx << T(1.), T(0.), T(0.),
      T(0.), cos(r), -sin(r),
      T(0.), sin(r), cos(r);

  return Rz * Ry * Rx;

};



template <typename T>
void rawyprt_to_eigenmat(
    const T& yaw_deg, const T& pitch_deg, const T& roll_deg,
    const T& tx, const T& ty, const T& tz,
    Eigen::Matrix<T,4,4>& dstT )
{
    dstT = Eigen::Matrix<T,4,4>::Identity();

    // ypr2R
    auto tmp = ypr2R( yaw_deg, pitch_deg, roll_deg );
    dstT.topLeftCorner(3,3) = tmp;

    dstT(0,3) = tx;
    dstT(1,3) = ty;
    dstT(2,3) = tz;
};

template <typename T>
T NormalizeAngle(const T& angle_degrees) {
  if (angle_degrees > T(180.0))
  	return angle_degrees - T(360.0);
  else if (angle_degrees < T(-180.0))
  	return angle_degrees + T(360.0);
  else
  	return angle_degrees;
};

class AngleLocalParameterization {
 public:

  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};

// done with helpers

class EA4DOFResidue
{
public:
    EA4DOFResidue(
        const double& fx, const double& fy, const double& cx, const double& cy, //<--- Camera params
        const double _Xx, const double _Xy, const double _Xz, //<--- 3d pts from curr in curr's ref
        const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>>& __interpolated_a,
        const double& _refimu_pitch_currimu, const double& _refimu_roll_currimu,
        const Matrix4d& __imu_T_cam, double ___weight
    ): fx(fx), fy(fy), cx(cx), cy(cy),
       c_Xx(_Xx),c_Xy(_Xy),c_Xz(_Xz),
       interp_a(__interpolated_a),
       refimu_pitch_currimu(_refimu_pitch_currimu ), refimu_roll_currimu(_refimu_roll_currimu),
       imu_T_cam(__imu_T_cam), imu_T_cam_inverse( __imu_T_cam.inverse() ),
       weight( ___weight )
    {}




    template <typename T>
    bool operator()( const T* const refimu_yaw_currimu, const T* const t, T* residue ) const
    {
        // use refimu_T_currimu <-- t refimu_yaw_currimu, this->refimu_pitch_currimu, this->refimu_roll_currimu,
        Eigen::Matrix<T,4,4> refimu_T_currimu;
        rawyprt_to_eigenmat(
            refimu_yaw_currimu[0], T(refimu_pitch_currimu), T(refimu_roll_currimu),
            t[0], t[1], t[2],
            refimu_T_currimu);

        // get ref_T_curr <-- imu_T_cam.inv * refimu_T_currimu * imu_T_cam
        Eigen::Matrix<T,4,4> ref_T_curr;
        // ref_T_curr = imu_T_cam.inverse().cast<T>() * refimu_T_currimu * imu_T_cam.cast<T>();
        ref_T_curr = imu_T_cam_inverse.cast<T>() * refimu_T_currimu * imu_T_cam.cast<T>();

        // transform cX
        Eigen::Matrix<T,4,1> c_X, ref_X;
        c_X << T(c_Xx), T(c_Xy), T(c_Xz), T(1.0);
        ref_X = ref_T_curr * c_X;


        // Perspective-Projection and scaling with K.
        // if( ref_X(2) < T(0.001) && ref_X(2) > T(-0.001) )
            // return false;
        // if( b_X(2) < T(0.0) )
            // return false;
        T _u = T(fx) * ref_X(0)/ref_X(2) + T(cx);
        T _v = T(fy) * ref_X(1)/ref_X(2) + T(cy);

        if( _u < T(0) || _u > T(640) || _v < T(0) || _v > T(480) ) {
            residue[0] = T(0.);
            return true;
        }

        interp_a.Evaluate( _u, _v, &residue[0] );  //original



        return true;
    }

    static ceres::CostFunction* Create(
        const double& fx, const double& fy, const double& cx, const double& cy,
        //const Eigen::Vector4d __a_X,
        const double a_Xx, const double a_Xy, const double a_Xz,
        //const Eigen::Ref<const VectorXd>& __a_X,
        const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>>& __interpolated_a ,
        const double& _refimu_pitch_currimu, const double& _refimu_roll_currimu,
        const Matrix4d& imu_T_cam, const double weight )
    {
        return( new ceres::AutoDiffCostFunction<EA4DOFResidue,1,1,3>
            (
                new EA4DOFResidue( fx, fy, cx, cy, a_Xx,a_Xy,a_Xz,__interpolated_a,
                    _refimu_pitch_currimu, _refimu_roll_currimu, imu_T_cam, weight)
            )
            );
    }



    private:
        const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>>& interp_a; //
        // const Eigen::Matrix3d& K;
        const double& fx, fy, cx, cy;
        const double c_Xx, c_Xy, c_Xz;
        const double weight;

        const Matrix4d& imu_T_cam;
        const Matrix4d& imu_T_cam_inverse;
        const double& refimu_pitch_currimu, refimu_roll_currimu;



};
