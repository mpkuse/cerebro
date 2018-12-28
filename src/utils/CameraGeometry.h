#pragma once
// This has 2 classes
// A) MonoGeometry
// B) StereoGeometry

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>


#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

#include "MiscUtils.h"
#include "ElapsedTime.h"
#include "PoseManipUtils.h"
#include "TermColor.h"

// Threads and threadsafety
#include <thread>
#include <mutex>
#include <atomic>

class MonoGeometry {
public:
    MonoGeometry() {  }

    MonoGeometry(camodocal::CameraPtr _camera );

    // Set the new intrinsic matrix (optional). By default will use the same as
    // the abstract camera. but the K matrix is essentially just scaling pixel sizes.
    // this comes to play in stereopairs and/or motion stereo.
    void set_K( Matrix3d K );

    // this is just a call to cv::remap with the maps already computed.
    void do_image_undistortion( const cv::Mat& im_raw, cv::Mat & im_undistorted );

private:
    camodocal::CameraPtr camera;
    Matrix3d K_new;
    float new_fx, new_fy, new_cx, new_cy;

    cv::Mat map_x, map_y;

    std::mutex m;

};


// There are 3 types of images
//      im*_raw
//      im*_undistorted
//      im*_stereo_rectified
class StereoGeometry {
public:
    // right_T_left: extrinsic calibration for stereo pair. The position of left camera as viewed from right camera.
    StereoGeometry( camodocal::CameraPtr _left_camera,
                    camodocal::CameraPtr _right_camera,
                    Matrix4d right_T_left
                );


    // intrinsic cam matrix for both cameras. Here it is important to set this
    // else the block matching computation won't be valid. If this is not set
    // we will use the matrix for left camera as K_new.
    // Everytime a new K is set, we have to recompute the maps.
    void set_K( Matrix3d K );
    void set_K( float _fx, float _fy, float _cx, float _cy );
    const Matrix3d& get_K();

    // set extrinsic. THis is thread safe
    // Everytime a new extrinsic is set, we have to recompute the maps.
    void set_stereoextrinsic( Matrix4d __right_T_left );
    void set_stereoextrinsic( Vector4d quat_xyzw, Vector3d tr_xyz );
    const Matrix4d& get_stereoextrinsic();

    void fundamentalmatrix_from_stereoextrinsic( Matrix3d& F );
    //TODO write another function to return fundamental matrix of rectified stereo pair.

    // Draw epipolar lines on both views. These images will be overwritten. You need
    // to give me undistorted image-pairs. You can undistory image pair with
    // StereoGeometry::do_image_undistortion()
    void draw_epipolarlines( cv::Mat& imleft_undistorted, cv::Mat& imright_undistorted );
    void draw_srectified_epipolarlines( cv::Mat& imleft_srectified, cv::Mat& imright_srectified );


    void do_image_undistortion( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                                cv::Mat& imleft_undistorted, cv::Mat& imright_undistorted
                            );

    // given the undistorted images outputs the stereo-rectified pairs
    // This is essentially just cv::remap using the stereo-rectification maps
    void do_stereo_rectification_of_undistorted_images(
        const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
        cv::Mat& imleft_srectified, cv::Mat& imright_srectified );


    // Given raw image pair return stereo-rectified pair. This is a 2 step process
    // raw --> undistorted --> stereo rectified
    void do_stereo_rectification_of_raw_images(
        const cv::Mat imleft_raw, const cv::Mat imright_raw,
        cv::Mat& imleft_srectified, cv::Mat& imright_srectified );


    // stereo rectified --> disparity. Uses the stereo-block matching algorithm,
    // ie. cv::StereoBM
    // if you use this function, besure to call `do_stereo_rectification_of_undistorted_images`
    // or `do_stereo_rectification_of_raw_images` on your images before calling this function.
    // returned disparity is CV_16SC1. Be cautioned.
    void do_stereoblockmatching_of_srectified_images(
        const cv::Mat& imleft_srectified, const cv::Mat& imright_srectified,
        cv::Mat& disparity
    );

    // raw--> disparity
    // internally does:
    //      step-1: raw-->undistorted
    //      step-2: undistorted--> srectify (stereo rectify)
    //      step-3: srectify --> disparity
    void do_stereoblockmatching_of_raw_images(
        const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
        cv::Mat& disparity
    );


    // undistorted --> disparity
    //      step-1: undistorted --> srectified
    //      step-2 : srectified --> disparity
    void do_stereoblockmatching_of_undistorted_images(
        const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
        cv::Mat& disparity
    );

    // # Inputs
    //  disparity_raw : The disparity image
    //  fill_eigen_matrix : _3dpts will be allocated and filled only if this flag is true. If this flag is false then eigen matrix is not filled in this function.
    //  make_homogeneous : true will result in Eigen matrix being 4xN else will be 3xN. fill_eigen_matrix need to be true for this to matter.
    // # Outputs
    //  out3d : 3 channel image. X,Y,Z --> ch0, ch1, ch2. Will also contain invalid 3d points.
    //  _3dpts : 4xN matrix containing only valid points. N < disparity_raw.shape[0]*disparity_raw.shape[1].
    void disparity_to_3DPoints(const cv::Mat& disparity_raw,
        cv::Mat& out3D, MatrixXd& _3dpts,
        bool fill_eigen_matrix=true, bool make_homogeneous=true );


    // #Input
    // imleft_raw, imright_raw : Raw images. As is from the camera.
    // # Output
    // _3dpts : 4xN
    //      1. raw --> srectified
    //      2. srectified --> disparity_raw
    //      3. disparity_raw --> 3d points
    void get3dpoints_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                                MatrixXd& _3dpts    );




    // #Input
    // imleft_raw, imright_raw : Raw images. As is from the camera.
    // # Output
    // _3dpts : 4xN
    //      1. raw --> srectified
    //      2. srectified --> disparity_raw
    //      3. disparity_raw --> 3d points
    // _3dImage: 3d points as 3 channel image. 1st channel is X, 2nd channel is Y and 3rd channel is Z.
    void get3dpoints_and_3dmap_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                                MatrixXd& _3dpts, cv::Mat& _3dImage    );


    // returns both 3dpoints and 3dimage along with srectified image pair
    void get3dpoints_and_3dmap_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                                MatrixXd& _3dpts, cv::Mat& _3dImage,
                            cv::Mat& imleft_srectified, cv::Mat& imright_srectified     );


    // #Input
    // imleft_raw, imright_raw : Raw images. As is from the camera.
    // # Output
    // _3dImage: 3d points as 3 channel image. 1st channel is X, 2nd channel is Y and 3rd channel is Z.
    // Also note that these co-ordinates and imleft_raw co-ordinate do not correspond. They correspond to
    // the srectified images. Incase you want to use the color info with these 3d points, this
    // will lead to wrong. You should compute the srectified images for that.
    //      1. raw --> srectified
    //      2. srectified --> disparity_raw
    //      3. disparity_raw --> 3d points
    void get3dmap_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                                cv::Mat& _3dImage );


    // #Input
    // imleft_raw, imright_raw : Raw images. As is from the camera.
    // # Output
    // e_3dImageX, e_3dImageY, e_3dImageZ: cv::split( _3dImage ). same size as imleft_raw.shape.
    // Also note that these co-ordinates and imleft_raw co-ordinate do not correspond. They correspond to
    // the srectified images. Incase you want to use the color info with these 3d points, this
    // will lead to wrong. You should compute the srectified images for that.
    //      1. raw --> srectified
    //      2. srectified --> disparity_raw
    //      3. disparity_raw --> 3d points
    void get3dmap_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                                MatrixXd& e_3dImageX, MatrixXd& e_3dImageY, MatrixXd& e_3dImageZ  );



    // Given raw image pair returns valid 3d points and disparity false color map.
    // #Input
    //      imleft_raw, imright_raw : Raw images. As is from the camera.
    // #Output
    //      _3dpts : 4xN matrix.
    //      disparity_for_visualization : false color mapped for visualization only. Don't do any processing with this one.
    void get3dpoints_and_disparity_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                                MatrixXd& _3dpts, cv::Mat& disparity_for_visualization    );

    // Given raw image pair return a) stereo-rectified image pair b) valid 3d points c) disparity false color map
    void get_srectifiedim_and_3dpoints_and_disparity_from_raw_images(
                        const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                        cv::Mat& imleft_srectified, cv::Mat& imright_srectified,
                        MatrixXd& _3dpts, cv::Mat& disparity_for_visualization    );
    void get_srectifiedim_and_3dpoints_and_3dmap_and_disparity_from_raw_images(
                        const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                        cv::Mat& imleft_srectified, cv::Mat& imright_srectified,
                        MatrixXd& _3dpts, cv::Mat& _3dImage, cv::Mat& disparity_for_visualization    );


    // Some  semi private getters of internal variables
public:
    const cv::Mat& get_Q() const { return rm_Q; }
    // TODO: (if need be) also have getters from rm_R1, rm_R2, rm_P1, rm_P2, rm_shift, rm_fundamental_matrix.

    void print_blockmatcher_algo_info();

private:
    camodocal::CameraPtr camera_left, camera_right;
    Matrix4d right_T_left; ///< extrinsic calibration of the stereopair.

    Matrix3d K_new;
    float new_fx, new_fy, new_cx, new_cy;

    std::shared_ptr<MonoGeometry> left_geom, right_geom;

    // stereo rectify maps
    // theory : http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO/tutorial.html

    std::mutex m_extrinsics;
    std::mutex m_intrinsics;

    // This function is called everytime you change the intrinsics and extrinsics for
    // the stereo pair. This outputs the stereo-rectification maps, ie. map1_x, map1_y, map2_x, map2_y
    // This is not intended to be a public call.
    // TODO: thread safety
    void make_stereo_rectification_maps();
    cv::Mat rm_R1, rm_R2, rm_P1, rm_P2, rm_Q;
    Vector3d rm_shift; // right_T_left
    Matrix3d rm_fundamental_matrix; // fundamental matrix after rectification
    cv::Mat map1_x, map1_y, map2_x, map2_y;


    // Stereo Block Matching
    cv::Ptr<cv::StereoBM> bm;
    cv::Ptr<cv::StereoSGBM> sgbm;

};

class GeometryUtils {
public:
    // Given the focal length make a K out of it
    static void make_K( float new_fx, float new_fy, float new_cx, float new_cy, Matrix3d& K );

    // For a camera gets a K
    static void getK( camodocal::CameraPtr m_cam, Matrix3d& K );

    // Ideal Projection:
    // a) c_X = c_X / c_X.row(2). ==> Z divide
    // b) perspective_proj = c_X.topRows(3)
    // c) uv = K * c_X
    // # Input
    // K : Camera params. K_new
    // c_X : 3D points expressed in frame of the camera.
    //          Note that if you have world 3d points you will need to transform
    //          it to camera co-ordinates before you pass it to this function.
    // uv : image co-ordinates (xy)
    static void idealProjection( const Matrix3d& K, const MatrixXd& c_X, MatrixXd& uv  );


    // given a point cloud as 3xN or 4xN matrix, gets colors for each based on the depth
    // min==-1 then we will compute the min from data, else will use whatever was supplied
    // max==-1 then we will compute the max from data, else will use whatever.
    static void depthColors( const MatrixXd& ptcld, vector<cv::Scalar>& out_colors, double min=-1, double max=-1 );
    static void depthColors( const MatrixXd& ptcld, MatrixXd& out_colors, double min=-1, double max=-1 ); // out_colors : 3xN rgb \in [0,1]

};
