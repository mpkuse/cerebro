#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>

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


class MiscUtils
{
public:
    static string type2str(int type);
    static string cvmat_info( const cv::Mat& mat );
    static std::vector<std::string>
    split( std::string const& original, char separator );


    // convert from opencv format of keypoints to Eigen
    static void keypoint_2_eigen( const std::vector<cv::KeyPoint>& kp, MatrixXd& uv, bool make_homogeneous=true );

    // given opencv keypoints and DMatch will produce M1, and M2 the co-ordinates
    static void dmatch_2_eigen( const std::vector<cv::KeyPoint>& kp1, const std::vector<cv::KeyPoint>& kp2,
                                const std::vector<cv::DMatch> matches,
                                MatrixXd& M1, MatrixXd& M2,
                                bool make_homogeneous=true
                            );


    // Eigen Interace:  PLotting functions with Eigen Interfaces
    static void plot_point_sets( const cv::Mat& im, const MatrixXd& pts_set, cv::Mat& dst,
                                            const cv::Scalar& color, bool enable_keypoint_annotation=true,const string& msg=string("") );

    // cv::Mat Interfaces: Plotting Functions with cv::Mat Interfaces.
    static void plot_point_sets( const cv::Mat& im, const cv::Mat& pts_set, cv::Mat& dst,
                                            const cv::Scalar& color, bool enable_keypoint_annotation=true, const string& msg=string("") );

    // Inplace plotting. Here dont need to specify a separate destination. src is modified.
    static void plot_point_sets( cv::Mat& im, const MatrixXd& pts_set,
                                            const cv::Scalar& color, bool enable_keypoint_annotation, const string& msg );

    // Plotting with annotations specified by VectorXi
    static void plot_point_sets( cv::Mat& im, const MatrixXd& pts_set, cv::Mat& dst,
                                            const cv::Scalar& color, const VectorXi& annotations, const string& msg );

    // Plotting with annotations specified by VectorXi inplace
    static void plot_point_sets( cv::Mat& im, const MatrixXd& pts_set,
                                            const cv::Scalar& color, const VectorXi& annotations, const string& msg );

    // Plots [ imA | imaB ] with points correspondences
    // [Input]
    //    imA, imB : Images
    //    ptsA, ptsB : 2xN or 3xN
    //    idxA, idxB : Index of each of the image. This will appear in status part. No other imppact of these.
    //    color_marker : color of the point marker
    //    color_line   : color of the line
    //    annotate_pts : true with putText for each point. False will not putText.
    // [Output]
    //    outImg : Output image
    static void plot_point_pair( const cv::Mat& imA, const MatrixXd& ptsA, int idxA,
                          const cv::Mat& imB, const MatrixXd& ptsB, int idxB,
                          cv::Mat& dst,
                          const cv::Scalar& color_marker,
                          const cv::Scalar& color_line=cv::Scalar(0,255,0),
                          bool annotate_pts=false,
                          const string& msg=string("N.A")
                         );


     // nearly same as the above, but will color every co-ordinate with different color
     // color_map_direction : 0 ==> // horizontal-gradiant
     //                       1 ==>  // vertical-gradiant
     //                       2 ==> // manhattan-gradiant
     //                       3 ==> // image centered manhattan-gradiant
     static void plot_point_pair( const cv::Mat& imA, const MatrixXd& ptsA, int idxA,
                           const cv::Mat& imB, const MatrixXd& ptsB, int idxB,
                           cv::Mat& dst,
                           short color_map_direction,
                           const string& msg=string("N.A")
                          );


};
