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
    static std::vector<std::string>
    split( std::string const& original, char separator );

    static void plot_point_sets( const cv::Mat& im, const MatrixXd& pts_set, cv::Mat& dst,
                                            const cv::Scalar& color, bool enable_keypoint_annotation=true,const string& msg=string("") );
    static void plot_point_sets( const cv::Mat& im, const cv::Mat& pts_set, cv::Mat& dst,
                                            const cv::Scalar& color, bool enable_keypoint_annotation=true, const string& msg=string("") );

};
