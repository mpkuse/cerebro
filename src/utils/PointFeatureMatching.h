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

#include "GMSMatcher/gms_matcher.h"
#include "ElapsedTime.h"
#include "MiscUtils.h"


class StaticPointFeatureMatching
{
public:

        static void gms_point_feature_matches( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                                    MatrixXd& u, MatrixXd& ud, int n_orb_feat=5000 ); //< n_orb_feat has to be a few thousands atleast for spatial consistency checks.

};
