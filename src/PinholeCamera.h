#pragma once
/** Class to handle camera intrinsics

      Author  : Manohar Kuse <mpkuse@connect.ust.hk>
      Created : 3rd Oct, 2017
      Modified: 26th Oct, 2018
*/


#include <iostream>
#include <string>
#include <fstream>


//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

//ros
#include <ros/ros.h>
#include <ros/package.h>


// Eigen3
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;



class PinholeCamera {

public:
  PinholeCamera() { mValid = false; }
  PinholeCamera( string config_file );

  void printCameraInfo( int verbosity=2 ) const;

  const cv::Mat get_mK() const {return m_K; }
  const cv::Mat get_mD() const {return m_D; }

  const Matrix3d get_eK() const {return e_K; }
  const Vector4d get_eD() const {return e_D; }

  // have a const after function, signals the compiler that these functions will not attempt to modify the variables inside
  bool isValid() const  { return mValid; }
  double fx() const  { return _fx; }
  double fy() const  { return _fy; }
  double cx() const  { return _cx; }
  double cy() const  { return _cy; }
  double k1() const  { return _k1; }
  double k2() const  { return _k2; }
  double p1() const  { return _p1; }
  double p2() const  { return _p2; }

  string getModelType() const  { return config_model_type; }
  string getCameraName() const  { return config_camera_name; }
  string getConfigFileName() const  { return config_file_name; }
  int getImageWidth() const  { return config_image_width; }
  int getImageHeight() const  { return config_image_height; }

  int getImageRows() const  { return this->getImageHeight(); }
  int getImageCols() const  { return this->getImageWidth(); }


  // Projection
  // Input 3d points in homogeneous co-ordinates 4xN matrix. Eigen I/O.
  // uses the camera matrix and D from this class 
  void perspectiveProject3DPoints( const MatrixXd& c_X, MatrixXd& out_pts );

private:
  string config_model_type, config_camera_name;
  string config_file_name;
  int config_image_width, config_image_height;

  double _fx, _fy, _cx, _cy;
  double _k1, _k2, _p1, _p2;

  bool mValid;

  void print_cvmat_info( string msg, const cv::Mat& A );
  string type2str(int type);

  cv::Mat m_K; //3x3
  cv::Mat m_D; //4x1

  Matrix3d e_K;
  Vector4d e_D;


};
