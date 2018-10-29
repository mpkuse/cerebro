#include "PinholeCamera.h"


PinholeCamera::PinholeCamera( string config_file )
{

  cv::FileStorage fs( config_file, cv::FileStorage::READ );
  if( !fs.isOpened() )
  {
    ROS_ERROR( "Cannot open config file : %s", config_file.c_str() );
    ROS_ERROR( "Quit");
    mValid = false;
    exit(1);
  }
  this->config_file_name = string(config_file);

  cout << "---Camera Config---\n";
  fs["model_type"] >> config_model_type;     cout << "config_model_type:"<< config_model_type << endl;
  fs["camera_name"] >> config_camera_name;   cout << "config_camera_name:" << config_camera_name << endl;
  fs["image_width"] >> config_image_width;   cout << "config_image_width:" << config_image_width << endl;
  fs["image_height"] >> config_image_height; cout << "config_image_height:" << config_image_height << endl;


  fs["projection_parameters"]["fx"] >> _fx;
  fs["projection_parameters"]["fy"] >> _fy;
  fs["projection_parameters"]["cx"] >> _cx;
  fs["projection_parameters"]["cy"] >> _cy;

  cout << "projection_parameters :: " << _fx << " " << _fy << " " << _cx << " " << _cy << " " << endl;

  fs["distortion_parameters"]["k1"] >> _k1;
  fs["distortion_parameters"]["k2"] >> _k2;
  fs["distortion_parameters"]["p1"] >> _p1;
  fs["distortion_parameters"]["p2"] >> _p2;
  cout << "distortion_parameters :: " << _k1 << " " << _k2 << " " << _p1 << " " << _p2 << " " << endl;
  cout << "---    ---\n";

  // Define the 3x3 Projection matrix eigen and/or cv::Mat.
  m_K = cv::Mat::zeros( 3,3,CV_32F );
  m_K.at<float>(0,0) = _fx;
  m_K.at<float>(1,1) = _fy;
  m_K.at<float>(0,2) = _cx;
  m_K.at<float>(1,2) = _cy;
  m_K.at<float>(2,2) = 1.0;

  m_D = cv::Mat::zeros( 4, 1, CV_32F );
  m_D.at<float>(0,0) = _k1;
  m_D.at<float>(1,0) = _k2;
  m_D.at<float>(2,0) = _p1;
  m_D.at<float>(3,0) = _p2;
  cout << "m_K" << m_K << endl;
  cout << "m_D" << m_D << endl;

  // Define 4x1 vector of distortion params eigen and/or cv::Mat.
  e_K << _fx, 0.0, _cx,
        0.0,  _fy, _cy,
        0.0, 0.0, 1.0;

  e_D << _k1 , _k2, _p1 , _p2;
  cout << "e_K:\n" << e_K << endl;
  cout << "e_D:\n" << e_D << endl;
  mValid = true;

}



void PinholeCamera::printCameraInfo( int verbosity ) const
{
  cout << "====== PinholeCamera::printCameraInfo ======\n";
  if( !isValid() )
    cout << "camera not set\n";

  cout << "config_file: " << this->config_file_name << endl;
  cout << "Image Rows: " << getImageRows() << endl;
  cout << "Image Cols: " << getImageCols() << endl;
  cout << "fx: " << fx() << " ;  fy: " << fy() << endl;
  cout << "cx: " << cx() << " ;  cy: " << cy() << endl;
  cout << "(distortion params)k1,k2,p1,p2" << " " <<  k1() << " " <<  k2() << " " <<  p1() << " " <<  p2() << endl;

  if( verbosity >= 1 ) {
    cout << "--cv::Mat \n";
    cout << "m_K\n" << m_K << endl;
    cout << "m_D\n" << m_D << endl;

    cout << "--Eigen::Matrix \n";
    cout << "e_K\n" << e_K << endl;
    cout << "e_K\n" << e_D << endl;
  }

  cout << "================================================\n";


}


void PinholeCamera::print_cvmat_info( string msg, const cv::Mat& A )
{
  cout << msg << ":" << "rows=" << A.rows << ", cols=" << A.cols << ", ch=" << A.channels() << ", type=" << type2str( A.type() ) << endl;
}

string PinholeCamera::type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}
