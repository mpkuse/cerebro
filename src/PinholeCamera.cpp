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


// Input 3d points in homogeneous co-ordinates 4xN matrix. Eigen I/O
void PinholeCamera::perspectiveProject3DPoints( const MatrixXd& c_X,
                              MatrixXd& out_pts )
{

    // DIY - Do It Yourself Projection
    // c_X.row(0).array() /= c_X.row(3).array();
    // c_X.row(1).array() /= c_X.row(3).array();
    // c_X.row(2).array() /= c_X.row(3).array();
    // c_X.row(3).array() /= c_X.row(3).array();



    // K [ I | 0 ]
    MatrixXd I_0;
    I_0 = Matrix4d::Identity().topLeftCorner<3,4>();
    // MatrixXf P1;
    // P1 = cam_intrin * I_0; //3x4

    // Project and Perspective Divide
    MatrixXd im_pts;
    im_pts = I_0 * c_X; //in normalized image co-ordinate. Beware that distortion need to be applied in normalized co-ordinates
    im_pts.row(0).array() /= im_pts.row(2).array();
    im_pts.row(1).array() /= im_pts.row(2).array();
    im_pts.row(2).array() /= im_pts.row(2).array();

    // Apply Distortion
    MatrixXd Xdd = MatrixXd( im_pts.rows(), im_pts.cols() );
    for( int i=0 ; i<im_pts.cols() ; i++)
    {
      double r2 = im_pts(0,i)*im_pts(0,i) + im_pts(1,i)*im_pts(1,i);
      double c = 1.0f + (double)k1()*r2 + (double)k2()*r2*r2;
      Xdd(0,i) = im_pts(0,i) * c + 2.0f*(double)p1()*im_pts(0,i)*im_pts(1,i) + (double)p2()*(r2 + 2.0*im_pts(0,i)*im_pts(0,i));
      Xdd(1,i) = im_pts(1,i) * c + 2.0f*(double)p2()*im_pts(0,i)*im_pts(1,i) + (double)p1()*(r2 + 2.0*im_pts(1,i)*im_pts(1,i));
      Xdd(2,i) = 1.0f;
    }

    out_pts = e_K * Xdd;


}
