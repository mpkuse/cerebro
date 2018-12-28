// A usage of my Stereo class.
#include <iostream>
using namespace std;




/** Experiments to project 3d points on images.
*/

#include <iostream>

#include "../utils/PoseManipUtils.h"
#include "../utils/RawFileIO.h"
#include "../utils/MiscUtils.h"

// #include "PinholeCamera.h"
#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"

// const string base_path = "/Bulk_Data/_tmp_cerebro/tum_magistrale1/";
const string base_path = "/Bulk_Data/_tmp/";

// TODO Should return read status, ie. false if file-not-found
bool load_i( int i, cv::Mat& im, Matrix4d& wTc, MatrixXd& uv, MatrixXd& cX )
{
    string fname = base_path+"/"+std::to_string( i );

    // Image
    im = cv::imread( fname+".jpg" );
    // cv::imshow( "win", im );

    // VINS-pose (odometry)
    RawFileIO::read_eigen_matrix( fname+".wTc", wTc );

    // 3D points - in camera cords
    RawFileIO::read_eigen_matrix( fname+".cX.pointcloud", cX );

    // 2D points - tracked points
    RawFileIO::read_eigen_matrix( fname+".uv", uv );

}

// TODO Should return read status, ie. false if file-not-found
bool load_i( int i, cv::Mat& im, Matrix4d& wTc, MatrixXd& uv, MatrixXd& cX, VectorXi& uv_ids )
{
    string fname = base_path+"/"+std::to_string( i );

    // Image
    im = cv::imread( fname+".jpg" );
    // cv::imshow( "win", im );

    // VINS-pose (odometry)
    RawFileIO::read_eigen_matrix( fname+".wTc", wTc );

    // 3D points - in camera cords
    RawFileIO::read_eigen_matrix( fname+".cX.pointcloud", cX );

    // 2D points - tracked points
    RawFileIO::read_eigen_matrix( fname+".uv", uv );

    // IDs
    RawFileIO::read_eigen_matrix( fname+".id", uv_ids );

}

int main()
{
    //
    // Load Camera - from yaml
    std::string calib_file = base_path+"/cameraIntrinsic.0.yaml";
    camodocal::CameraPtr m_camera;
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    std::cout << ((m_camera)?"cam is initialized":"cam is not initiazed") << std::endl; //this should print 'initialized'





    // Load 1st
    int _idx_1 = 1331;
    cv::Mat im_1;
    Matrix4d wTc_1;
    MatrixXd uv_1, cX_1;
    VectorXi uv_id_1;
    load_i( _idx_1, im_1, wTc_1, uv_1, cX_1, uv_id_1 );
    // cv::imshow( "533", im_1 );
    {
        cv::Mat dst;
        MiscUtils::plot_point_sets( im_1, uv_1, dst, cv::Scalar(255,0,0), uv_id_1, "self uv" );
        cv::imshow( std::to_string(_idx_1), dst );
    }


    // // Load 2nd
    int _idx_2 = 839;
    cv::Mat im_2;
    Matrix4d wTc_2;
    MatrixXd uv_2, cX_2;
    VectorXi uv_id_2;
    // load_i( 536, im_2, wTc_2, uv_2, cX_2, uv_id_2 );
    // load_i( 887, im_2, wTc_2, uv_2, cX_2 );
    load_i( _idx_2, im_2, wTc_2, uv_2, cX_2, uv_id_2 );
    // cv::imshow( "535", im_2 );
    {
        cv::Mat dst;
        MiscUtils::plot_point_sets( im_2, uv_2, dst, cv::Scalar(255,255,255), uv_id_2, "self uv" );
        cv::imshow( std::to_string(_idx_2), dst );
    }


    // project 1st 3d points using camodocal
    MatrixXd reprojected_uv_1 = MatrixXd::Zero( 2, cX_1.cols() );
    for( int i=0 ; i<cX_1.cols() ; i++ )
    {
        Vector3d P = Vector3d( cX_1.col(i).topRows(3) );
        cout << "P.rows=" << P.rows() << "  P.cols()=" << P.cols() << endl;
        cout << P << endl;

        Vector2d p;
        m_camera->spaceToPlane( P, p );
        cout << "---\n" << p << endl;
        reprojected_uv_1.col(i) = p;
    }
    cv::Mat dst;
    // MiscUtils::plot_point_sets( im_1, uv_1, dst, cv::Scalar(255,0,0), uv_id_1, "self uv in blue" );
    MiscUtils::plot_point_sets( im_2, reprojected_uv_1, dst, cv::Scalar(0,0,255), uv_id_1, ";self reproejct in red" );
    cv::imshow( "reproejct", dst );


    cv::waitKey(0);
    return 0;

}
