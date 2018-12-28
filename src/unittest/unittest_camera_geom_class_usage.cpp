#include <iostream>
using namespace std;

#include <iostream>
#include <string>

#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../utils/MiscUtils.h"
#include "../utils/ElapsedTime.h"
#include "../utils/PoseManipUtils.h"
#include "../utils/TermColor.h"
#include "../utils/CameraGeometry.h"
#include "../utils/RawFileIO.h"

// #include "gms_matcher.h"

#include <assert.h>

#include <ceres/ceres.h>
using namespace ceres;


// Monocular
int monocular_demo()
{
    const std::string BASE = "/Bulk_Data/_tmp/";


    // Abstract Camera
    camodocal::CameraPtr m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.0.yaml");
    cout << m_camera->parametersToString() << endl;

    // Init Monocular Geometry
    MonoGeometry monogeom( m_camera );

    // Raw Image - Image from camera
    int frame_id = 23;
    cv::Mat im_raw =  cv::imread( BASE+"/"+std::to_string(frame_id)+".jpg" );

    cv::Mat im_undistorted;
    monogeom.do_image_undistortion( im_raw, im_undistorted );


    cv::imshow( "im_raw", im_raw );
    cv::imshow( "im_undistorted", im_undistorted );
    cv::waitKey(0);
}


// Stereo
int stereo_demo()
{
    const std::string BASE = "/Bulk_Data/_tmp/";

    //--------- Intrinsics load
    camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.0.yaml");
    camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.1.yaml");
    cout << left_camera->parametersToString() << endl;
    cout << right_camera->parametersToString() << endl;

    //----------- Stereo Base line load (alsoed called extrinsic calibration)
        // mynt eye
    // TODO function to load extrinsic from yaml file.
    // Vector4d q_xyzw = Vector4d( -1.8252509868889259e-04,-1.6291774489779708e-03,-1.2462127842978489e-03,9.9999787970731446e-01 );
    // Vector3d tr_xyz = Vector3d( -1.2075905420832895e+02/1000.,5.4110610639412482e-01/1000.,2.4484815673909591e-01/1000. );
    Vector4d q_xyzw = Vector4d( -7.0955103716253032e-04,-1.5775578725333590e-03,-1.2732644461763854e-03,9.9999769332040711e-01 );
    Vector3d tr_xyz = Vector3d( -1.2006984141573309e+02/1000.,3.3956264524978619e-01/1000.,-1.6784055634087214e-01/1000. );
    Matrix4d right_T_left;
    PoseManipUtils::raw_xyzw_to_eigenmat( q_xyzw, tr_xyz, right_T_left );


    std::shared_ptr<StereoGeometry> stereogeom;
    stereogeom = std::make_shared<StereoGeometry>( left_camera,right_camera,     right_T_left  );
    stereogeom->set_K( 375.0, 375.0, 376.0, 240.0 );


    int frame_id = 1005;
    cout << "READ IMAGE " << frame_id << endl;
    cv::Mat imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_id)+".jpg", 0 );
    cv::Mat imright_raw =  cv::imread( BASE+"/"+std::to_string(frame_id)+"_1.jpg", 0 );

    if( imleft_raw.empty() || imright_raw.empty() ) {
        cout << "imleft_raw is empty OR imright_raw is empty\n";
        return 0;
    }

    // will get 3d points, stereo-rectified image, and disparity false colormap
    MatrixXd _3dpts; //4xN
    cv::Mat imleft_srectified, imright_srectified;
    cv::Mat disparity_for_visualization;

    ElapsedTime timer;
    timer.tic();
    stereogeom->get_srectifiedim_and_3dpoints_and_disparity_from_raw_images(imleft_raw, imright_raw,
        imleft_srectified, imright_srectified,
         _3dpts, disparity_for_visualization );
    cout << timer.toc_milli() << " (ms)!!  get_srectifiedim_and_3dpoints_and_disparity_from_raw_images\n";

    cv::imshow( "imleft_srectified", imleft_srectified );
    cv::imshow( "imright_srectified", imright_srectified );
    cv::imshow( "disparity_for_visualization", disparity_for_visualization );

    cv::waitKey(0);
    return 0;

}

int main()
{
    cout << "Gello Horld\n";
    // monocular_demo();
    stereo_demo();
}
