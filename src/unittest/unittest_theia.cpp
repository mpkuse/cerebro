// Opens the log.json file :
// A) Plots the vio poses
// B) plot loop candidates
// c) at loop candidate compute relative pose using pnp
// d) plot new camera and old camera.


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

#include "../utils/CameraGeometry.h"


#include "../utils/MiscUtils.h"
#include "../utils/ElapsedTime.h"
#include "../utils/PoseManipUtils.h"
#include "../utils/TermColor.h"
#include "../utils/RawFileIO.h"
#include "../utils/RosMarkerUtils.h"

#include "../utils/nlohmann/json.hpp"
using json = nlohmann::json;

#include "../utils/GMSMatcher/gms_matcher.h"
#include <theia/theia.h>


const std::string BASE = "/Bulk_Data/_tmp/";


void point_feature_matches( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                            MatrixXd& u, MatrixXd& ud )
{
    ElapsedTime timer;


    //
    // Point feature and descriptors extract
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat d1, d2; //< descriptors

    cv::Ptr<cv::ORB> orb = cv::ORB::create(10000);
    orb->setFastThreshold(0);

    timer.tic();
    orb->detectAndCompute(imleft_undistorted, cv::Mat(), kp1, d1);
    orb->detectAndCompute(imright_undistorted, cv::Mat(), kp2, d2);
    cout <<  timer.toc_milli()  << " (ms) 2X detectAndCompute(ms) : "<< endl;
    // std::cout << "d1 " << MiscUtils::cvmat_info( d1 ) << std::endl;
    // std::cout << "d2 " << MiscUtils::cvmat_info( d2 ) << std::endl;

    //plot
    // cv::Mat dst_left, dst_right;
    // MatrixXd e_kp1, e_kp2;
    // MiscUtils::keypoint_2_eigen( kp1, e_kp1 );
    // MiscUtils::keypoint_2_eigen( kp2, e_kp2 );
    // MiscUtils::plot_point_sets( imleft_undistorted, e_kp1, dst_left, cv::Scalar(0,0,255), false );
    // MiscUtils::plot_point_sets( imright_undistorted, e_kp2, dst_right, cv::Scalar(0,0,255), false );
    // cv::imshow( "dst_left", dst_left );
    // cv::imshow( "dst_right", dst_right );

    //
    // Point feature matching
    cv::BFMatcher matcher(cv::NORM_HAMMING); // TODO try FLANN matcher here.
    vector<cv::DMatch> matches_all, matches_gms;
    timer.tic();
    matcher.match(d1, d2, matches_all);
    std::cout << timer.toc_milli() << " : (ms) BFMatcher took (ms)\t";
    std::cout << "BFMatcher : npts = " << matches_all.size() << std::endl;


    // gms_matcher
    timer.tic();
    std::vector<bool> vbInliers;
    gms_matcher gms(kp1, imleft_undistorted.size(), kp2, imright_undistorted.size(), matches_all);
    int num_inliers = gms.GetInlierMask(vbInliers, false, false);
    cout << timer.toc_milli() << " : (ms) GMSMatcher took.\t" ;
    cout << "Got total gms matches " << num_inliers << " matches." << endl;

    // collect matches
    for (size_t i = 0; i < vbInliers.size(); ++i)
    {
        if (vbInliers[i] == true)
        {
            matches_gms.push_back(matches_all[i]);
        }
    }
    // MatrixXd M1, M2;
    MiscUtils::dmatch_2_eigen( kp1, kp2, matches_gms, u, ud, true );


}




std::shared_ptr<StereoGeometry> make_stereo_geom()
{
    //--------- Intrinsics load
    camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.0.yaml");
    camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.1.yaml");
    // camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_left.yaml");
    // camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_right.yaml");

    cout << left_camera->parametersToString() << endl;
    cout << right_camera->parametersToString() << endl;


    //----------- Stereo Base line load (alsoed called extrinsic calibration)
        // mynt eye
    Vector4d q_xyzw = Vector4d( -7.0955103716253032e-04,-1.5775578725333590e-03,-1.2732644461763854e-03,9.9999769332040711e-01 );
    Vector3d tr_xyz = Vector3d( -1.2006984141573309e+02/1000.,3.3956264524978619e-01/1000.,-1.6784055634087214e-01/1000. );
    Matrix4d right_T_left;
    PoseManipUtils::raw_xyzw_to_eigenmat( q_xyzw, tr_xyz, right_T_left );
    cout << "right_T_left: " << PoseManipUtils::prettyprintMatrix4d( right_T_left ) << endl;


    //-------------------- init stereogeom
    std::shared_ptr<StereoGeometry> stereogeom;
    stereogeom = std::make_shared<StereoGeometry>( left_camera,right_camera,     right_T_left  );
    stereogeom->set_K( 375.0, 375.0, 376.0, 240.0 );

    return stereogeom;

}


// will return false if cannot compute pose
bool relative_pose_compute_with_theia( std::shared_ptr<StereoGeometry> stereogeom,
                        int frame_a, int frame_b,
                        Matrix4d& out_b_T_a )
{
    ElapsedTime timer;

    //------ 3d points from frame_a
    cv::Mat a_imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_a)+".jpg", 0 );
    cv::Mat a_imright_raw = cv::imread( BASE+"/"+std::to_string(frame_a)+"_1.jpg", 0 );
    if( a_imright_raw.empty() || a_imright_raw.empty() )
    {
        cout << TermColor::RED() << "Cannot read image with idx=" << frame_a << " perhaps filenotfound" << TermColor::RESET() << endl;
        return false;
    }
    cv::Mat a_imleft_srectified, a_imright_srectified;
    cv::Mat _3dImage;
    MatrixXd _3dpts;
    // stereogeom->get3dpoints_and_3dmap_from_raw_images( a_imleft_raw, a_imright_raw,
                        // _3dpts, _3dImage,
                        // a_imleft_srectified, a_imright_srectified );

    cv::Mat disp_viz;
    timer.tic();
    stereogeom->get_srectifiedim_and_3dpoints_and_3dmap_and_disparity_from_raw_images( a_imleft_raw, a_imright_raw,
        a_imleft_srectified,a_imright_srectified,  _3dpts, _3dImage, disp_viz );
    cout << timer.toc_milli() << " : (ms) get_srectifiedim_and_3dpoints_and_3dmap_and_disparity_from_raw_images\n";


    #if 1
    cv::imshow( "a_imleft_raw", a_imleft_raw );
    cv::imshow( "a_imright_raw", a_imright_raw );
    // cv::imshow( "a_imleft_srectified", a_imleft_srectified );
    // cv::imshow( "a_imright_srectified", a_imright_srectified );
    cv::imshow( "a_disp_viz", disp_viz );
    #endif

    //--------------- stereo rectify b
    cv::Mat b_imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_b)+".jpg", 0 );
    cv::Mat b_imright_raw = cv::imread( BASE+"/"+std::to_string(frame_b)+"_1.jpg", 0 );
    if( b_imleft_raw.empty() || b_imright_raw.empty() )
    {
        cout << TermColor::RED() << "Cannot read image with idx=" << frame_b << " perhaps filenotfound" << TermColor::RESET() << endl;
        return false;
    }
    cv::Mat b_imleft_srectified, b_imright_srectified;
    stereogeom->do_stereo_rectification_of_raw_images( b_imleft_raw, b_imright_raw,
                            b_imleft_srectified, b_imright_srectified );


    //------------ point matches between a_left, b_left
    MatrixXd u, ud; // u is from frame_a; ud is from frame_b
    point_feature_matches(a_imleft_srectified, b_imleft_srectified, u, ud );
    if( u.cols() < 30 ) {
        cout << TermColor::RED() << "too few gms matches between " << frame_a << " and " << frame_b << TermColor::RESET() << endl;
        return false;
    }





    //---------------- make collection of 3d 2d points
    int c = 0;
    MatrixXd ud_normalized = stereogeom->get_K().inverse() * ud;
    MatrixXd u_normalized = stereogeom->get_K().inverse() * u;
    std::vector<Eigen::Vector2d> feature_position;
    std::vector<Eigen::Vector3d> world_point;
    for( int k=0 ; k<u.cols() ; k++ )
    {
        cv::Vec3f _3dpt = _3dImage.at<cv::Vec3f>( (int)u(1,k), (int)u(0,k) );
        if( _3dpt[2] < 0.1 || _3dpt[2] > 25.  )
            continue;

        c++;
        #if 0
        cout << TermColor::RED() << "---" << k << "---" << TermColor::RESET() << endl;
        cout << "ud=" << ud.col(k).transpose() ;
        cout << " <--> ";
        cout << "u=" << u.col(k).transpose() ;
        cout << "  3dpt of u=";
        cout <<  TermColor::GREEN() << _3dpt[0] << " " << _3dpt[1] << " " << _3dpt[2] << " " << TermColor::RESET();
        cout << endl;
        #endif

        feature_position.push_back( Vector2d( ud_normalized(0,k), ud_normalized(1,k) ) );
        // feature_position.push_back( Vector2d( u_normalized(0,k), u_normalized(1,k) ) );
        world_point.push_back( Vector3d( _3dpt[0], _3dpt[1], _3dpt[2] ) );
    }
    cout << "of the total " << u.cols() << " point feature correspondences " << c << " had valid depths\n";
    if( c < 30 ) {
        cout << TermColor::RED() << "too few valid 3d points between " << frame_a << " and " << frame_b << TermColor::RESET() << endl;
        return false;
    }


    #if 1
    cv::Mat dst_feat_matches;
    string msg = string("gms_matcher;")+"of the total "+std::to_string(u.cols())+" point feature correspondences " +std::to_string(c)+ " had valid depths";
    MiscUtils::plot_point_pair( a_imleft_srectified, u, frame_a,
                     b_imleft_srectified, ud, frame_b,
                        dst_feat_matches, 3, msg
                         );
    cv::Mat dst_feat_matches_resized;
    cv::resize(dst_feat_matches, dst_feat_matches_resized, cv::Size(), 0.5, 0.5);

    cv::imshow( "dst_feat_matches", dst_feat_matches_resized);
    #endif

    //------------- TODO assess u, ud, world_point for histogram spreads. a more spread in measurements will give more precise more. Also will be helpful to weedout bad poses

    //------------------ theia pnp
    std::vector<Eigen::Quaterniond> solution_rotations;
    std::vector<Eigen::Vector3d> solution_translations;
    timer.tic();
    theia::DlsPnp( feature_position, world_point, &solution_rotations, &solution_translations  );
    cout << timer.toc_milli() << " : (ms) : theia::DlsPnp done in\n";
    cout << "solutions count = " << solution_rotations.size() << " " << solution_translations.size() << endl;
    if( solution_rotations.size() == 0 ) {
        cout << TermColor::RED() << " theia::DlsPnp returns no solution" << TermColor::RESET() << endl;
        return false;
    }
    Matrix4d b_T_a = Matrix4d::Identity();
    b_T_a.topLeftCorner(3,3) = solution_rotations[0].toRotationMatrix();
    b_T_a.col(3).topRows(3) = solution_translations[0];
    // cout << "solution_T " << b_T_a << endl;
    cout << "solution_T (b_T_a): " << PoseManipUtils::prettyprintMatrix4d( b_T_a ) << endl;
    out_b_T_a = b_T_a;
    return true;


    // TODO : Verify this computed pose by reprojecting 3d points. 

}

void plot_vio_poses( json& json_obj, ros::Publisher& pub )
{
    int n_max = json_obj["DataNodes"].size();
    visualization_msgs::Marker cam, cam_text;
    ros::Rate loop_rate(100);
    for( int i=0 ; i<n_max ; i++ ) {
        if(  json_obj["DataNodes"][i]["isPoseAvailable"] == 1 ) {
            // cout << i << " isPoseAvailable: " <<  json_obj["DataNodes"][i]["isPoseAvailable"] << endl;
            // Matrix4d w_T_c_from_file;
            // RawFileIO::read_eigen_matrix( BASE+"/"+to_string(i)+".wTc", w_T_c_from_file );
            // cout << "w_T_c_from_file "<< w_T_c_from_file << endl;


            Matrix4d w_T_c;
            vector<double> raa = json_obj["DataNodes"][i]["w_T_c"]["data"];
            RawFileIO::read_eigen_matrix( raa, w_T_c );//loads as row-major
            // cout << "w_T_c "<< w_T_c << endl;
            cout << "wTc : " << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl;

            RosMarkerUtils::init_text_marker( cam_text );
            cam_text.ns = "vio_pose_text";
            cam_text.id = i;
            cam_text.text = std::to_string(i);
            cam_text.scale.z = 0.08;
            RosMarkerUtils::setcolor_to_marker( 1.0,1.0,1.0, cam_text );
            RosMarkerUtils::setpose_to_marker( w_T_c, cam_text );
            pub.publish( cam_text );


            RosMarkerUtils::init_camera_marker( cam, 0.7 );
            cam.ns = "vio_pose";
            cam.id = i;
            RosMarkerUtils::setpose_to_marker( w_T_c, cam );
            RosMarkerUtils::setcolor_to_marker( 0.0, 1.0, 0.0, cam );
            pub.publish( cam );
            ros::spinOnce();
            loop_rate.sleep();

        }
    }

}


void plot_loop_candidates_as_lines( json& log, json& loopcandidates, ros::Publisher&pub )
{

    visualization_msgs::Marker loop_line_marker;
    visualization_msgs::Marker loop_line_marker_new;
    std::shared_ptr<StereoGeometry> stereogeom = make_stereo_geom();

    ros::Rate loop_rate(100);
    for( int i=0 ; i<loopcandidates.size(); i++ )
    {
        // int global_a = loopcandidates[i]["global_a"];
        // int global_b = loopcandidates[i]["global_b"];
        int global_a = loopcandidates[i]["global_b"]; //smaller indx to be called `a`. This inverting was done for pose computation.
        int global_b = loopcandidates[i]["global_a"];
        cout << TermColor::iGREEN() << i  << "   global_a=" << global_a << "  global_b=" << global_b << TermColor::RESET() << endl;

        vector<double> r_wta = log["DataNodes"][global_a]["w_T_c"]["data"];
        vector<double> r_wtb = log["DataNodes"][global_b]["w_T_c"]["data"];
        Matrix4d w_T_a, w_T_b;
        RawFileIO::read_eigen_matrix( r_wta, w_T_a );
        RawFileIO::read_eigen_matrix( r_wtb, w_T_b );


        RosMarkerUtils::init_line_marker( loop_line_marker, w_T_a.col(3).topRows(3), w_T_b.col(3).topRows(3)  );
        RosMarkerUtils::setcolor_to_marker( 1.0, 0.0, 0.0, loop_line_marker );
        loop_line_marker.ns = "loopcandidates";
        loop_line_marker.id = i;
        pub.publish( loop_line_marker );



        // PnP
        Matrix4d b_T_a;
        bool status = relative_pose_compute_with_theia( stereogeom, global_a,global_b, b_T_a );
        if( status ) {
            Matrix4d w_Tcap_b =  w_T_a * b_T_a.inverse(); //< new pose

            cout << "w_T_"<< global_a << ": " << PoseManipUtils::prettyprintMatrix4d( w_T_a ) << endl;
            cout << "w_T_"<< global_b << ": " << PoseManipUtils::prettyprintMatrix4d( w_T_b ) << endl;
            cout << "w_Tcap_"<< global_b << ": " << PoseManipUtils::prettyprintMatrix4d( w_Tcap_b ) << endl; //new pose of b.

            RosMarkerUtils::init_line_marker( loop_line_marker_new, w_T_a.col(3).topRows(3), w_Tcap_b.col(3).topRows(3)  );
            RosMarkerUtils::setcolor_to_marker( 1.0, 1.0, 1.0, loop_line_marker_new );
            loop_line_marker_new.ns = "loopcandidates_corrected";
            loop_line_marker_new.id = i;
            pub.publish( loop_line_marker_new );
            char key = cv::waitKey(0);
            if( key == 'q' ) {
                cout << "q pressed\n";
                return;
            }
        }
        else {
            // blue-out the edge which failed geometric verification
            RosMarkerUtils::setcolor_to_marker( 0.0, 0.0, 1.0, loop_line_marker );
            pub.publish( loop_line_marker );
        }




        ros::spinOnce();
        loop_rate.sleep();

    }
}

int main()
{
    std::shared_ptr<StereoGeometry> stereogeom = make_stereo_geom();

    // will return false if cannot compute pose
    Matrix4d out_b_T_a;
    bool status = relative_pose_compute_with_theia(stereogeom, 1276, 1354, out_b_T_a );
    cv::waitKey(0);

    cout << "####RESULT####\n";
    cout << "status: " << status << endl;
    cout << PoseManipUtils::prettyprintMatrix4d( out_b_T_a );

}

int main1( int argc, char ** argv )
{

    ///////// ROS INIT
    ros::init(argc, argv, "camera_geometry_inspect_ptcld" );
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<visualization_msgs::Marker>("chatter", 1000);
    ros::Rate loop_rate(10);


    //////////// Open `log.json` and `loopcandidates_liverun.json`
    string json_fname = BASE+"/log.json";
    cout << TermColor::GREEN() << "Open file: " << json_fname << TermColor::RESET() <<  endl;
    std::ifstream json_fileptr(json_fname);
    json json_obj;
    json_fileptr >> json_obj;
    cout << "Successfully opened file "<< json_fname << endl;


    string json_loops_fname = BASE+"/loopcandidates_liverun.json";
    cout << TermColor::GREEN() << "Open file: " << json_loops_fname << TermColor::RESET() <<  endl;
    std::ifstream json_fileptr_loops(json_loops_fname);
    json json_obj_loopcandidates;
    json_fileptr_loops >> json_obj_loopcandidates;
    cout << "Successfully opened file "<< json_loops_fname << endl;


    ///////////// plot_vio_poses
    plot_vio_poses( json_obj, chatter_pub );

    //////////// plot loop candidates
    plot_loop_candidates_as_lines(  json_obj, json_obj_loopcandidates, chatter_pub );



    cout << "ros::spin()\n. Press CTRL+C to quit\n";
    ros::spin();


    ///// stereo geometry
/*
    std::shared_ptr<StereoGeometry> stereogeom = make_stereo_geom();
    int frame_a = 840;
    int frame_b = 1344;
    Matrix4d b_T_a;
    relative_pose_compute_with_theia( stereogeom, frame_a,frame_b, b_T_a );
    cv::waitKey(0);
*/



}
