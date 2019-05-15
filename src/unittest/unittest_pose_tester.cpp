// This unittest will test and evaluate
// ` StaticTheiaPoseCompute::P3P_ICP` and `StaticTheiaPoseCompute::PNP`
// from DlsPnpWithRansac.h/cpp
//  Author  : Manohar Kuse <mpkuse@connect.ust.hk>
//  Created : 14th May, 2019
//


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

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;

// custom
#include "../utils/CameraGeometry.h"
#include "../utils/TermColor.h"
#include "../utils/RawFileIO.h"

#include "../utils/PointFeatureMatching.h"
// #include "../utils/GMSMatcher/gms_matcher.h"


#include "../DlsPnpWithRansac.h"

const std::string BASE = "/Bulk_Data/_tmp/";

std::shared_ptr<StereoGeometry> make_stereo_geom()
{
    //--------- Intrinsics load
    camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.0.yaml");
    camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(BASE+"/cameraIntrinsic.1.yaml");
    // camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_left.yaml");
    // camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(string(BASE+"../leveled_cam_pinhole/")+"/camera_right.yaml");

    cout << left_camera->parametersToString() << endl;
    cout << right_camera->parametersToString() << endl;

    if( !left_camera || !right_camera )
    {
        cout << "\nERROR : Abstract stereo Camera is not set. You need to init camera before setting it to geometry clases\n";
        exit(10);
    }


    //----------- Stereo Base line load (alsoed called extrinsic calibration)
        // mynt eye
    // Vector4d q_xyzw = Vector4d( -7.0955103716253032e-04,-1.5775578725333590e-03,-1.2732644461763854e-03,9.9999769332040711e-01 );
    // Vector3d tr_xyz = Vector3d( -1.2006984141573309e+02/1000.,3.3956264524978619e-01/1000.,-1.6784055634087214e-01/1000. );

        // realsense
    Vector4d q_xyzw = Vector4d( 0.0, 0.0, 0.0, 1.0 );
    Vector3d tr_xyz = Vector3d( -49.8/1000., 0.0/1000., 0.0/1000. );

    Matrix4d right_T_left;
    PoseManipUtils::raw_xyzw_to_eigenmat( q_xyzw, tr_xyz, right_T_left );
    cout << "XXright_T_left: " << PoseManipUtils::prettyprintMatrix4d( right_T_left ) << endl;

    // StereoGeometry* sthm = new StereoGeometry( left_camera, right_camera, right_T_left );
    // cout << "sthn->get_K() " << sthm->get_K() << endl;


    // StereoGeometry NNN = StereoGeometry( left_camera, right_camera, right_T_left );
    // cout << "NNN->get_K() " << NNN.get_K() << endl;

    //-------------------- init stereogeom
    std::shared_ptr<StereoGeometry> stereogeom;
    stereogeom = std::make_shared<StereoGeometry>( left_camera,right_camera,     right_T_left  );
    // stereogeom->set_K( 375.0, 375.0, 376.0, 240.0 );

    cout << "[in make_stereo_geom]" << stereogeom->get_K() << endl;

    return stereogeom;

}

bool compute_rel_pose( std::shared_ptr<StereoGeometry> stereogeom, int frame_a, int frame_b )
{
    //------------------------------------------------
    //------ 3d points from frame_a
    //------------------------------------------------
    cout << TermColor::iGREEN() << "LOAD[A]: BASE/"+std::to_string(frame_a)+".jpg" << TermColor::RESET() << endl;
    cv::Mat a_imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_a)+".jpg", 0 );
    cv::Mat a_imright_raw = cv::imread( BASE+"/"+std::to_string(frame_a)+"_1.jpg", 0 );
    Matrix4d wTA;
    RawFileIO::read_eigen_matrix( BASE+"/"+std::to_string(frame_a)+".wTc", wTA );
    cout << "wTA: " << PoseManipUtils::prettyprintMatrix4d( wTA ) << endl;
    if( a_imright_raw.empty() || a_imright_raw.empty() )
    {
        cout << TermColor::RED() << "Cannot read image with idx=" << frame_a << " perhaps filenotfound" << TermColor::RESET() << endl;
        return false;
    }


    cv::Mat a_imleft_srectified, a_imright_srectified;
    cv::Mat a_3dImage;
    MatrixXd a_3dpts;
    cv::Mat a_disp_viz;
    stereogeom->get_srectifiedim_and_3dpoints_and_3dmap_and_disparity_from_raw_images( a_imleft_raw, a_imright_raw,
        a_imleft_srectified,a_imright_srectified,  a_3dpts, a_3dImage, a_disp_viz );



    //-----------------------------------------------------
    //--------------- 3d points from frame_b
    //-----------------------------------------------------
    cout << TermColor::iGREEN() << "LOAD[B]: BASE/"+std::to_string(frame_b)+".jpg" << TermColor::RESET() << endl;
    cv::Mat b_imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_b)+".jpg", 0 );
    cv::Mat b_imright_raw = cv::imread( BASE+"/"+std::to_string(frame_b)+"_1.jpg", 0 );
    Matrix4d wTB;
    RawFileIO::read_eigen_matrix( BASE+"/"+std::to_string(frame_b)+".wTc", wTB );
    cout << "wTB: " << PoseManipUtils::prettyprintMatrix4d( wTB ) << endl;
    if( b_imleft_raw.empty() || b_imright_raw.empty() )
    {
        cout << TermColor::RED() << "Cannot read image with idx=" << frame_b << " perhaps filenotfound" << TermColor::RESET() << endl;
        return false;
    }
    cv::Mat b_imleft_srectified, b_imright_srectified;
    cv::Mat b_3dImage;
    MatrixXd b_3dpts;
    cv::Mat b_disp_viz;
    stereogeom->get_srectifiedim_and_3dpoints_and_3dmap_and_disparity_from_raw_images( b_imleft_raw, b_imright_raw,
                            b_imleft_srectified, b_imright_srectified,  b_3dpts, b_3dImage, b_disp_viz );


    cout << "ODOM b_T_a: " << PoseManipUtils::prettyprintMatrix4d( wTB.inverse() * wTA ) << endl;

    //---------------------------------------------------------------------
    //------------ point matches between a_left, b_left
    //---------------------------------------------------------------------
    cout << TermColor::iGREEN() << "point matches between a_left, b_left" << TermColor::RESET() << endl;
    MatrixXd uv, uv_d; // u is from frame_a; ud is from frame_b
    ElapsedTime timer;
    timer.tic();
    StaticPointFeatureMatching::gms_point_feature_matches(a_imleft_srectified, b_imleft_srectified, uv, uv_d );
    string msg_pf_matches = to_string( timer.toc_milli() )+" (ms) elapsed time for point_feature_matches computation";
    if( uv.cols() < 30 ) {
        cout << TermColor::RED() << "too few gms matches between " << frame_a << " and " << frame_b << TermColor::RESET() << endl;
        return false;
    }


    #if 1 // plot ppoint-feature matches
    cv::Mat dst_feat_matches;
    MiscUtils::plot_point_pair( a_imleft_srectified, uv, frame_a,
                     b_imleft_srectified, uv_d, frame_b,
                        dst_feat_matches, 3, msg_pf_matches+";#pf-matches: "+to_string( uv.cols() )  );
    cv::resize(dst_feat_matches, dst_feat_matches, cv::Size(), 0.5, 0.5 );
    cv::imshow( "dst_feat_matches", dst_feat_matches );
    #endif

    #if 0  // disparty maps of both images
    cv::Mat dst_disp;
    MiscUtils::side_by_side( a_disp_viz, b_disp_viz, dst_disp );
    MiscUtils::append_status_image( dst_disp, "a="+ to_string(frame_a)+"     b="+to_string(frame_b), .8 );
    cv::resize(dst_disp, dst_disp, cv::Size(), 0.5, 0.5 );
    cv::imshow( "dst_disp", dst_disp );
    #endif


    //----------------------------------------------------------------------
    //----------- compute pose
    //----------------------------------------------------------------------
    cout << TermColor::iGREEN() << "compute pose" << TermColor::RESET() << endl;
    #if 0 // pnp( 3d_pts_of_a, 2d_pts_of_b )
    cout << TermColor::iBLUE() << " pnp( 3d_pts_of_a, 2d_pts_of_b )" << TermColor::RESET() << endl;


    //-----------prep data
    std::vector<Eigen::Vector2d> feature_position_uv;   //< these will be normalized image co-ordinates
    std::vector<Eigen::Vector2d> feature_position_uv_d; //< these will be normalized image co-ordinates
    std::vector<Eigen::Vector3d> world_point_uv;
    StaticPointFeatureMatching::make_3d_2d_collection__using__pfmatches_and_disparity(
        stereogeom, uv, a_3dImage, uv_d,
                                feature_position_uv, feature_position_uv_d, world_point_uv);


    //----------initial guess
    Matrix4d out_b_T_a = Matrix4d::Identity();
    out_b_T_a = wTB.inverse() * wTA ;
    cout << TermColor::YELLOW() <<  "initial_guess___out_b_T_a : " << PoseManipUtils::prettyprintMatrix4d( out_b_T_a ) << TermColor::RESET() << endl;


    //---------- PNP
    string ceres_pnp_msh;
    StaticCeresPoseCompute::PNP( world_point_uv, feature_position_uv_d, out_b_T_a, ceres_pnp_msh );

    Matrix4d out_b_T_a111 = Matrix4d::Identity();
    string usual_msg;
    StaticTheiaPoseCompute::PNP( world_point_uv, feature_position_uv_d, out_b_T_a111, usual_msg );
    cout << TermColor::YELLOW() <<  "final_soltion___out_b_T_a(theia) : " << PoseManipUtils::prettyprintMatrix4d( out_b_T_a111 ) << TermColor::RESET() << endl;


    //--------final solution
    cout << TermColor::YELLOW() <<  "final_soltion___out_b_T_a(ceres) : " << PoseManipUtils::prettyprintMatrix4d( out_b_T_a ) << TermColor::RESET() << endl;


    #if 1 //verification

    //---------------------------------------------
    //--- Use the theia's estimated pose do PI( b_T_a * 3dpts ) and plot these
    //--- points on B. Also plot detected points of B
    //---------------------------------------------
    #if 1
        MatrixXd _3dpts_of_A_projectedonB = MatrixXd::Zero(3, world_point_uv.size() );
        MatrixXd _detectedpts_of_B = MatrixXd::Zero(3, world_point_uv.size() );
        MatrixXd _detectedpts_of_A = MatrixXd::Zero(3, world_point_uv.size() );

        int n_good = 0;
        for( int i=0 ; i<world_point_uv.size() ; i++ ) {
            Vector4d a_X_i;
            a_X_i << Vector3d(world_point_uv[i]), 1.0;
            Vector4d b_X_i = out_b_T_a * a_X_i;
            Vector3d _X_i = b_X_i.topRows(3);
            _X_i /= _X_i(2); // Z-division for projection
            _3dpts_of_A_projectedonB.col(i) = stereogeom->get_K() * _X_i; //< scaling with camera-intrinsic matrix


            Vector3d _tmp;
            _tmp << feature_position_uv_d[i], 1.0 ;
            _detectedpts_of_B.col(i) = stereogeom->get_K() * _tmp;

            _tmp << feature_position_uv[i], 1.0 ;
            _detectedpts_of_A.col(i) = stereogeom->get_K() * _tmp;


            Vector3d delta = _3dpts_of_A_projectedonB.col(i) - _detectedpts_of_B.col(i);
            // cout << "i=" << i << " ";
            if( abs(delta(0)) < 2. && abs(delta(1)) < 2. ) {
                // cout << "  delta=" << delta.transpose();
                n_good++;
            }
            else {
                // cout << TermColor::RED() << "delta=" << delta.transpose() << TermColor::RESET();
            }
            // cout << endl;
        }
        cout << "n_good=" <<n_good << " of " << world_point_uv.size() << endl;
    #endif // PI( b_T_a * 3dpts )




    //------------------------------
    //--- Use relative pose of obometry to do PI( odom_b_T_a * 3dpts ) and plot these on image-b
    //--- load odometry poses
    //-------------------------------
    #if 1
        MatrixXd _3dpts_of_A_projectedonB_with_odom_rel_pose = MatrixXd::Zero(3, world_point_uv.size() );
        // Matrix4d wTA;
        // RawFileIO::read_eigen_matrix( BASE+"/"+std::to_string(frame_a)+".wTc", wTA );
        // cout << "wTa(odom)" << PoseManipUtils::prettyprintMatrix4d( wTa ) << endl;
        // Matrix4d wTB;
        // RawFileIO::read_eigen_matrix( BASE+"/"+std::to_string(frame_b)+".wTc", wTB );
        // cout << "wTb(odom)" << PoseManipUtils::prettyprintMatrix4d( wTb ) << endl;
        Matrix4d odom_b_T_a = wTB.inverse() * wTA;
        cout << "odom_b_T_a" << PoseManipUtils::prettyprintMatrix4d( odom_b_T_a ) << endl;

        // PI( odom_b_T_a * a_X )
        for( int i=0 ; i<world_point_uv.size() ; i++ ) {
            Vector4d a_X_i;
            a_X_i << Vector3d(world_point_uv[i]), 1.0;
            Vector4d b_X_i = odom_b_T_a * a_X_i;
            Vector3d _X_i = b_X_i.topRows(3);
            _X_i /= _X_i(2); // Z-division for projection
            _3dpts_of_A_projectedonB_with_odom_rel_pose.col(i) = stereogeom->get_K() * _X_i; //< scaling with camera-intrinsic matrix
        }

    #endif // PI( odom_b_T_a * 3dpts )


    #if 1
        // plot( B, ud )
        cv::Mat __dst;
        MiscUtils::plot_point_sets( b_imleft_srectified, _detectedpts_of_B, __dst, cv::Scalar( 0,0,255), false, "_detectedpts_of_B (red)" );
        // cv::imshow( "plot( B, ud )", __dst );

        // plot( B, _3dpts_of_A_projectedonB )
        MiscUtils::plot_point_sets( __dst, _3dpts_of_A_projectedonB, cv::Scalar( 0,255,255), false, ";_3dpts_of_A_projectedonB, PI( b_T_a * X),(yellow)" );
        // MiscUtils::plot_point_sets( b_imleft_srectified, _3dpts_of_A_projectedonB, __dst, cv::Scalar( 0,255,255), false, "_3dpts_of_A_projectedonB" );

        MiscUtils::plot_point_sets( __dst, _detectedpts_of_A, cv::Scalar( 255,255,255), false, ";;_detectedpts_of_A(white)" );

        MiscUtils::plot_point_sets( __dst, _3dpts_of_A_projectedonB_with_odom_rel_pose, cv::Scalar( 255,0,0), false, ";;;_3dpts_of_A_projectedonB_with_odom_rel_pose, PI( odom_b_T_a * X),(blue)" );

        string status_msg = "";
        status_msg += "_detectedpts_of_B (red);";
        status_msg += "_3dpts_of_A_projectedonB, PI( b_T_a * X),(yellow);";
        status_msg += "_detectedpts_of_A(white);";
        status_msg += "_3dpts_of_A_projectedonB_with_odom_rel_pose, PI( odom_b_T_a * X),(blue)";
        MiscUtils::append_status_image( __dst,  status_msg );

        cv::imshow( "b_imleft_srectified", __dst );
    #endif //plotting

    #endif //verification

    #endif //option-1



    #if 1 // p3p( 3d_pts_of_a, 3d_pts_of_b )
    cout << TermColor::iBLUE() << " p3p( 3d_pts_of_a, 3d_pts_of_b )" << TermColor::RESET() << endl;

    //---------prep data
    vector< Vector3d> uv_X;
    vector< Vector3d> uvd_Y;
    StaticPointFeatureMatching::make_3d_3d_collection__using__pfmatches_and_disparity( uv, a_3dImage, uv_d, b_3dImage,
        uv_X, uvd_Y );

    //-------------initial guess
    Matrix4d icp_b_T_a;
    icp_b_T_a = wTB.inverse() * wTA ;
    cout << TermColor::YELLOW() <<  "initial icp_b_T_a(ceres) : " << PoseManipUtils::prettyprintMatrix4d( icp_b_T_a ) << TermColor::RESET() << endl;


    //------------------p3p (icp)
    string p3p__msg;
    // float p3p_goodness = StaticTheiaPoseCompute::P3P_ICP( uv_X, uvd_Y, icp_b_T_a, p3p__msg );
    float p3p_goodness = StaticCeresPoseCompute::P3P_ICP( uv_X, uvd_Y, icp_b_T_a, p3p__msg );



    //--------final solution
    cout << TermColor::YELLOW() <<  "final icp_b_T_a(ceres) : " << PoseManipUtils::prettyprintMatrix4d( icp_b_T_a ) << TermColor::RESET() << endl;

    #endif //option-3

}

int main(int argc, char ** argv )
{
    //---
    cout << TermColor::iGREEN() << "StereoGeometry\n" << TermColor::RESET() ;
    auto stereogeom = make_stereo_geom();


    //--- Cmdline
    if( argc != 3 ) {
        cout << "[MAIN::INVALID USAGE] I am expecting you to give two numbers in the command line between which you want to compute the pose\n";
        exit(1);
    }
    Matrix4d out_b_T_a = Matrix4d::Identity();
    int a = stoi( argv[1] );
    int b = stoi( argv[2] );


    //--
    compute_rel_pose( stereogeom, a, b );
    cv::waitKey(0);



}


#include "../utils/nlohmann/json.hpp"
using json = nlohmann::json;
int main1( int argc, char ** argv )
{
    std::shared_ptr<StereoGeometry> stereogeom = make_stereo_geom();

    // load ProcessedLoopCandidate.json
    cout << "Open JSON : " << BASE+"/matching/ProcessedLoopCandidate.json" << endl;
    std::ifstream fp(BASE+"/matching/ProcessedLoopCandidate.json");
    json proc_candi_json;
    fp >> proc_candi_json;
    for( int i =0 ; i<proc_candi_json.size() ; i++ ) // loop over all the candidates
    {
        int a = proc_candi_json[i]["idx_a"];
        int b = proc_candi_json[i]["idx_b"];
        cout << a << " " << b << endl;
        bool status = compute_rel_pose( stereogeom, a, b );
        cv::waitKey(0);

    }
}
