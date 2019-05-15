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




/////////// DlsPnp-RANSAC ///////////////////
// Data
struct CorrespondencePair_3d2d {
    Vector3d a_X; // 3d point expressed in co-ordinate system of `image-a`
    Vector2d uv_d; //feature point in normalized-image co-ordinates. of the other view (b)
};

// Model
struct RelativePose {
    Matrix4d b_T_a;
};


class DlsPnpWithRansac: public theia::Estimator<CorrespondencePair_3d2d, RelativePose> {
public:
    // Number of points needed to estimate a line.
    double SampleSize() const  { return 15; }

    // Estimate a pose from N points
    bool EstimateModel( const std::vector<CorrespondencePair_3d2d>& data,
                            std::vector<RelativePose>* models) const {

        // cout << "EstimateModel: data.size="<< data.size() << " " <<  data[0].uv_d.transpose() << endl;
        std::vector<Vector2d> feature_position;
        std::vector<Vector3d> world_point;
        for( int i=0 ; i<data.size() ; i++ ) {
            feature_position.push_back( data[i].uv_d );
            world_point.push_back( data[i].a_X );
        }

        std::vector<Quaterniond> solution_rotations;
        std::vector<Vector3d> solution_translations;

        theia::DlsPnp( feature_position, world_point, &solution_rotations, &solution_translations );
        if( solution_rotations.size() == 1 ) {
            RelativePose r;
            r.b_T_a = Matrix4d::Identity();
            r.b_T_a.topLeftCorner(3,3) = solution_rotations[0].toRotationMatrix();
            r.b_T_a.col(3).topRows(3) = solution_translations[0];
            models->push_back( r );
            return true;
        }
        else {
            return false;
        }
    }


    double Error( const CorrespondencePair_3d2d& point, const RelativePose& r ) const {
        Vector3d  b_X = r.b_T_a.topLeftCorner(3,3) * point.a_X + r.b_T_a.col(3).topRows(3);
        double depth = b_X(2);
        b_X /= depth;
        double reproj_error = abs(b_X(0) - point.uv_d(0)) + abs(b_X(1) - point.uv_d(1));

        // I want nearby points to have smaller Error than far off points.
        // So artificially increase the error when depth is higher for the point.
        double f = 1.;
        #if 0
        if( depth < 1. )
            f = .5;
        if( depth >=1 && depth < 3 )
            f = 5.0;
        if( depth >=3 && depth < 8 )
            f = 2.;
        if( depth >= 8. && depth < 15 )
            f = 1.7;
        if( depth >= 15 )
            f = 0.1;
        #endif

        return reproj_error*f;

    }
};
/////////// END DlsPnp-RANSAC ///////////////////



//////////////////// AlignPointCloudsUmeyama with Ransac ///////////////////////
// Data
struct CorrespondencePair_3d3d {
    Vector3d a_X; // 3d point expressed in co-ordinate system of `image-a`
    Vector3d b_X; // 3d point expressed in co-ordinate system of `image-b`
};

// Model
// struct RelativePose {
    // Matrix4d b_T_a;
// };

class AlignPointCloudsUmeyamaWithRansac: public theia::Estimator<CorrespondencePair_3d3d, RelativePose> {
public:
    // Number of points needed to estimate a line.
    double SampleSize() const  { return 10; }

    bool EstimateModel( const std::vector<CorrespondencePair_3d3d>& data,
                            std::vector<RelativePose>* models) const {
        //TODO

        std::vector<Vector3d> a_X;
        std::vector<Vector3d> b_X;
        for( int i=0 ; i<data.size() ; i++ ) {
            a_X.push_back( data[i].a_X );
            b_X.push_back( data[i].b_X );
        }

        Matrix3d ____R;
        Vector3d ____t; double ___s=1.0;
        theia::AlignPointCloudsUmeyama( a_X, b_X, &____R, &____t, &___s );

        if( min( ___s, 1.0/___s ) > 0.9 ) {
            RelativePose r;
            r.b_T_a = Matrix4d::Identity();
            r.b_T_a.topLeftCorner(3,3) = ____R;
            r.b_T_a.col(3).topRows(3) = ____t;
            models->push_back( r );
            // cout << "Estimate s=" << ___s << " " << PoseManipUtils::prettyprintMatrix4d(r.b_T_a)<< endl;
            return true;
        } else {
            return false;
        }

    }

    double Error( const CorrespondencePair_3d3d& point, const RelativePose& r ) const {
        Vector3d  b_X_cap = r.b_T_a.topLeftCorner(3,3) * point.a_X + r.b_T_a.col(3).topRows(3);

        double err = (b_X_cap - point.b_X).norm();

        double f=1.0;
        if( point.a_X(2) < 1. && point.a_X(2) > 8. )
            f = 1.2;

        // cout << "ICP RAnsac error=" << err << endl;

        return f*err;

    }

};

//////////////////// END AlignPointCloudsUmeyama with Ransac ///////////////////////

void point_feature_matches( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                            MatrixXd& u, MatrixXd& ud )
{
    ElapsedTime timer;


    //
    // Point feature and descriptors extract
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat d1, d2; //< descriptors

    cv::Ptr<cv::ORB> orb = cv::ORB::create(5000);
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

    if( !left_camera || !right_camera )
    {
        cout << "\nERROR : Abstract stereo Camera is not set. You need to init camera before setting it to geometry clases\n";
        exit(10);
    }


    //----------- Stereo Base line load (alsoed called extrinsic calibration)
        // mynt eye
    Vector4d q_xyzw = Vector4d( -7.0955103716253032e-04,-1.5775578725333590e-03,-1.2732644461763854e-03,9.9999769332040711e-01 );
    Vector3d tr_xyz = Vector3d( -1.2006984141573309e+02/1000.,3.3956264524978619e-01/1000.,-1.6784055634087214e-01/1000. );
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


// will return false if cannot compute pose
bool relative_pose_compute_with_theia( std::shared_ptr<StereoGeometry> stereogeom,
                        int frame_a, int frame_b,
                        Matrix4d& out_b_T_a )
{
    ElapsedTime timer;

    //------------------------------------------------
    //------ 3d points from frame_a
    //------------------------------------------------
    cout << TermColor::iGREEN() << "LOAD: BASE/"+std::to_string(frame_a)+".jpg" << TermColor::RESET() << endl;
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

    //-----------------------------------------------------
    //--------------- stereo rectify b
    //-----------------------------------------------------
    cout << TermColor::iGREEN() << "LOAD: BASE/"+std::to_string(frame_b)+".jpg" << TermColor::RESET() << endl;
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



    //---------------------------------------------------------------------
    //------------ point matches between a_left, b_left
    //---------------------------------------------------------------------
    MatrixXd u, ud; // u is from frame_a; ud is from frame_b
    point_feature_matches(a_imleft_srectified, b_imleft_srectified, u, ud );
    if( u.cols() < 30 ) {
        cout << TermColor::RED() << "too few gms matches between " << frame_a << " and " << frame_b << TermColor::RESET() << endl;
        return false;
    }





    //---------------- make collection of 3d 2d points
    int c = 0;
    MatrixXd u_normalized = stereogeom->get_K().inverse() * u;
    MatrixXd ud_normalized = stereogeom->get_K().inverse() * ud;
    std::vector<Eigen::Vector3d> world_point;
    std::vector<Eigen::Vector2d> feature_position_ud;
    std::vector<Eigen::Vector2d> feature_position_u;
    for( int k=0 ; k<u.cols() ; k++ )
    {
        cv::Vec3f _3dpt = _3dImage.at<cv::Vec3f>( (int)u(1,k), (int)u(0,k) );
        if( _3dpt[2] < 0.1 || _3dpt[2] > 25.  )
            continue;

        // the 3d point should also project to same 2d points.


        c++;
        #if 0
        cout << TermColor::RED() << "---" << k << "---" << TermColor::RESET() << endl;
        cout << "ud=" << ud.col(k).transpose() ;
        cout << " <--> ";
        cout << "u=" << u.col(k).transpose() ;
        cout << "  3dpt of u=";
        cout <<  TermColor::GREEN() << _3dpt[0] << " " << _3dpt[1] << " " << _3dpt[2] << " " << TermColor::RESET();

        // project 3d point with Identity
        Vector3d X_i = Vector3d( (double) _3dpt[0],(double) _3dpt[1],(double) _3dpt[2] );
        X_i(0) /= X_i(2);
        X_i(1) /= X_i(2);
        X_i(2) /= X_i(2);
        Vector3d u_cap = stereogeom->get_K() * X_i;
        cout << "\nPI(3dpt)=" << u_cap.transpose() ;

        auto delta = u_cap - u.col(k);
        if( abs(delta(0))  < 2. && abs(delta(1)) < 2. ) {
            cout << TermColor::GREEN() << "\tdelta=" << delta.transpose() << TermColor::RESET();
            make_n_good++;
        }
        else
            cout << "\tdelta=" << delta.transpose() ;

        cout << endl;
        #endif

        feature_position_ud.push_back( Vector2d( ud_normalized(0,k), ud_normalized(1,k) ) );
        feature_position_u.push_back( Vector2d( u_normalized(0,k), u_normalized(1,k) ) );

        world_point.push_back( Vector3d( _3dpt[0], _3dpt[1], _3dpt[2] ) );
        // world_point.push_back( X_i );
    }
    cout << TermColor::GREEN() << "of the total " << u.cols() << " point feature correspondences " << c << " had valid depths" << TermColor::RESET() << endl;
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


    //---------------------------------------------------------------------
    //------------------ theia pnp
    //---------------------------------------------------------------------


    #if 1
    // simple DlsPnp()
    std::vector<Eigen::Quaterniond> solution_rotations;
    std::vector<Eigen::Vector3d> solution_translations;
    timer.tic();
    theia::DlsPnp( feature_position_ud, world_point, &solution_rotations, &solution_translations  );
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
    // return true;
    #endif


    #if 1
    // DlsPnp with ransac

    timer.tic();
    // prep data
    vector<CorrespondencePair_3d2d> data_r;
    for( int i=0 ; i<world_point.size() ; i++ )
    {
        CorrespondencePair_3d2d _data;
        _data.a_X = world_point[i];
        _data.uv_d = feature_position_ud[i];
        data_r.push_back( _data );
    }

    // Specify RANSAC parameters.

    DlsPnpWithRansac dlspnp_estimator;
    RelativePose best_rel_pose;

    // Set the ransac parameters.
    theia::RansacParameters params;
    params.error_thresh = 0.02;
    params.min_inlier_ratio = 0.90;
    params.max_iterations = 50;
    params.min_iterations = 5;
    params.use_mle = true;

    // Create Ransac object, specifying the number of points to sample to
    // generate a model estimation.
    theia::Ransac<DlsPnpWithRansac> ransac_estimator(params, dlspnp_estimator);
    // theia::LMed<DlsPnpWithRansac> ransac_estimator(params, dlspnp_estimator);
    // Initialize must always be called!
    ransac_estimator.Initialize();

    theia::RansacSummary summary;
    ransac_estimator.Estimate(data_r, &best_rel_pose, &summary);
    //   cout << "Line y = " << best_line.m << "*x + " << best_line.b;
    cout << timer.toc_milli() << "(ms)!! DlsPnpWithRansac ElapsedTime includes data prep\n";
    cout << "Ransac:";
    // for( int i=0; i<summary.inliers.size() ; i++ )
        // cout << "\t" << i<<":"<< summary.inliers[i];
    cout << "\tnum_iterations=" << summary.num_iterations;
    cout << "\tconfidence=" << summary.confidence;
    cout << endl;

    cout << "best solution (ransac) : "<< PoseManipUtils::prettyprintMatrix4d( best_rel_pose.b_T_a ) << endl;
    b_T_a = best_rel_pose.b_T_a;

    #endif




    // TODO : Verify this computed pose by reprojecting 3d points.
    cout << "######################\n## Verification ##\n#########################\n";

    #if 0
    //---------------------
    // --- Experiment-1: project 3d points with identity and plot of A
    //------------------------
    MatrixXd _3dpts_of_A_projectedonA = MatrixXd::Zero(3, world_point.size() );
    MatrixXd _detectedpts_of_A = MatrixXd::Zero(3, world_point.size() );
    int n_good = 0;
    for( int i = 0 ; i<world_point.size() ; i++ ) {
        Vector3d _X_i = Vector3d(world_point[i]);
        _X_i /= _X_i(2); // Z-division for projection
        _3dpts_of_A_projectedonA.col(i) = stereogeom->get_K() * _X_i; //< scaling with camera-intrinsic matrix



        Vector3d _tmp;
        _tmp << feature_position_u[i], 1.0 ;
        _detectedpts_of_A.col(i) = stereogeom->get_K() * _tmp;


        // cout << "---\n";
        cout << i << " ) ";
        cout << "_3dpts_of_A_projectedonA.col(i)="<< _3dpts_of_A_projectedonA.col(i).transpose() << "\t";
        cout << "_detectedpts_of_A.col(i)=" << _detectedpts_of_A.col(i).transpose() ;

        Vector3d delta = _3dpts_of_A_projectedonA.col(i) - _detectedpts_of_A.col(i);
        if( abs(delta(0)) < 2. && abs(delta(1)) < 2. ) {
        cout << "  delta=" << delta.transpose();
        n_good++;
        }
        else {
            cout << TermColor::RED() << "delta=" << delta.transpose() << TermColor::RESET();
        }
        cout << endl;
   }
   cout << "# of points which with ok delta=" << n_good << " out of total" << world_point.size() << endl;


    // plot( A, u );
    cv::Mat __dst;
    MiscUtils::plot_point_sets( a_imleft_srectified, _detectedpts_of_A, __dst, cv::Scalar( 0,0,255), false );
    cv::imshow( "plot( A, u )", __dst );


    // plot( A, _3dpts_of_A_projectedonA )
    MiscUtils::plot_point_sets( a_imleft_srectified, _3dpts_of_A_projectedonA, __dst, cv::Scalar( 0,255,255), false );
    cv::imshow( "plot( A, _3dpts_of_A_projectedonA )", __dst );
    #endif


    //---------------------------------------------
    //--- Use the theia's estimated pose do PI( b_T_a * 3dpts ) and plot these
    //--- points on B. Also plot detected points of B
    //---------------------------------------------
    #if 1
    MatrixXd _3dpts_of_A_projectedonB = MatrixXd::Zero(3, world_point.size() );
    MatrixXd _detectedpts_of_B = MatrixXd::Zero(3, world_point.size() );
    MatrixXd _detectedpts_of_A = MatrixXd::Zero(3, world_point.size() );

    int n_good = 0;
    for( int i=0 ; i<world_point.size() ; i++ ) {
        Vector4d a_X_i;
        a_X_i << Vector3d(world_point[i]), 1.0;
        Vector4d b_X_i = b_T_a * a_X_i;
        Vector3d _X_i = b_X_i.topRows(3);
        _X_i /= _X_i(2); // Z-division for projection
        _3dpts_of_A_projectedonB.col(i) = stereogeom->get_K() * _X_i; //< scaling with camera-intrinsic matrix


        Vector3d _tmp;
        _tmp << feature_position_ud[i], 1.0 ;
        _detectedpts_of_B.col(i) = stereogeom->get_K() * _tmp;

        _tmp << feature_position_u[i], 1.0 ;
        _detectedpts_of_A.col(i) = stereogeom->get_K() * _tmp;


        Vector3d delta = _3dpts_of_A_projectedonB.col(i) - _detectedpts_of_B.col(i);
        if( abs(delta(0)) < 2. && abs(delta(1)) < 2. ) {
            // cout << "  delta=" << delta.transpose();
            n_good++;
        }
        else {
            // cout << TermColor::RED() << "delta=" << delta.transpose() << TermColor::RESET();
        }
        // cout << endl;
    }
    cout << "n_good=" <<n_good << endl;
    #endif


    //------------------------------
    //--- Use relative pose of obometry to do PI( odom_b_T_a * 3dpts ) and plot these on image-b
    //--- load odometry poses
    //-------------------------------
    #if 1
    MatrixXd _3dpts_of_A_projectedonB_with_odom_rel_pose = MatrixXd::Zero(3, world_point.size() );
    Matrix4d wTA;
    RawFileIO::read_eigen_matrix( BASE+"/"+std::to_string(frame_a)+".wTc", wTA );
    // cout << "wTa(odom)" << PoseManipUtils::prettyprintMatrix4d( wTa ) << endl;
    Matrix4d wTB;
    RawFileIO::read_eigen_matrix( BASE+"/"+std::to_string(frame_b)+".wTc", wTB );
    // cout << "wTb(odom)" << PoseManipUtils::prettyprintMatrix4d( wTb ) << endl;
    Matrix4d odom_b_T_a = wTB.inverse() * wTA;
    cout << "odom_b_T_a" << PoseManipUtils::prettyprintMatrix4d( odom_b_T_a ) << endl;

    // PI( odom_b_T_a * a_X )
    for( int i=0 ; i<world_point.size() ; i++ ) {
        Vector4d a_X_i;
        a_X_i << Vector3d(world_point[i]), 1.0;
        Vector4d b_X_i = odom_b_T_a * a_X_i;
        Vector3d _X_i = b_X_i.topRows(3);
        _X_i /= _X_i(2); // Z-division for projection
        _3dpts_of_A_projectedonB_with_odom_rel_pose.col(i) = stereogeom->get_K() * _X_i; //< scaling with camera-intrinsic matrix
    }

    #endif


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


bool self_projection_test( std::shared_ptr<StereoGeometry> stereogeom,
                        int frame_a, int frame_b )
{
    // I hacve a doubt that the 3dImage which is created from stereo geometry
    // gives out the 3d points in co-ordinate system of right camera-center.
    // I am wanting it in left camera center co-ordinates. Just reproject those
    // points and ensure my premise.

    //------------------------------------------------
    //------ 3d points from frame_a
    //------------------------------------------------
    cout << TermColor::iGREEN() << "LOAD: BASE/"+std::to_string(frame_a)+".jpg" << TermColor::RESET() << endl;
    cv::Mat a_imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_a)+".jpg", 0 );
    cv::Mat a_imright_raw = cv::imread( BASE+"/"+std::to_string(frame_a)+"_1.jpg", 0 );
    Matrix4d wTA;
    RawFileIO::read_eigen_matrix( BASE+"/"+std::to_string(frame_a)+".wTc", wTA );
    if( a_imright_raw.empty() || a_imright_raw.empty() )
    {
        cout << TermColor::RED() << "Cannot read image with idx=" << frame_a << " perhaps filenotfound" << TermColor::RESET() << endl;
        return false;
    }


    cv::Mat a_imleft_srectified, a_imright_srectified;
    cv::Mat _3dImage;
    MatrixXd _3dpts;
    cv::Mat disp_viz;
    stereogeom->get_srectifiedim_and_3dpoints_and_3dmap_and_disparity_from_raw_images( a_imleft_raw, a_imright_raw,
        a_imleft_srectified,a_imright_srectified,  _3dpts, _3dImage, disp_viz );



    //-----------------------------------------------------
    //--------------- stereo rectify b
    //-----------------------------------------------------
    cout << TermColor::iGREEN() << "LOAD: BASE/"+std::to_string(frame_b)+".jpg" << TermColor::RESET() << endl;
    cv::Mat b_imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_b)+".jpg", 0 );
    cv::Mat b_imright_raw = cv::imread( BASE+"/"+std::to_string(frame_b)+"_1.jpg", 0 );
    Matrix4d wTB;
    RawFileIO::read_eigen_matrix( BASE+"/"+std::to_string(frame_b)+".wTc", wTB );
    if( b_imleft_raw.empty() || b_imright_raw.empty() )
    {
        cout << TermColor::RED() << "Cannot read image with idx=" << frame_b << " perhaps filenotfound" << TermColor::RESET() << endl;
        return false;
    }
    cv::Mat b_imleft_srectified, b_imright_srectified;
    stereogeom->do_stereo_rectification_of_raw_images( b_imleft_raw, b_imright_raw,
                            b_imleft_srectified, b_imright_srectified );


    //---------------------------------------------------------------------
    //------------ point matches between a_left, b_left
    //---------------------------------------------------------------------
    MatrixXd u, ud; // u is from frame_a; ud is from frame_b
    point_feature_matches(a_imleft_srectified, b_imleft_srectified, u, ud );
    if( u.cols() < 30 ) {
        cout << TermColor::RED() << "too few gms matches between " << frame_a << " and " << frame_b << TermColor::RESET() << endl;
        return false;
    }


    //----------------------------------------------------------------------
    //---------------- make collection of 3d 2d points
    //----------------------------------------------------------------------
    int c = 0;
    MatrixXd u_normalized = stereogeom->get_K().inverse() * u;
    MatrixXd ud_normalized = stereogeom->get_K().inverse() * ud;
    std::vector<Eigen::Vector3d> world_point;
    std::vector<Eigen::Vector2d> feature_position_ud;
    std::vector<Eigen::Vector2d> feature_position_u;
    for( int k=0 ; k<u.cols() ; k++ )
    {
        cv::Vec3f _3dpt = _3dImage.at<cv::Vec3f>( (int)u(1,k), (int)u(0,k) );
        if( _3dpt[2] < 0.1 || _3dpt[2] > 25.  )
            continue;

        // the 3d point should also project to same 2d points.


        c++;
        #if 0
        cout << TermColor::RED() << "---" << k << "---" << TermColor::RESET() << endl;
        cout << "ud=" << ud.col(k).transpose() ;
        cout << " <--> ";
        cout << "u=" << u.col(k).transpose() ;
        cout << "  3dpt of u=";
        cout <<  TermColor::GREEN() << _3dpt[0] << " " << _3dpt[1] << " " << _3dpt[2] << " " << TermColor::RESET();

        // project 3d point with Identity
        Vector3d X_i = Vector3d( (double) _3dpt[0],(double) _3dpt[1],(double) _3dpt[2] );
        X_i(0) /= X_i(2);
        X_i(1) /= X_i(2);
        X_i(2) /= X_i(2);
        Vector3d u_cap = stereogeom->get_K() * X_i;
        cout << "\nPI(3dpt)=" << u_cap.transpose() ;

        auto delta = u_cap - u.col(k);
        if( abs(delta(0))  < 2. && abs(delta(1)) < 2. ) {
            cout << TermColor::GREEN() << "\tdelta=" << delta.transpose() << TermColor::RESET();
            make_n_good++;
        }
        else
            cout << "\tdelta=" << delta.transpose() ;

        cout << endl;
        #endif

        feature_position_ud.push_back( Vector2d( ud_normalized(0,k), ud_normalized(1,k) ) );
        feature_position_u.push_back( Vector2d( u_normalized(0,k), u_normalized(1,k) ) );

        world_point.push_back( Vector3d( _3dpt[0], _3dpt[1], _3dpt[2] ) );
        // world_point.push_back( X_i );
    }
    cout << TermColor::GREEN() << "of the total " << u.cols() << " point feature correspondences " << c << " had valid depths" << TermColor::RESET() << endl;


    #if 0
    // PI( _3dpts )
    cout << "_3dpts.shape=" << _3dpts.rows() << "," << _3dpts.cols() << endl;
    MatrixXd reporj = MatrixXd::Zero( 3, _3dpts.cols() );
    vector<cv::Scalar> reporj_falsedepthcolors;
    auto fp_map = FalseColors();
    for( int i=0 ; i<_3dpts.cols() ; i++ ) {
        Vector4d a_X = _3dpts.col(i); //< 3d pt
        reporj_falsedepthcolors.push_back( fp_map.getFalseColor(a_X(2)/10.) );
        reporj.col(i) = stereogeom->get_K() * (a_X.topRows(3) / a_X(2)); // u <== PI( a_X )
    }

    cv::Mat __dst;
    // MiscUtils::plot_point_sets( a_imleft_raw, reporj, __dst, reporj_falsedepthcolors, 0.5, "PI(_3dpts) on a_imleft_raw" );
    MiscUtils::plot_point_sets( a_imright_raw, reporj, __dst, reporj_falsedepthcolors, 0.5, "PI(_3dpts) on a_imright_raw" );
    cv::imshow( "__dst", __dst );

    cv::imshow( "strip", fp_map.getStrip(30, 300) );
    cv::waitKey(0);
    #endif



    #if 0
    // PI(world_point) - OK
    MatrixXd reproj_of_worldpts_on_left_of_A = MatrixXd::Zero(3,world_point.size());
    for( int i=0 ; i<world_point.size() ; i++ ) {
        reproj_of_worldpts_on_left_of_A.col(i) = stereogeom->get_K() * (world_point[i] / world_point[i](2) );
    }

    cv::Mat __dst;
    MiscUtils::plot_point_sets( a_imleft_raw, reproj_of_worldpts_on_left_of_A, __dst, cv::Scalar(0,0,255), false, "reproj_of_worldpts_on_left_of_A(red)" );
    MiscUtils::plot_point_sets( __dst, u, cv::Scalar(0,255,0), false, ";u plotted on A (in green)" );
    cv::imshow( "__dst", __dst );
    cv::waitKey(0);
    #endif


    Matrix4d odom_b_T_a = wTB.inverse() * wTA ;

    Matrix3d rm_R1, rm_R2;
    cv::cv2eigen( stereogeom->get_rm_R1(), rm_R1 );
    cv::cv2eigen( stereogeom->get_rm_R2(), rm_R2 );
    Matrix4d rm_T1, rm_T2;
    rm_T1 = Matrix4d::Identity();
    rm_T2 = Matrix4d::Identity();
    rm_T1.topLeftCorner<3,3>() = rm_R1;
    rm_T2.topLeftCorner<3,3>() = rm_R2;
    cout << "odom_b_T_a" << PoseManipUtils::prettyprintMatrix4d( odom_b_T_a ) << endl;
    cout << "rm_T1" << PoseManipUtils::prettyprintMatrix4d( rm_T1 ) << endl;
    cout << "rm_T2" << PoseManipUtils::prettyprintMatrix4d( rm_T2 ) << endl;

    cout << "get_rm_R1\n" << stereogeom->get_rm_R1() << endl;
    cout << "get_rm_R1\n" << rm_R1 << endl;
    cout << "---\n";
    cout << "get_rm_R2\n" << stereogeom->get_rm_R2() << endl;
    cout << "get_rm_R2\n" << rm_R2 << endl;


    MatrixXd reproj_of_worldpts_on_srectifiedleft_of_B = MatrixXd::Zero(3,world_point.size());
    MatrixXd reproj_of_worldpts_on_srectifiedleft_of_B__1 = MatrixXd::Zero(3,world_point.size());

    for( int i=0 ; i<world_point.size() ; i++ ) {
        Vector4d ad_X;
        ad_X << world_point[i] , 1.0;
        Vector4d bd_X = rm_T2.inverse() * odom_b_T_a * rm_T1 * ad_X;
        // Vector4d bd_X = odom_b_T_a * ad_X;
        // Vector4d bd_X = rm_T1.inverse() * odom_b_T_a * ad_X;
        reproj_of_worldpts_on_srectifiedleft_of_B.col(i) =
            stereogeom->get_K() * (bd_X.topRows(3) / bd_X(2) );


        Vector4d bd_X_1 = (rm_T2.inverse() * (odom_b_T_a * rm_T1)) * ad_X;
        reproj_of_worldpts_on_srectifiedleft_of_B__1.col(i) =
            stereogeom->get_K() * (bd_X_1.topRows(3) / bd_X_1(2) );
    }
    cv::Mat __dst;
    MiscUtils::plot_point_sets( b_imleft_srectified, ud, __dst, cv::Scalar(0,255,0), false, "ud plotted on B srectified_left (in green)" );
    MiscUtils::plot_point_sets( __dst, reproj_of_worldpts_on_srectifiedleft_of_B, cv::Scalar(0,0,255), false, ";reproj_of_worldpts_on_srectifiedleft_of_B with odom_b_T_a(red)" );
    MiscUtils::plot_point_sets( __dst, reproj_of_worldpts_on_srectifiedleft_of_B__1, cv::Scalar(0,128,255), false, ";;reproj_of_worldpts_on_srectifiedleft_of_B__1 with odom_b_T_a(yellow)" );
    cv::imshow( "__dst", __dst );
    cv::waitKey(0);

}

//-----------------------------------------------
//-------- NEW ----------------------------------
//-----------------------------------------------
// Given the point feature matches and the 3d image (from disparity map) will return
// the valid world points and corresponding points.
// [Input]
//      uv: 2xN matrix of point-feature in image-a. In image co-ordinates (not normalized image cords)
//      _3dImage_uv : 3d image from disparity map of image-a. sizeof( _3dImage_uv) === WxHx3
//      uv_d: 2xN matrix of point-feature in image-b. Note that uv<-->uv_d are correspondences so should of equal sizes
// [Output]
//      feature_position_uv : a subset of uv but normalized_image_cordinates
//      feature_position_uv_d : a subset of uv_d. results in normalized_image_cordinates
//      world_point : 3d points of uv.
// [Note]
//      feature_position_uv \subset uv. Selects points which have valid depths.
//      size of output is same for all 3
//      world points are of uv and in co-ordinate system of camera center of uv (or image-a).

bool make_3d_2d_collection__using__pfmatches_and_disparity( std::shared_ptr<StereoGeometry> stereogeom,
            const MatrixXd& uv, const cv::Mat& _3dImage_uv,     const MatrixXd& uv_d,
                            std::vector<Eigen::Vector2d>& feature_position_uv, std::vector<Eigen::Vector2d>& feature_position_uv_d,
                            std::vector<Eigen::Vector3d>& world_point )
{
    assert( (uv.cols() == uv_d.cols() ) && "[Cerebro::make_3d_2d_collection__using__pfmatches_and_disparity] pf-matches need to be of same length. You provided of different lengths\n" );
    assert( _3dImage_uv.type() == CV_32FC3 );

    if( uv.cols() != uv_d.cols() ) {
        cout << TermColor::RED() << "[Cerebro::make_3d_2d_collection__using__pfmatches_and_disparity] pf-matches need to be of same length. You provided of different lengths\n" << TermColor::RESET();
        return false;
    }

    if( _3dImage_uv.type() != CV_32FC3 && _3dImage_uv.rows <= 0 && _3dImage_uv.cols <= 0  ) {
        cout << TermColor::RED() << "[Cerebro::make_3d_2d_collection__using__pfmatches_and_disparity] _3dImage is expected to be of size CV_32FC3\n" << TermColor::RESET();
        return false;
    }



    int c = 0;
    MatrixXd ud_normalized = stereogeom->get_K().inverse() * uv_d;
    MatrixXd u_normalized = stereogeom->get_K().inverse() * uv;
    feature_position_uv.clear();
    feature_position_uv_d.clear();
    world_point.clear();

    for( int k=0 ; k<uv.cols() ; k++ )
    {
        cv::Vec3f _3dpt = _3dImage_uv.at<cv::Vec3f>( (int)uv(1,k), (int)uv(0,k) );
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

        feature_position_uv.push_back( Vector2d( u_normalized(0,k), u_normalized(1,k) ) );
        feature_position_uv_d.push_back( Vector2d( ud_normalized(0,k), ud_normalized(1,k) ) );
        world_point.push_back( Vector3d( _3dpt[0], _3dpt[1], _3dpt[2] ) );
    }

        cout << "[make_3d_2d_collection__using__pfmatches_and_disparity]of the total " << uv.cols() << " point feature correspondences " << c << " had valid depths\n";

    return true;

    if( c < 30 ) {
        cout << TermColor::RED() << "too few valid 3d points between frames" <<  TermColor::RESET() << endl;
        return false;
    }

}

// given pf-matches uv<-->ud_d and their _3dImages. returns the 3d point correspondences at points where it is valid
// uv_X: the 3d points are in frame of ref of camera-uv
// uvd_Y: these 3d points are in frame of ref of camera-uvd
bool make_3d_3d_collection__using__pfmatches_and_disparity(
    const MatrixXd& uv, const cv::Mat& _3dImage_uv,
    const MatrixXd& uv_d, const cv::Mat& _3dImage_uv_d,
    vector<Vector3d>& uv_X, vector<Vector3d>& uvd_Y
)
{
    assert( uv.cols() > 0 && uv.cols() == uv_d.cols() );
    uv_X.clear();
    uvd_Y.clear();

    // similar to above but should return world_point__uv and world_point__uv_d
    int c=0;
    for( int k=0 ; k<uv.cols() ; k++ )
    {
        cv::Vec3f uv_3dpt = _3dImage_uv.at<cv::Vec3f>( (int)uv(1,k), (int)uv(0,k) );
        cv::Vec3f uvd_3dpt = _3dImage_uv_d.at<cv::Vec3f>( (int)uv_d(1,k), (int)uv_d(0,k) );

        if( uv_3dpt[2] < 0.1 || uv_3dpt[2] > 25. || uvd_3dpt[2] < 0.1 || uvd_3dpt[2] > 25.  )
            continue;

        uv_X.push_back( Vector3d( uv_3dpt[0], uv_3dpt[1], uv_3dpt[2] ) );
        uvd_Y.push_back( Vector3d( uvd_3dpt[0], uvd_3dpt[1], uvd_3dpt[2] ) );
        c++;

    }
    cout << "[make_3d_3d_collection__using__pfmatches_and_disparity] of the total " << uv.cols() << " point feature correspondences " << c << " had valid depths\n";

}


// Theia's ICP
// [Input]
//      uv_X: a 3d point cloud expressed in some frame-of-ref, call it frame-of-ref of `uv`
//      uvd_Y: a 3d point cloud expressed in another frame-of-ref, call it frame-of-ref of `uvd`
// [Output]
//      uvd_T_uv: Relative pose between the point clouds
// [Note]
//      uv_X <---> uvd_Y
#define ____P3P_ICP_( msg ) msg;
// #define ____P3P_ICP_( msg ) ;
float P3P_ICP( const vector<Vector3d>& uv_X, const vector<Vector3d>& uvd_Y,
    Matrix4d& uvd_T_uv, string & p3p__msg )
{

    // call this theia::AlignPointCloudsUmeyamaWithWeights
    // void AlignPointCloudsUmeyamaWithWeights(
    //     const std::vector<Eigen::Vector3d>& left,
    //     const std::vector<Eigen::Vector3d>& right,
    //     const std::vector<double>& weights, Eigen::Matrix3d* rotation,
    //     Eigen::Vector3d* translation, double* scale)
    p3p__msg = "";
    Matrix3d ____R;
    Vector3d ____t; double ___s=1.0;

    ElapsedTime timer;
    timer.tic();

    #if 0
    theia::AlignPointCloudsUmeyama( uv_X, uvd_Y, &____R, &____t, &___s ); // all weights = 1. TODO: ideally weights could be proportion to point's Z.
    // theia::AlignPointCloudsICP( uv_X, uvd_Y, &____R, &____t ); // all weights = 1. TODO: ideally weights could be proportion to point's Z.

    double elapsed_time_p3p = timer.toc_milli();

    ____P3P_ICP_(
    cout << "___R:\n" << ____R << endl;
    cout << "___t: " << ____t.transpose() << endl;
    cout << "___s: " << ___s << endl;
    )

    uvd_T_uv = Matrix4d::Identity();
    uvd_T_uv.topLeftCorner<3,3>() = ____R;
    uvd_T_uv.col(3).topRows(3) = ____t;

    ____P3P_ICP_(
    cout << TermColor::GREEN() << "p3p done in (ms) " << elapsed_time_p3p << "  p3p_ICP: {uvd}_T_{uv} : " << PoseManipUtils::prettyprintMatrix4d( uvd_T_uv ) << TermColor::RESET() << endl;
    )



    if( min( ___s, 1.0/___s ) < 0.9 ) {
        ____P3P_ICP_( cout << TermColor::RED() << "theia::AlignPointCloudsUmeyama scales doesn't look good;this usually implies that estimation is bad.        scale= " << ___s << TermColor::RESET() <<  endl; )
        p3p__msg += "p3p_ICP: scale=" +to_string( ___s )+" {uvd}_T_{uv} : " + PoseManipUtils::prettyprintMatrix4d( uvd_T_uv );
        p3p__msg += "p3p done in (ms)" + to_string(elapsed_time_p3p)+";    theia::AlignPointCloudsUmeyama scales doesn't look good, this usually implies that estimation is bad. scale= " + to_string(___s);
        // return -1;
    }

    p3p__msg += "p3p done in (ms)" + to_string(elapsed_time_p3p)+";    p3p_ICP: {uvd}_T_{uv} : " + PoseManipUtils::prettyprintMatrix4d( uvd_T_uv );
    p3p__msg += ";weight="+to_string( min( ___s, 1.0/___s ) );
    // return  min( ___s, 1.0/___s );
    #endif


    // with ransac
    timer.tic();
    vector<CorrespondencePair_3d3d> data_r;
    for( int i=0 ; i<uv_X.size() ; i++ )
    {
        CorrespondencePair_3d3d _data;
        _data.a_X = uv_X[i];
        _data.b_X = uvd_Y[i];
        data_r.push_back( _data );
    }

    AlignPointCloudsUmeyamaWithRansac icp_estimator;
    RelativePose best_rel_pose;

    // set ransac params
    theia::RansacParameters params;
    params.error_thresh = 0.1;
    params.min_inlier_ratio = 0.7;
    params.max_iterations = 50;
    params.min_iterations = 5;
    params.use_mle = true;

    theia::Ransac<AlignPointCloudsUmeyamaWithRansac> ransac_estimator( params, icp_estimator);
    ransac_estimator.Initialize();

    theia::RansacSummary summary;
    ransac_estimator.Estimate(data_r, &best_rel_pose, &summary);
    auto elapsed_time_p3p_ransac = timer.toc_milli();

    uvd_T_uv = Matrix4d::Identity();
    uvd_T_uv =  best_rel_pose.b_T_a ;


    ____P3P_ICP_(
    cout << TermColor::iGREEN() << "ICP Ransac:";
    // for( int i=0; i<summary.inliers.size() ; i++ )
        // cout << "\t" << i<<":"<< summary.inliers[i];
    cout << "\tnum_iterations=" << summary.num_iterations;
    cout << "\tconfidence=" << summary.confidence;
    cout << endl;
    cout << "best solution (ransac icp ) : "<< PoseManipUtils::prettyprintMatrix4d( best_rel_pose.b_T_a ) << TermColor::RESET() << endl;
    )
    p3p__msg += "ICP Ransac;";
    p3p__msg += "    best solution (b_T_a) : "+ PoseManipUtils::prettyprintMatrix4d( best_rel_pose.b_T_a ) + ";";
    p3p__msg += "     #iterations="+to_string( summary.num_iterations );
    p3p__msg += "    confidence="+to_string( summary.confidence ) ;
    p3p__msg += "    icp_ransac_elapsetime_ms="+to_string( elapsed_time_p3p_ransac ) +";";
    return summary.confidence;

}

// Computation of pose given 3d points and imaged points
// [Input]
//      w_X: 3d points in co-ordinate system `w`
//      c_uv_normalized: normalized image co-ordinates of camera c. These are projections of w_X on the camera c
// [Output]
//      c_T_w: pose of the camera is actually the inverse of this matrix. Becareful.
// #define ___P_N_P__( msg ) msg;
#define ___P_N_P__( msg ) ;
float PNP( const std::vector<Vector3d>& w_X, const std::vector<Vector2d>& c_uv_normalized,
    Matrix4d& c_T_w,
    string& pnp__msg )
{
    pnp__msg = "";
    ElapsedTime timer;
    #if 0
    //--- DlsPnp
    std::vector<Eigen::Quaterniond> solution_rotations;
    std::vector<Eigen::Vector3d> solution_translations;
    timer.tic();
    theia::DlsPnp( c_uv_normalized, w_X, &solution_rotations, &solution_translations  );
    auto elapsed_dls_pnp = timer.toc_milli() ;
    ___P_N_P__(
    cout << elapsed_dls_pnp << " : (ms) : theia::DlsPnp done in\n";
    cout << "solutions count = " << solution_rotations.size() << " " << solution_translations.size() << endl;
    )

    if( solution_rotations.size() == 0 ) {
        ___P_N_P__(
        cout << TermColor::RED() << " theia::DlsPnp returns no solution" << TermColor::RESET() << endl;
        )
        pnp__msg = " theia::DlsPnp returns no solution";
        return -1.;
    }

    if( solution_rotations.size() > 1 ) {
        ___P_N_P__(
        cout << TermColor::RED() << " theia::DlsPnp returns multiple solution" << TermColor::RESET() << endl;
        )
        pnp__msg =  " theia::DlsPnp returns multiple solution";
        return -1.;
    }

    // retrive solution
    c_T_w = Matrix4d::Identity();
    c_T_w.topLeftCorner(3,3) = solution_rotations[0].toRotationMatrix();
    c_T_w.col(3).topRows(3) = solution_translations[0];

    ___P_N_P__(
    // cout << "solution_T " << b_T_a << endl;
    cout << TermColor::GREEN() << "DlsPnp (c_T_w): " << PoseManipUtils::prettyprintMatrix4d( c_T_w ) << TermColor::RESET() << endl;
    // out_b_T_a = b_T_a;
    )

    pnp__msg +=  "DlsPnp (c_T_w): " + PoseManipUtils::prettyprintMatrix4d( c_T_w ) + ";";
    pnp__msg += "  elapsed_dls_pnp_ms="+to_string(elapsed_dls_pnp)+";";

    // return 1.0;
    #endif


    #if 1
    //--- DlsPnpWithRansac
    timer.tic();
    // prep data
    vector<CorrespondencePair_3d2d> data_r;
    for( int i=0 ; i<w_X.size() ; i++ )
    {
        CorrespondencePair_3d2d _data;
        _data.a_X = w_X[i];
        _data.uv_d = c_uv_normalized[i];
        data_r.push_back( _data );
    }

    // Specify RANSAC parameters.

    DlsPnpWithRansac dlspnp_estimator;
    RelativePose best_rel_pose;

    // Set the ransac parameters.
    theia::RansacParameters params;
    params.error_thresh = 0.02;
    params.min_inlier_ratio = 0.7;
    params.max_iterations = 50;
    params.min_iterations = 5;
    params.use_mle = true;

    // Create Ransac object, specifying the number of points to sample to
    // generate a model estimation.
    theia::Ransac<DlsPnpWithRansac> ransac_estimator(params, dlspnp_estimator);
    // Initialize must always be called!
    ransac_estimator.Initialize();

    theia::RansacSummary summary;
    ransac_estimator.Estimate(data_r, &best_rel_pose, &summary);

    auto elapsed_dls_pnp_ransac=timer.toc_milli();
    ___P_N_P__(
    cout << elapsed_dls_pnp_ransac << "(ms)!! DlsPnpWithRansac ElapsedTime includes data prep\n";
    cout << "Ransac:";
    // for( int i=0; i<summary.inliers.size() ; i++ )
        // cout << "\t" << i<<":"<< summary.inliers[i];
    cout << "\tnum_iterations=" << summary.num_iterations;
    cout << "\tconfidence=" << summary.confidence;
    cout << endl;
    cout << TermColor::GREEN() << "best solution (ransac) : "<< PoseManipUtils::prettyprintMatrix4d( best_rel_pose.b_T_a ) << TermColor::RESET() << endl;
    )
    pnp__msg +=  "DlsPnpWithRansac (best_rel_pose.b_T_a): " + PoseManipUtils::prettyprintMatrix4d( best_rel_pose.b_T_a ) + ";";
    pnp__msg += string("    num_iterations=")+to_string(summary.num_iterations)+"  confidence="+to_string(summary.confidence);
    pnp__msg += string( "   elapsed_dls_pnp_ransac (ms)=")+to_string(elapsed_dls_pnp_ransac)+";";


    c_T_w = best_rel_pose.b_T_a;
    return summary.confidence;
    #endif



}

bool verified_alignment( std::shared_ptr<StereoGeometry> stereogeom,
                        int frame_a, int frame_b )
{
    // I hacve a doubt that the 3dImage which is created from stereo geometry
    // gives out the 3d points in co-ordinate system of right camera-center.
    // I am wanting it in left camera center co-ordinates. Just reproject those
    // points and ensure my premise.

    //------------------------------------------------
    //------ 3d points from frame_a
    //------------------------------------------------
    cout << TermColor::iGREEN() << "LOAD: BASE/"+std::to_string(frame_a)+".jpg" << TermColor::RESET() << endl;
    cv::Mat a_imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_a)+".jpg", 0 );
    cv::Mat a_imright_raw = cv::imread( BASE+"/"+std::to_string(frame_a)+"_1.jpg", 0 );
    Matrix4d wTA;
    RawFileIO::read_eigen_matrix( BASE+"/"+std::to_string(frame_a)+".wTc", wTA );
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
    //--------------- stereo rectify b
    //-----------------------------------------------------
    cout << TermColor::iGREEN() << "LOAD: BASE/"+std::to_string(frame_b)+".jpg" << TermColor::RESET() << endl;
    cv::Mat b_imleft_raw =  cv::imread( BASE+"/"+std::to_string(frame_b)+".jpg", 0 );
    cv::Mat b_imright_raw = cv::imread( BASE+"/"+std::to_string(frame_b)+"_1.jpg", 0 );
    Matrix4d wTB;
    RawFileIO::read_eigen_matrix( BASE+"/"+std::to_string(frame_b)+".wTc", wTB );
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


    //---------------------------------------------------------------------
    //------------ point matches between a_left, b_left
    //---------------------------------------------------------------------
    MatrixXd uv, uv_d; // u is from frame_a; ud is from frame_b
    ElapsedTime timer;
    timer.tic();
    point_feature_matches(a_imleft_srectified, b_imleft_srectified, uv, uv_d );
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

    #if 1  // disparty maps of both images
    cv::Mat dst_disp;
    MiscUtils::side_by_side( a_disp_viz, b_disp_viz, dst_disp );
    MiscUtils::append_status_image( dst_disp, "a="+ to_string(frame_a)+"     b="+to_string(frame_b), .8 );
    cv::resize(dst_disp, dst_disp, cv::Size(), 0.5, 0.5 );
    #endif

    //
    // Collect 3d points
    //

    // Option-A:
    cout << TermColor::BLUE() << "Option-A" << TermColor::RESET() << endl;
    std::vector<Eigen::Vector2d> feature_position_uv;
    std::vector<Eigen::Vector2d> feature_position_uv_d;
    std::vector<Eigen::Vector3d> world_point_uv;
    make_3d_2d_collection__using__pfmatches_and_disparity( stereogeom, uv, a_3dImage, uv_d,
                                feature_position_uv, feature_position_uv_d, world_point_uv);
    Matrix4d op1__b_T_a; string pnp__msg;
    float pnp_goodness = PNP( world_point_uv, feature_position_uv_d, op1__b_T_a, pnp__msg  );
    cout << TermColor::YELLOW() << pnp_goodness << " op1__b_T_a = " << PoseManipUtils::prettyprintMatrix4d( op1__b_T_a ) << TermColor::RESET() << endl;
    cout << pnp__msg << endl;

    // Option-B:
    cout << TermColor::BLUE() << "Option-B" << TermColor::RESET() << endl;
    std::vector<Eigen::Vector3d> world_point_uv_d;
    make_3d_2d_collection__using__pfmatches_and_disparity( stereogeom, uv_d, b_3dImage, uv,
                                feature_position_uv_d, feature_position_uv, world_point_uv_d);
    Matrix4d op2__a_T_b, op2__b_T_a; string pnp__msg_option_B;
    float pnp_goodness_optioN_B = PNP( world_point_uv_d, feature_position_uv, op2__a_T_b, pnp__msg_option_B  );
    op2__b_T_a = op2__a_T_b.inverse();
    cout << TermColor::YELLOW() << pnp_goodness_optioN_B << " op2__a_T_b = " << PoseManipUtils::prettyprintMatrix4d( op2__a_T_b ) << TermColor::RESET() << endl;
    cout << TermColor::YELLOW() << pnp_goodness_optioN_B << " op2__b_T_a = " << PoseManipUtils::prettyprintMatrix4d( op2__b_T_a ) << TermColor::RESET() << endl;
    cout << pnp__msg_option_B << endl;

    // Option-C
    cout << TermColor::BLUE() << "Option-C" << TermColor::RESET() << endl;
    vector< Vector3d> uv_X;
    vector< Vector3d> uvd_Y;
    make_3d_3d_collection__using__pfmatches_and_disparity( uv, a_3dImage, uv_d, b_3dImage,
        uv_X, uvd_Y );
    Matrix4d icp_b_T_a; string p3p__msg;
    float p3p_goodness = P3P_ICP( uv_X, uvd_Y, icp_b_T_a, p3p__msg );
    cout << TermColor::YELLOW() << p3p_goodness << " icp_b_T_a = " << PoseManipUtils::prettyprintMatrix4d( icp_b_T_a ) << TermColor::RESET() << endl;
    cout << p3p__msg << endl;


    #if 1
    MiscUtils::append_status_image( dst_disp, "[b_T_a <-- PNP( 3d(a), uv(b))];"+pnp__msg+";[a_T_b <-- PNP( 3d(b), uv(a))];"+pnp__msg_option_B+";[icp_b_T_a <-- ICP(3d(a), 3d(b))];"+p3p__msg );
    cv::imshow( "dst_disp", dst_disp );
    #endif



    Matrix4d odom_b_T_a = wTB.inverse() * wTA;
    cout << TermColor::YELLOW() << "odom_b_T_a = " << PoseManipUtils::prettyprintMatrix4d( odom_b_T_a ) << TermColor::RESET() << endl;


    cout << "|op1 - op2|" << PoseManipUtils::prettyprintMatrix4d( op1__b_T_a.inverse() * op2__b_T_a ) << endl;
    cout << "|op1 - icp|" << PoseManipUtils::prettyprintMatrix4d( op1__b_T_a.inverse() * icp_b_T_a ) << endl;
    cout << "|op2 - icp|" << PoseManipUtils::prettyprintMatrix4d( op2__b_T_a.inverse() * icp_b_T_a ) << endl;


}

int main0( int argc, char ** argv )
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
        bool status = verified_alignment( stereogeom, a, b );
        cv::waitKey(0);

    }
}

int main(int argc, char ** argv)
{
    std::shared_ptr<StereoGeometry> stereogeom = make_stereo_geom();
    // cout << "exit main\n";
    // return 0;

    // will return false if cannot compute pose

    if( argc != 3 ) {
        cout << "[MAIN::INVALID USAGE] I am expecting you to give two numbers in the command line between which you want to compute the pose\n";
        exit(1);
    }
    Matrix4d out_b_T_a = Matrix4d::Identity();
    int a = stoi( argv[1] );
    int b = stoi( argv[2] );
    // bool status = relative_pose_compute_with_theia(stereogeom, a, b, out_b_T_a );
    // bool status = self_projection_test( stereogeom, a, b );
    bool status = verified_alignment( stereogeom, a, b );


    // bool status = relative_pose_compute_with_theia(stereogeom, 195, 3, out_b_T_a );
    cv::waitKey(0);

    cout << "####RESULT####\n";
    cout << "status: " << status << endl;
    cout << PoseManipUtils::prettyprintMatrix4d( out_b_T_a ) << endl;


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
