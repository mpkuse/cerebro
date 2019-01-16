#include "PointFeatureMatching.h"

// #define ___StaticPointFeatureMatching__gms_point_feature_matches( msg ) msg;
#define ___StaticPointFeatureMatching__gms_point_feature_matches( msg ) ;
void StaticPointFeatureMatching::gms_point_feature_matches( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                            MatrixXd& u, MatrixXd& ud, int n_orb_feat )
{
    ElapsedTime timer;


    //
    // Point feature and descriptors extract
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat d1, d2; //< descriptors

    // cv::Ptr<cv::ORB> orb = cv::ORB::create(5000);
    cv::Ptr<cv::ORB> orb = cv::ORB::create(n_orb_feat);
    orb->setFastThreshold(0);

    ___StaticPointFeatureMatching__gms_point_feature_matches(timer.tic();)
    orb->detectAndCompute(imleft_undistorted, cv::Mat(), kp1, d1);
    orb->detectAndCompute(imright_undistorted, cv::Mat(), kp2, d2);
    ___StaticPointFeatureMatching__gms_point_feature_matches(cout <<  timer.toc_milli()  << " (ms) 2X detectAndCompute(ms) : "<< endl;)
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
    ___StaticPointFeatureMatching__gms_point_feature_matches(timer.tic();)
    matcher.match(d1, d2, matches_all);
    ___StaticPointFeatureMatching__gms_point_feature_matches(
    std::cout << timer.toc_milli() << " : (ms) BFMatcher took (ms)\t";
    std::cout << "BFMatcher : npts = " << matches_all.size() << std::endl;
    )


    // gms_matcher
    ___StaticPointFeatureMatching__gms_point_feature_matches(timer.tic();)
    std::vector<bool> vbInliers;
    gms_matcher gms(kp1, imleft_undistorted.size(), kp2, imright_undistorted.size(), matches_all);
    int num_inliers = gms.GetInlierMask(vbInliers, false, false);
    ___StaticPointFeatureMatching__gms_point_feature_matches(
    cout << timer.toc_milli() << " : (ms) GMSMatcher took.\t" ;
    cout << "Got total gms matches " << num_inliers << " matches." << endl;
    )

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

bool StaticPointFeatureMatching::make_3d_2d_collection__using__pfmatches_and_disparity( std::shared_ptr<StereoGeometry> stereogeom,
            const MatrixXd& uv, const cv::Mat& _3dImage_uv,     const MatrixXd& uv_d,
                            std::vector<Eigen::Vector2d>& feature_position_uv, std::vector<Eigen::Vector2d>& feature_position_uv_d,
                            std::vector<Eigen::Vector3d>& world_point )
{
    assert( (uv.cols() == uv_d.cols() ) && "[StaticPointFeatureMatching::make_3d_2d_collection__using__pfmatches_and_disparity] pf-matches need to be of same length. You provided of different lengths\n" );
    assert( _3dImage_uv.type() == CV_32FC3 );

    if( uv.cols() != uv_d.cols() ) {
        cout << TermColor::RED() << "[StaticPointFeatureMatching::make_3d_2d_collection__using__pfmatches_and_disparity] pf-matches need to be of same length. You provided of different lengths\n" << TermColor::RESET();
        return false;
    }

    if( _3dImage_uv.type() != CV_32FC3 && _3dImage_uv.rows <= 0 && _3dImage_uv.cols <= 0  ) {
        cout << TermColor::RED() << "[StaticPointFeatureMatching::make_3d_2d_collection__using__pfmatches_and_disparity] _3dImage is expected to be of size CV_32FC3\n" << TermColor::RESET();
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

    // cout << "[make_3d_2d_collection__using__pfmatches_and_disparity]of the total " << uv.cols() << " point feature correspondences " << c << " had valid depths\n";

    return true;

    if( c < 30 ) {
        cout << TermColor::RED() << "too few valid 3d points between frames" <<  TermColor::RESET() << endl;
        return false;
    }

}

// given pf-matches uv<-->ud_d and their _3dImages. returns the 3d point correspondences at points where it is valid
// uv_X: the 3d points are in frame of ref of camera-uv
// uvd_Y: these 3d points are in frame of ref of camera-uvd
bool StaticPointFeatureMatching::make_3d_3d_collection__using__pfmatches_and_disparity(
    const MatrixXd& uv, const cv::Mat& _3dImage_uv,
    const MatrixXd& uv_d, const cv::Mat& _3dImage_uv_d,
    vector<Vector3d>& uv_X, vector<Vector3d>& uvd_Y
)
{
    assert( uv.cols() > 0 && uv.cols() == uv_d.cols() );
    uv_X.clear();
    uvd_Y.clear();
    if( uv.cols() != uv_d.cols() ) {
        cout << TermColor::RED() << "[StaticPointFeatureMatching::make_3d_3d_collection__using__pfmatches_and_disparity] pf-matches need to be of same length. You provided of different lengths\n" << TermColor::RESET();
        return false;
    }

    if( _3dImage_uv.type() != CV_32FC3 && _3dImage_uv.rows <= 0 && _3dImage_uv.cols <= 0  ) {
        cout << TermColor::RED() << "[StaticPointFeatureMatching::make_3d_3d_collection__using__pfmatches_and_disparity] _3dImage is expected to be of size CV_32FC3\n" << TermColor::RESET();
        return false;
    }


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
    // cout << "[make_3d_3d_collection__using__pfmatches_and_disparity] of the total " << uv.cols() << " point feature correspondences " << c << " had valid depths\n";
    return true;
}
