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
    if( matches_gms.size() > 0 )
        MiscUtils::dmatch_2_eigen( kp1, kp2, matches_gms, u, ud, true );


}

void StaticPointFeatureMatching::point_feature_matches(
    const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
    MatrixXd&u, MatrixXd& ud )
{
    PointFeatureMatchingSummary summary;
    point_feature_matches(  imleft_undistorted, imright_undistorted, u, ud, summary );
}

// #define ___StaticPointFeatureMatching__point_feature_matches( msg ) msg;
#define ___StaticPointFeatureMatching__point_feature_matches( msg ) ;
void StaticPointFeatureMatching::point_feature_matches( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
            MatrixXd&u, MatrixXd& ud,
            PointFeatureMatchingSummary& summary )
{
    ___StaticPointFeatureMatching__point_feature_matches(
    cout << TermColor::YELLOW() << "StaticPointFeatureMatching::point_feature_matches\n" << TermColor::RESET();
    cout << "imleft_undistorted: " << MiscUtils::cvmat_info( imleft_undistorted ) << "\n";
    cout << "imright_undistorted: " << MiscUtils::cvmat_info( imright_undistorted ) << "\t";
    cout << endl;
    )
    assert( !imleft_undistorted.empty() && !imright_undistorted.empty() );

    cv::Ptr<cv::Feature2D> fdetector = cv::ORB::create(500);
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    fdetector->detectAndCompute(imleft_undistorted, cv::Mat(), keypoints1, descriptors1);
    fdetector->detectAndCompute(imright_undistorted, cv::Mat(), keypoints2, descriptors2);
    ___StaticPointFeatureMatching__point_feature_matches(
    cout << "# of keypoints : "<< keypoints1.size() << "\t";
    cout << "# of keypoints : "<< keypoints2.size() << "\t";
    cout << "descriptors shape : "<< descriptors1.rows << "x" << descriptors1.cols << "\t";
    cout << "descriptors shape : "<< descriptors2.rows << "x" << descriptors2.cols << endl;
    )

    summary.feature_detector_type = "ORB";
    summary.feature_descriptor_type = "ORB";
    summary.n_keypoints = descriptors1.rows;
    summary.n_descriptor_dimension = descriptors1.cols;


    #if 0
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector< cv::DMatch > matches;
    matcher.match(descriptors1, descriptors2, matches);
    ___StaticPointFeatureMatching__point_feature_matches(
    cout << "#number of matches : " << matches.size() << endl; )

    // MatrixXd M1, M2;
    if( matches.size() > 0 )
        MiscUtils::dmatch_2_eigen( keypoints1, keypoints2, matches, u, ud, true );

    #endif

    //
    // Matcher - FLAN (Approx NN)
    ___StaticPointFeatureMatching__point_feature_matches( cout << "FLANN based matcher\n"; )
    if(descriptors1.type()!=CV_32F)
    {
        ___StaticPointFeatureMatching__point_feature_matches(
        cout << "descriptors1 is not of type, CV_32F, so convert\n";)
        descriptors1.convertTo(descriptors1, CV_32F);
        descriptors2.convertTo(descriptors2, CV_32F);
    }
    cv::FlannBasedMatcher matcher;
    summary.matcher_type = "FlannBasedMatcher";

    ___StaticPointFeatureMatching__point_feature_matches( cout << "matcher.knnMatch()\n"; )
    std::vector< std::vector< cv::DMatch > > matches_raw;
    matcher.knnMatch( descriptors1, descriptors2, matches_raw, 2 );
    ___StaticPointFeatureMatching__point_feature_matches(
    cout << "# Matches : " << matches_raw.size() << "\t"; //N
    cout << "# Matches[0] : " << matches_raw[0].size() << endl; //2
    )

    //
    // Lowe's Ratio test
    vector<cv::Point2f> pts_1, pts_2;
    lowe_ratio_test( keypoints1, keypoints2, matches_raw, pts_1, pts_2 );
    ___StaticPointFeatureMatching__point_feature_matches(
    cout << "# Retained (after lowe_ratio_test): "<< pts_1.size() << endl; // == pts_2.size()
    )
    summary.n_keypoints_pass_ratio_test = pts_1.size();


    #if 0
    //
    bool make_homogeneous = true;
    u = MatrixXd::Constant( (make_homogeneous?3:2), pts_1.size(), 1.0 );
    ud = MatrixXd::Constant( (make_homogeneous?3:2), pts_2.size(), 1.0 );
    for( int k=0 ; k<pts_1.size() ; k++ )
    {
        u(0,k) = pts_1[k].x;
        u(1,k) = pts_1[k].y;

        ud(0,k) = pts_2[k].x;
        ud(1,k) = pts_2[k].y;
    }
    #endif

    // F-test
    vector<uchar> status;
    ___StaticPointFeatureMatching__point_feature_matches( cout << "compute cv::findFundamentalMat()\n";
    cout << "pts_1.size=" << pts_1.size() << "\tpts_2.size=" << pts_2.size() << endl;
    // for( int g=0 ; g<pts_1.size() ; g++ )
    // {
    //         cout << g << ":" << pts_1[g] << "\t" << pts_2[g] << endl;
    // }
    )
    cv::findFundamentalMat(pts_1, pts_2, cv::FM_RANSAC, 2.9, 0.99, status);
    // cv::findFundamentalMat(pts_1, pts_2, cv::FM_LMEDS, 3.0, 0.99, status);
    ___StaticPointFeatureMatching__point_feature_matches( cout << "done..., pts_2.size="<< pts_2.size() << ", status.size=" << status.size() << " \n"; )
    assert( pts_2.size() == status.size() );
    int n_good = 0;
    for( int k=0 ;k<status.size();k++) {
        if( status[k] > 0 )
            n_good++;
        }
    ___StaticPointFeatureMatching__point_feature_matches(
        cout << "....Only " << n_good << " points pass the F-test\n";)
        summary.n_keypoints_pass_f_test = n_good;

        //
        bool make_homogeneous = true;
        u = MatrixXd::Constant( (make_homogeneous?3:2), n_good, 1.0 );
        ud = MatrixXd::Constant( (make_homogeneous?3:2), n_good, 1.0 );
        int ck = 0;
        for( int k=0 ; k<pts_1.size() ; k++ )
        {
            if( status[k] > 0 ) {
                u(0,ck) = pts_1[k].x;
                u(1,ck) = pts_1[k].y;

                ud(0,ck) = pts_2[k].x;
                ud(1,ck) = pts_2[k].y;
                ck++;
            }
        }
        assert( ck == n_good );



    ___StaticPointFeatureMatching__point_feature_matches(
    cout << "[StaticPointFeatureMatching::point_feature_matches] Finished\n";)
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



#if 0
// i dont want such a specific function in the library, removal
bool StaticPointFeatureMatching::image_correspondence_with_depth(
    const cv::Mat& image_a, const cv::Mat& depth_a,
    const cv::Mat& image_b, const cv::Mat& depth_b,
    MatrixXd& uv_a, MatrixXd& uv_b,
    MatrixXd& aX, MatrixXd& bX, vector<bool>& valids
)
{
    if( image_a.empty() || image_b.empty() || depth_a.empty() || depth_b.empty() ) {
        cout << TermColor::YELLOW() << "[StaticPointFeatureMatching::image_correspondence_with_depth] WARN, input images where empty...\n";
        return false;
    }

    ElapsedTime elp;

    #if 1
    // -- GMS Matcher
    elp.tic();
    StaticPointFeatureMatching::gms_point_feature_matches( image_a, image_b, uv_a, uv_b );
    cout << TermColor::BLUE() << "StaticPointFeatureMatching::gms_point_feature_matches returned in " << elp.toc_milli() << " ms\n" << TermColor::RESET();

    cout << "uv_a: " << uv_a.rows() << "x" << uv_a.cols() << "\t";
    cout << "uv_b: " << uv_b.rows() << "x" << uv_b.cols() << "\t";
    cout << endl;

    if( uv_a.cols() < 50 ) {
        cout << TermColor::YELLOW() << "\nGMSMatcher produced fewer than 50 point matches, return false\n" << TermColor::RESET();
        return false;
    }

    #if 0
    cv::Mat dst_gmsmatcher;
    MiscUtils::plot_point_pair( image_a, uv_a, idx_a,
                                image_b, uv_b, idx_b, dst_gmsmatcher, 3, "gms plot (resize 0.5)" );
    cv::resize(dst_gmsmatcher, dst_gmsmatcher, cv::Size(), 0.5, 0.5);
    cv::imshow( "GMSMatcher", dst_gmsmatcher );
    #endif
    #endif


    #if 0
    // -- Simple ORB Matcher

    elp.tic();
    PointFeatureMatchingSummary summary;
    StaticPointFeatureMatching::point_feature_matches( image_a, image_b, uv_a, uv_b, summary );
    cout << TermColor::BLUE() << "[StaticPointFeatureMatching::image_correspondence_with_depth] StaticPointFeatureMatching::point_feature_matches returned in " << elp.toc_milli() << " ms\n" << TermColor::RESET();


    cout << "uv_a: " << uv_a.rows() << "x" << uv_a.cols() << "\t";
    cout << "uv_b: " << uv_b.rows() << "x" << uv_b.cols() << "\t";
    cout << endl;

    if( uv_a.cols() < 5 ) {
        cout << TermColor::YELLOW() << "\npoint_feature_matches() produced fewer than 5 point matches, return false\n" << TermColor::RESET();
        return false;
    }

    #if 0
    cv::Mat dst_matcher;
    MiscUtils::plot_point_pair( image_a, uv_a, idx_a,
                                image_b, uv_b, idx_b, dst_matcher,
                                // 3, "gms plot (resize 0.5)"
                                cv::Scalar( 0,0,255 ), cv::Scalar( 0,255,0 ), false, "gms plot (resize 0.5)"
                            );
    cv::resize(dst_matcher, dst_matcher, cv::Size(), 0.5, 0.5);
    cv::imshow( "GMSMatcher", dst_matcher );
    #endif
    #endif

    // --- Depth lookup at correspondences
    #if 0
    // -- 3D Points from depth image at the correspondences
    // vector<bool> valids;
    valids.clear();
    StaticPointFeatureMatching::make_3d_3d_collection__using__pfmatches_and_depthimage(
        xloader.left_camera,
        uv_a, depth_a, uv_b, depth_b,
        aX, bX, valids
    );

    int nvalids = 0;
    for( int i=0 ; i<valids.size() ; i++ )
        if( valids[i]  == true )
            nvalids++;
    cout << TermColor::YELLOW() <<  "nvalids_depths=" << nvalids << " of total=" << valids.size() << TermColor::RESET() << "\t";

    cout << "aX: " << aX.rows() << "x" << aX.cols() << "\t";
    cout << "bX: " << bX.rows() << "x" << bX.cols() << endl;

    if( nvalids < 5 ) {
        cout << TermColor::YELLOW() << "Of the total " << valids.size() << " point-matches, only " << nvalids << " had good depths, this is less than the threshold, so return false\n" << TermColor::RESET();
        return false;
    }
    int nvalids = (int) sf.sum();
    cout << TermColor::YELLOW() << "Of the total " << uv_a.cols() << " point-matches, only " << nvalids << " had good depths\n" << TermColor::RESET();
    #endif

    return true;
}

#endif

bool StaticPointFeatureMatching::make_3d_3d_collection__using__pfmatches_and_depthimage(
    camodocal::CameraPtr _camera,
    const MatrixXd& uv, const cv::Mat& depth_image_uv,
    const MatrixXd& uv_d, const cv::Mat& depth_image_uvd,
    //vector<Vector3d>& uv_X, vector<Vector3d>& uvd_Y
    MatrixXd& uv_X, MatrixXd& uvd_Y, vector<bool>& valids,
    float near , float far
)
{
    // float near = 0.5;
    // float far = 4.5;
    assert( uv.cols() > 0 && uv.cols() == uv_d.cols() );
    // uv_X.clear();
    // uvd_Y.clear();
    valids.clear();
    if( uv.cols() != uv_d.cols() ) {
        cout << TermColor::RED() << "[StaticPointFeatureMatching::make_3d_3d_collection__using__pfmatches_and_depthimage] pf-matches need to be of same length. You provided of different lengths\n" << TermColor::RESET();
        return false;
    }
    assert( depth_image_uv.type() == CV_16UC1 || depth_image_uv.type() == CV_32FC1 );
    assert( depth_image_uv.type() == depth_image_uvd.type() );


    // make 3d points out of 2d point feature matches
    uv_X = MatrixXd::Constant( 4, uv.cols(), 1.0 );
    for( int i=0 ; i<uv.cols() ; i++ )
    {

        float depth_val;
        if( depth_image_uv.type() == CV_16UC1 ) {
            depth_val = .001 * depth_image_uv.at<uint16_t>( uv(1,i), uv(0,i) );
        }
        else if( depth_image_uv.type() == CV_32FC1 ) {
            // just assuming the depth values are in meters when CV_32FC1
            depth_val = depth_image_uv.at<float>( uv(1,i), uv(0,i) );
        }
        else {
            assert( false );
            cout << "[StaticPointFeatureMatching::make_3d_3d_collection__using__pfmatches_and_depthimage]depth type is neighter of CV_16UC1 or CV_32FC1\n";
            exit(1);
        }
        // cout << "ath uv_i=" << i << " depth_val = " << depth_val << endl;

        Vector3d _0P;
        Vector2d _0p; _0p << uv(0,i), uv(1,i);
        _camera->liftProjective( _0p, _0P );
        // uv_X.push_back( depth_val * _0P );
        uv_X.col(i).topRows(3) = depth_val * _0P;

        if( depth_val > near && depth_val < far )
            valids.push_back( true );
        else
            valids.push_back( false );
    }


    uvd_Y = MatrixXd::Constant( 4, uv_d.cols(), 1.0 );
    for( int i=0 ; i<uv_d.cols() ; i++ )
    {

        float depth_val;
        if( depth_image_uvd.type() == CV_16UC1 ) {
            depth_val = 0.001 * depth_image_uvd.at<uint16_t>( uv_d(1,i), uv_d(0,i) );
        }
        else if( depth_image_uvd.type() == CV_32FC1 ) {
            // just assuming the depth values are in meters when CV_32FC1
            depth_val =  depth_image_uvd.at<float>( uv_d(1,i), uv_d(0,i) );
        }
        else {
            assert( false );
            cout << "[StaticPointFeatureMatching::make_3d_3d_collection__using__pfmatches_and_depthimage]depth type is neighter of CV_16UC1 or CV_32FC1\n";
            exit(1);
        }
        // cout << "ath uvd_i=" << i << " depth_val = " << depth_val << endl;

        Vector3d _1P;
        Vector2d _1p; _1p << uv_d(0,i), uv_d(1,i);
        _camera->liftProjective( _1p, _1P );
        // uvd_Y.push_back( depth_val * _1P );
        uvd_Y.col(i).topRows(3) = depth_val * _1P;

        if( depth_val > near && depth_val < far )
            valids[i] = valids[i] && true;
        else
            valids[i] = valids[i] && false;
    }


    return true;


}


void StaticPointFeatureMatching::lowe_ratio_test( const vector<cv::KeyPoint>& keypoints1, const vector<cv::KeyPoint>& keypoints2 ,
                  const std::vector< std::vector< cv::DMatch > >& matches_raw,
                  vector<cv::Point2f>& pts_1, vector<cv::Point2f>& pts_2, float threshold )
{
    pts_1.clear();
    pts_2.clear();
    assert( matches_raw.size() > 0 );
    assert( matches_raw[0].size() >= 2 );

    for( int j=0 ; j<matches_raw.size() ; j++ )
    {
        if( matches_raw[j][0].distance < threshold * matches_raw[j][1].distance ) //good match
        {
            // Get points
            int t = matches_raw[j][0].trainIdx;
            int q = matches_raw[j][0].queryIdx;
            pts_1.push_back( keypoints1[q].pt );
            pts_2.push_back( keypoints2[t].pt );
        }
    }

}
