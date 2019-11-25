#include "PointFeatureMatching.h"



#if 0 // TODO removal. old implementation
// #define ___StaticPointFeatureMatching__gms_point_feature_matches( msg ) msg;
#define ___StaticPointFeatureMatching__gms_point_feature_matches( msg ) ;
void StaticPointFeatureMatching::gms_point_feature_matches( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                            MatrixXd& u, MatrixXd& ud, int n_orb_feat )
{
    ___StaticPointFeatureMatching__gms_point_feature_matches( cout << "in function StaticPointFeatureMatching::gms_point_feature_matches\n";)
    ElapsedTime timer;
    assert( imleft_undistorted.data && imright_undistorted.data );

    if(imleft_undistorted.empty() || imright_undistorted.empty() )
        return;

    //
    // Point feature and descriptors extract
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat d1, d2; //< descriptors

    // cv::Ptr<cv::ORB> orb = cv::ORB::create(5000);
    cv::Ptr<cv::ORB> orb = cv::ORB::create(n_orb_feat);
    orb->setFastThreshold(0);

    ___StaticPointFeatureMatching__gms_point_feature_matches(
        cout << "attempt detectAndCompute\n";
        timer.tic();)
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
    ___StaticPointFeatureMatching__gms_point_feature_matches(
        cout << "Attempt cv::BFMatcher\n";
        timer.tic();
    )
    matcher.match(d1, d2, matches_all);
    ___StaticPointFeatureMatching__gms_point_feature_matches(
    std::cout << timer.toc_milli() << " : (ms) BFMatcher took (ms)\t";
    std::cout << "BFMatcher : npts = " << matches_all.size() << std::endl;
    )


    // gms_matcher
    ___StaticPointFeatureMatching__gms_point_feature_matches(
        cout << "Attept GMS match\n";
        timer.tic();
    )
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
#endif

#if 0 //TODO removal. old implementation
void StaticPointFeatureMatching::gms_point_feature_matches_scaled( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                            MatrixXd& u, MatrixXd& ud,
                            float scale, int n_orb_feat )
{
    assert( imleft_undistorted.data && imright_undistorted.data );

    assert( scale > 0.1 && scale < 0.99 );

    if(imleft_undistorted.empty() || imright_undistorted.empty() )
        return;

    // scale images
    cv::Mat imleft_undistorted_scaled, imright_undistorted_scaled;
    cv::resize(imleft_undistorted, imleft_undistorted_scaled, cv::Size(), scale, scale);
    cv::resize(imright_undistorted, imright_undistorted_scaled, cv::Size(), scale, scale);

    // core matcher
    gms_point_feature_matches( imleft_undistorted_scaled, imright_undistorted_scaled, u, ud, n_orb_feat );

    // un-scale the co-ordinates
    if( u.cols() > 0 && ud.cols() > 0 ) {
        // this will cause seg-fault if no point feature matches were found
        u *= (1.0/scale);
        ud *= (1.0/scale);
    }


}
#endif



// #define ___StaticPointFeatureMatching__gms_point_feature_matches( msg ) msg;
#define ___StaticPointFeatureMatching__gms_point_feature_matches( msg ) ;

// #define _orb_param( msg ) msg;
#define _orb_param( msg ) ;
void StaticPointFeatureMatching::gms_point_feature_matches( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                            MatrixXd& u, MatrixXd& ud, int n_orb_feat )
{
    ElapsedTime timer;


    //
    // Point feature and descriptors extract
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat d1, d2; //< descriptors


    #if 0
    // original code
    ___StaticPointFeatureMatching__gms_point_feature_matches( cout << "n_orb_feat=" << n_orb_feat << endl; )
    // cv::Ptr<cv::ORB> orb = cv::ORB::create(5000);
    cv::Ptr<cv::ORB> orb = cv::ORB::create(n_orb_feat);

    ___StaticPointFeatureMatching__gms_point_feature_matches(timer.tic();)
    orb->detectAndCompute(imleft_undistorted, cv::Mat(), kp1, d1);
    orb->detectAndCompute(imright_undistorted, cv::Mat(), kp2, d2);
    ___StaticPointFeatureMatching__gms_point_feature_matches(cout <<  timer.toc_milli()  << " (ms) 2X detectAndCompute(ms) : "<< endl;)
    // std::cout << "d1 " << MiscUtils::cvmat_info( d1 ) << std::endl;
    // std::cout << "d2 " << MiscUtils::cvmat_info( d2 ) << std::endl;

    #endif


    #if 0
    // try to get it correct on scaled images, especially not see the border of 31 which props up :(
    ___StaticPointFeatureMatching__gms_point_feature_matches( cout << "n_orb_feat=" << n_orb_feat << endl; )
    // cv::Ptr<cv::ORB> orb = cv::ORB::create(5000);
    cv::Ptr<cv::ORB> orb = cv::ORB::create(n_orb_feat);
    orb->setMaxFeatures( 5000 );
    orb->setNLevels( 1 );
    orb->setFastThreshold(0);
    orb->setPatchSize( 11 );
    cout << "getPatchSize=" << orb->getPatchSize() << endl;

    ___StaticPointFeatureMatching__gms_point_feature_matches(timer.tic();)
    orb->detectAndCompute(imleft_undistorted, cv::Mat(), kp1, d1);
    orb->detectAndCompute(imright_undistorted, cv::Mat(), kp2, d2);
    ___StaticPointFeatureMatching__gms_point_feature_matches(cout <<  timer.toc_milli()  << " (ms) 2X detectAndCompute(ms) : "<< endl;)
    // std::cout << "d1 " << MiscUtils::cvmat_info( d1 ) << std::endl;
    // std::cout << "d2 " << MiscUtils::cvmat_info( d2 ) << std::endl;
    #endif

    ___StaticPointFeatureMatching__gms_point_feature_matches(timer.tic();)

    #if 1
    int im_width = imleft_undistorted.cols;
    _orb_param( cout << "ORB PARAMS:\t"; )
    int 	nfeatures = n_orb_feat;              _orb_param( cout << "nfeatures=" << nfeatures << "\t"; )
    float 	scaleFactor = 1.5f;            _orb_param( cout << "scaleFactor=" << scaleFactor << "\t"; )
    int 	nlevels = 3;                   _orb_param( cout << "nlevels=" << nlevels << "\t"; )
    int 	edgeThreshold = im_width/20;   _orb_param( cout << "edgeThreshold=" << edgeThreshold << "\t"; )
    int 	firstLevel = 0;                _orb_param( cout << "firstLevel=" << firstLevel << "\t"; )
    int 	WTA_K = 4;                     _orb_param( cout << "WTA_K=" << WTA_K << "\t"; )
    int 	scoreType =   ORB::FAST_SCORE ; //ORB::HARRIS_SCORE;
    int 	patchSize = 31; /*im_width/8;*/       _orb_param( cout << "patchSize=" << patchSize << "\t"; )
    float 	fastThreshold = 0;             _orb_param( cout << "fastThreshold=" << fastThreshold << "\t\n"; )

    #if 0
    cv::Ptr<FeatureDetector> detector = ORB::create(nfeatures,scaleFactor,nlevels,edgeThreshold, firstLevel, WTA_K,scoreType,patchSize,fastThreshold );
    cv::Ptr<DescriptorExtractor> descriptor = ORB::create(nfeatures,scaleFactor,nlevels,edgeThreshold, firstLevel, WTA_K,scoreType,patchSize,fastThreshold);
    detector->detect ( imleft_undistorted,kp1 );
    detector->detect ( imright_undistorted,kp2 );
    descriptor->compute ( imleft_undistorted,  kp1, d1 );
    descriptor->compute ( imright_undistorted, kp2, d2 );
    #else
    cv::Ptr<cv::ORB> orb = ORB::create(nfeatures,scaleFactor,nlevels,edgeThreshold, firstLevel, WTA_K,scoreType,patchSize,fastThreshold );
    orb->detectAndCompute(imleft_undistorted, cv::Mat(), kp1, d1);
    orb->detectAndCompute(imright_undistorted, cv::Mat(), kp2, d2);
    #endif


    ___StaticPointFeatureMatching__gms_point_feature_matches(
    cout << TermColor::YELLOW();
    std::cout << ">>> kp1.size=" << kp1.size() << "\t" << "d1 " << MiscUtils::cvmat_info( d1 ) << endl;
    std::cout << ">>> kp2.size=" << kp2.size() << "\t" << "d2 " << MiscUtils::cvmat_info( d2 ) << endl;

    cout << TermColor::RESET();
    std::cout << timer.toc_milli() << " : (ms) detectAndCompute took\n";
    )


    #endif

    #if 0
    //plot
    cv::Mat dst_left, dst_right;
    MatrixXd e_kp1, e_kp2;
    MiscUtils::keypoint_2_eigen( kp1, e_kp1 );
    MiscUtils::keypoint_2_eigen( kp2, e_kp2 );
    MiscUtils::plot_point_sets( imleft_undistorted, e_kp1, dst_left, cv::Scalar(0,0,255), false );
    MiscUtils::plot_point_sets( imright_undistorted, e_kp2, dst_right, cv::Scalar(0,0,255), false );
    cv::imshow( "dst_left", dst_left );
    cv::imshow( "dst_right", dst_right );
    #endif

    //
    // Point feature matching
    cv::BFMatcher matcher(cv::NORM_HAMMING2); // TODO try FLANN matcher here.
    vector<cv::DMatch> matches_all, matches_gms;
    ___StaticPointFeatureMatching__gms_point_feature_matches(timer.tic();)
    matcher.match(d1, d2, matches_all);
    ___StaticPointFeatureMatching__gms_point_feature_matches(
    std::cout << timer.toc_milli() << " : (ms) BFMatcher took (ms)\t";
    std::cout << "BFMatcher : npts = " << matches_all.size() << std::endl;
    )


    // gms_matcher
    #if 1 // make this to 0 to disable GMS, and just return the brute force matches. 1 to enable GMS
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
    #else
    matches_gms = matches_all;
    #endif
    // MatrixXd M1, M2;
    if( matches_gms.size() > 0 )
        MiscUtils::dmatch_2_eigen( kp1, kp2, matches_gms, u, ud, true );


}

void StaticPointFeatureMatching::gms_point_feature_matches_scaled( const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
                            MatrixXd& u, MatrixXd& ud,
                            float scale, int n_orb_feat )
{
    assert( imleft_undistorted.data && imright_undistorted.data );
    assert( scale > 0.1 && scale < 0.99 );

    // blur input images
    int scale_inv = 1 + (int) (1.0 / scale);
    cv::Mat imleft_undistorted_blur, imright_undistorted_blur;
    cv::blur(imleft_undistorted,imleft_undistorted_blur,cv::Size(scale_inv,scale_inv));
    cv::blur(imright_undistorted,imright_undistorted_blur,cv::Size(scale_inv,scale_inv));


    // scale images
    cv::Mat imleft_undistorted_scaled, imright_undistorted_scaled;
    #if 0
    cv::resize(imleft_undistorted, imleft_undistorted_scaled, cv::Size(), scale, scale);
    cv::resize(imright_undistorted, imright_undistorted_scaled, cv::Size(), scale, scale);
    #else
    cv::resize(imleft_undistorted_blur, imleft_undistorted_scaled, cv::Size(), scale, scale);
    cv::resize(imright_undistorted_blur, imright_undistorted_scaled, cv::Size(), scale, scale);
    #endif

    // core matcher
    gms_point_feature_matches( imleft_undistorted_scaled, imright_undistorted_scaled, u, ud, n_orb_feat );

    // un-scale the co-ordinates
    if( u.cols() > 0 && ud.cols() > 0 ) {
        // this will cause seg-fault if no point feature matches were found
        u *= (1.0/scale);
        ud *= (1.0/scale);
    }

}



// #define  ___StaticPointFeatureMatching__refine_and_sparsify_matches( msg ) msg;
#define ___StaticPointFeatureMatching__refine_and_sparsify_matches( msg ) ;
// Given a match refines it. Use this with GMS matcher which usually gives quite dense matching but not as precise.
//      This does the following:
//          a. Sparify the matches
//          b. Do optical flow tracking using the matches as initial guess for LKOpticalFlow. This is done to improve precision of the point matches.
//      Params:
//      im_a, im_b : Input images (full resolution)
//      uv_a, uv_b : Coarse matching points 2xN or 3xN. Initial matches to be refined for those 2 images respectively.
//      refined_uv_a, refined_uv_b [output] : refined and sparsified points
void StaticPointFeatureMatching::refine_and_sparsify_matches(
    const cv::Mat im_a, const cv::Mat im_b,
    const MatrixXd& uv_a, const MatrixXd& uv_b,
    MatrixXd& refined_uv_a, MatrixXd& refined_uv_b
)
{
    ___StaticPointFeatureMatching__refine_and_sparsify_matches(
    cout << TermColor::GREEN() << "[refine_and_sparsify_matches] starts\n" << TermColor::RESET(); )
    //---- safety when im_a is empty and uv_a is empty
    {
        if( im_a.empty() || im_b.empty() || uv_a.cols() == 0 || uv_b.cols() == 0 ) {
            cout << "[refine_and_sparsify_matches] either of the images of input points was empty\n";
            return;
        }
        if( uv_a.rows() == 2 || uv_a.rows() ==3 || uv_b.rows() == 2 || uv_b.rows() == 3 ) {
            ; // OK!
        }
        else {
            cout << "[refine_and_sparsify_matches] input uv_a,uv_b has to be 2xN or 3xN\n";
            return;
        }
    }


    // PARAMS
    int W = 31; //< 1 point max in a window of this half_win_size

    // END PARAMS


    //---- Sparsify pts
    // loop over all the point matches uv_a and note them in sparse matrix.
    // the purpose is to not pick too many points in a small neibhourhood
    MatrixXi mat = MatrixXi::Zero(im_a.rows, im_a.cols);
    vector<cv::Point2f> ret_uv_a, ret_uv_b;
    ___StaticPointFeatureMatching__refine_and_sparsify_matches( ElapsedTime t_sparsify("Sparsify"); )
    for( int i=0 ; i<uv_a.cols() ; i++ )
    {
        auto u = uv_a(0,i); //x coef
        auto v = uv_a(1,i); //y coef
        if( mat(v,u) > 0 ) {
            // cout << TermColor::RED() << u << "," << v << TermColor::RESET() <<  endl;
            continue;
        }

        auto ud = uv_b(0,i); //x coef
        auto vd = uv_b(1,i); //y coef

        // cout << u << "," << v << "<--->" << ud << "," << vd << endl;
        ret_uv_a.push_back( cv::Point2f(u,v) );
        ret_uv_b.push_back( cv::Point2f(ud,vd) );

        int v_m = v-int(W/2); //max( 0, v-int(W/2) );  //x coef
        int u_m = u-int(W/2); //max( 0, u-int(W/2) );  //y coef
        mat.block( v_m, u_m, W, W ) = MatrixXi::Constant( W,W, 1.0 );
        // for( int _u = -W ; _u<=W ; _u++)
        //     for( int _v = -W ; _v<=W ; _v++)
        //         mat.coeffRef(v+_v,u+_u) = true;
    }
    ___StaticPointFeatureMatching__refine_and_sparsify_matches(
    cout << t_sparsify.toc() << "\tn_retained=" << ret_uv_a.size() << " out of total=" << uv_a.cols() << endl; )



    //---- optical flow based refine
    ___StaticPointFeatureMatching__refine_and_sparsify_matches( ElapsedTime t_optflow( "Optical Flow Refinement");)
    vector<float> err;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.01);

    #if 0 // set this to 1 to use fwd and reverse optical flow, more robust
    vector<uchar> status1, status2;
    calcOpticalFlowPyrLK(im_a, im_b, ret_uv_a, ret_uv_b, status1, err, cv::Size(35,35), 4, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
    calcOpticalFlowPyrLK(im_b, im_a, ret_uv_b, ret_uv_a, status2, err, cv::Size(35,35), 4, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
    vector<uchar> status = MiscUtils::vector_of_uchar_AND( status1, status2 );
    ___StaticPointFeatureMatching__refine_and_sparsify_matches(
    cout << t_optflow.toc() << "\tOptical flow pts retained: (fwd,rev, AND)="
                << "(" << MiscUtils::total_positives(status1 ) << ","
                << MiscUtils::total_positives(status2 )  << ","
                << MiscUtils::total_positives(status )
                << ")" << endl;
            )
    #endif

    #if 1 //1 way optical flow
    vector<uchar> status;
    calcOpticalFlowPyrLK(im_a, im_b, ret_uv_a, ret_uv_b, status, err, cv::Size(35,35), 4, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
    ___StaticPointFeatureMatching__refine_and_sparsify_matches(
    cout <<  t_optflow.toc() << "\t optical flow retained points: " << MiscUtils::total_positives(status ) << endl; )
    #endif

    MiscUtils::reduce_vector( ret_uv_a, status );
    MiscUtils::reduce_vector( ret_uv_b, status );
    MiscUtils::point2f_2_eigen( ret_uv_a, refined_uv_a );
    MiscUtils::point2f_2_eigen( ret_uv_b, refined_uv_b );


    // for( int j=0 ; j<refined_uv_a.cols() ; j++ )
        // cout << refined_uv_a.col(j).transpose() << "<--->" <<refined_uv_b.col(j).transpose() << endl;

    ___StaticPointFeatureMatching__refine_and_sparsify_matches(
    cout << TermColor::GREEN() << "[refine_and_sparsify_matches] ends\n" << TermColor::RESET(); )
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


    #if 1
    //
    bool make_homogeneous = true;
    u = MatrixXd::Constant( (make_homogeneous?3:2), pts_1.size(), 1.0 );
    ud = MatrixXd::Constant( (make_homogeneous?3:2), pts_2.size(), 1.0 );
    for( int k=0 ; k<(int)pts_1.size() ; k++ )
    {
        u(0,k) = pts_1[k].x;
        u(1,k) = pts_1[k].y;

        ud(0,k) = pts_2[k].x;
        ud(1,k) = pts_2[k].y;
    }
    #endif


    #if 0
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
        #endif


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

    for( int j=0 ; j<(int)matches_raw.size() ; j++ )
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






bool StaticPointFeatureMatching::image_coordinates_to_normalized_image_coordinates(
    const camodocal::CameraPtr camera,
    const MatrixXd& uv, MatrixXd& normed_uv )
{
    assert( uv.cols() > 0 );
    assert( uv.rows() == 3 || uv.rows() == 2 );
    assert( camera );


    normed_uv = MatrixXd::Constant( 3, uv.cols(), 1.0 );
    for( int mm=0 ; mm<uv.cols() ; mm++ ) //to compute normalized image co-ordinates
    {
        Vector2d ____uv(  uv(0,mm), uv(1,mm) );
        Vector3d ____normed_uv;
        camera->liftProjective( ____uv, ____normed_uv );

        // eigen_result_of_opticalflow_normed_im_cord.col(mm).topRows(3) = ____normed_uv;
        normed_uv(0,mm) = ____normed_uv(0);
        normed_uv(1,mm) = ____normed_uv(1);
        normed_uv(2,mm) = ____normed_uv(2);
    }

    return true;
}


MatrixXd StaticPointFeatureMatching::image_coordinates_to_normalized_image_coordinates(
    const camodocal::CameraPtr camera,
    const MatrixXd& uv )
{
    assert( uv.cols() > 0 );
    assert( uv.rows() == 3 || uv.rows() == 2 );
    assert( camera );


    MatrixXd normed_uv = MatrixXd::Constant( 3, uv.cols(), 1.0 );
    for( int mm=0 ; mm<uv.cols() ; mm++ ) //to compute normalized image co-ordinates
    {
        Vector2d ____uv(  uv(0,mm), uv(1,mm) );
        Vector3d ____normed_uv;
        camera->liftProjective( ____uv, ____normed_uv );

        // eigen_result_of_opticalflow_normed_im_cord.col(mm).topRows(3) = ____normed_uv;
        normed_uv(0,mm) = ____normed_uv(0);
        normed_uv(1,mm) = ____normed_uv(1);
        normed_uv(2,mm) = ____normed_uv(2);
    }

    return normed_uv;
}



MatrixXd StaticPointFeatureMatching::normalized_image_cordinates_to_image_coordinates(
    const camodocal::CameraPtr camera, const MatrixXd& normed_uv )
{
    assert( normed_uv.cols() > 0 );
    assert( normed_uv.rows() == 3 );
    assert( camera );

    MatrixXd uv = MatrixXd::Constant( 3, normed_uv.cols(), 1.0 );
    for( int mm=0 ; mm<normed_uv.cols() ; mm++ )
    {
        Vector2d p;
        Vector3d PP;
        PP = normed_uv.col(mm);
        camera->spaceToPlane( PP, p );
        uv(0,mm) = p(0);
        uv(1,mm) = p(1);
        uv(2,mm) = 1.0;
    }

    return uv;
}


VectorXd StaticPointFeatureMatching::depth_at_image_coordinates( const MatrixXd& uv, const cv::Mat& depth_image )
{
    // float near = 0.5;
    // float far = 4.5;
    assert( uv.cols() > 0 && (uv.rows() == 2 || uv.rows() == 3)  );
    assert( depth_image.rows > 0 && depth_image.cols > 0 );
    assert( depth_image.type() == CV_16UC1 || depth_image.type() == CV_32FC1 );
    assert( depth_image.type() == depth_image.type() );


    // make 3d points out of 2d point feature matches
    VectorXd uv_X = VectorXd::Zero( uv.cols() );
    for( int i=0 ; i<uv.cols() ; i++ )
    {
        if( uv(0,i) < 0 || uv(0,i) > depth_image.cols || uv(1,i) < 0 || uv(1,i) > depth_image.rows )
            continue;

        float depth_val;
        if( depth_image.type() == CV_16UC1 ) {
            depth_val = .001 * depth_image.at<uint16_t>( uv(1,i), uv(0,i) );
        }
        else if( depth_image.type() == CV_32FC1 ) {
            // just assuming the depth values are in meters when CV_32FC1
            depth_val = depth_image.at<float>( uv(1,i), uv(0,i) );
        }
        else {
            assert( false );
            cout << "[StaticPointFeatureMatching::depth_at_image_coordinates]depth type is neighter of CV_16UC1 or CV_32FC1\n";
            exit(1);
        }
        // cout << "ath uv_i=" << i << " depth_val = " << depth_val << endl;

        uv_X(i) = depth_val;
        #if 0
        Vector3d _0P;
        Vector2d _0p; _0p << uv(0,i), uv(1,i);
        _camera->liftProjective( _0p, _0P );
        // uv_X.push_back( depth_val * _0P );
        uv_X.col(i).topRows(3) = depth_val * _0P;

        if( depth_val > near && depth_val < far )
            valids.push_back( true );
        else
            valids.push_back( false );
        #endif
    }

    return uv_X;

}



MatrixXd StaticPointFeatureMatching::normalized_image_coordinates_and_depth_to_3dpoints(
    const MatrixXd& normed_uv, const VectorXd d, bool make_homogeneous )
{
    assert( normed_uv.rows() == 3 && d.size() == normed_uv.cols() && normed_uv.cols() > 0  );
    int N = normed_uv.cols();

    MatrixXd result_3d;
    if( make_homogeneous )
        result_3d = MatrixXd::Constant( 4, N, 1.0 );
    else
        result_3d = MatrixXd::Constant( 3, N, 1.0 );


    result_3d.row(0) = normed_uv.row(0).array() * d.transpose().array();
    result_3d.row(1) = normed_uv.row(1).array() * d.transpose().array();
    result_3d.row(2) = normed_uv.row(2).array() * d.transpose().array();
    return result_3d;
}
