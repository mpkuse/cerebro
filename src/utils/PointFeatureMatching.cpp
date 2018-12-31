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
