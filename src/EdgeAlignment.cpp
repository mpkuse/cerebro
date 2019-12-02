#include "EdgeAlignment.h"

EdgeAlignment::EdgeAlignment(const camodocal::CameraPtr _cam, const cv::Mat _im_ref, const cv::Mat _im_curr, cv::Mat _depth_curr):
        cam(_cam), im_ref(_im_ref), im_curr(_im_curr),  depth_curr( _depth_curr)
{
    #if 0
    cout << "~~~~\n~~~~[EdgeAlignment::EdgeAlignment]~~~~\n~~~~\n";
    cout << "\tim_ref : " << MiscUtils::cvmat_info( im_ref ) << endl;
    cout << "\tim_curr: " << MiscUtils::cvmat_info( im_curr ) << endl;
    cout << "\tdepth_curr: " << MiscUtils::cvmat_info( depth_curr ) << endl;
    // TODO Check the data types atleast
    #endif
}

// #define __EdgeAlignment__solve_debug(msg) msg;
#define __EdgeAlignment__solve_debug(msg) ;

// #define __EdgeAlignment__solve( msg ) msg;
#define __EdgeAlignment__solve( msg ) ;

// #define __EdgeAlignment__solve_imshow( msg ) msg;
#define __EdgeAlignment__solve_imshow( msg ) ;
bool EdgeAlignment::solve( const Matrix4d& initial_guess____ref_T_curr, Matrix4d& out_optimized__ref_T_curr )
{
    //----- distance transform will be made with edgemap of reference image
    ElapsedTime t_distanceTransform( "Distance Transform");
    cv::Mat disTrans, edge_map;
    get_distance_transform( im_ref, disTrans, edge_map );
    Eigen::MatrixXd e_disTrans;
    cv::cv2eigen( disTrans, e_disTrans );

    __EdgeAlignment__solve(
    cout << TermColor::uGREEN() <<  t_distanceTransform.toc() << "\t" << TermColor::RESET();
    std::cout << "disTrans: " << MiscUtils::cvmat_info( disTrans ) << "\t" << MiscUtils::cvmat_minmax_info( disTrans ) << endl;
    )



    __EdgeAlignment__solve_imshow(
    cv::imshow("Distance Transform of ref image", disTrans); //numbers between 0 and 1.
    cv::imshow( "edge_map of ref image", edge_map );

    cv::Mat cm_disTrans;
    cv::Mat __tmp;
    disTrans.convertTo( __tmp, CV_8UC1, 255 );
    cv::applyColorMap( __tmp, cm_disTrans, cv::COLORMAP_JET);
    // cv::imshow( "false color map of disTrans", cm_disTrans );

    );



     //---- 3d points will be made from curr. Only at edges
     ElapsedTime t_3dpts( "3D edge-points of curr image" );
     MatrixXd cX = get_cX_at_edge_pts( im_curr, depth_curr ); //cX will be 4xN
     __EdgeAlignment__solve(
     cout << TermColor::uGREEN() << t_3dpts.toc() << TermColor::RESET() << "\ttotal edge pts=" << cX.cols() << endl;
     )
     __EdgeAlignment__solve_debug( cout << "cX(1st 10 cols)\n" << cX.leftCols(10) << endl; )


     //  use the initial guess and try projecting these points on im_ref
     __EdgeAlignment__solve_debug( cout << "Initial Guess : " << PoseManipUtils::prettyprintMatrix4d( initial_guess____ref_T_curr ) << endl;  )
     __EdgeAlignment__solve_debug( cout << "Initial Guess :\n" << initial_guess____ref_T_curr << endl; )

     __EdgeAlignment__solve_imshow(
     MatrixXd ref_uv = reproject( cX, initial_guess____ref_T_curr );
     cv::Mat dst;
     #if 1 //make this to 1 to plot of input image, 0 to plot on distance transform of ref image
     MiscUtils::plot_point_sets( im_ref, ref_uv, dst, cv::Scalar(0,0,255), false, "initial" );
     #else
     MiscUtils::plot_point_sets( cm_disTrans, ref_uv, dst, cv::Scalar(0,0,255), false, "initial" );
     #endif
     MiscUtils::append_status_image( dst, "^^^im_ref;reprojecting 3d pts of im_curr on im_ref using initial guess of rel-pose;initial_guess____ref_T_curr="+PoseManipUtils::prettyprintMatrix4d(initial_guess____ref_T_curr) );
     cv::imshow( "reprojecting 3d pts of curr on ref using initial guess of rel-pose", dst );
     )



     //---
     //---- Setup the optimization problem
     //---

    // Grid
    // cout << "e_disTrans.shape = " << e_disTrans.rows() << ", " << e_disTrans.cols() << endl;
    ceres::Grid2D<double,1> grid( e_disTrans.data(), 0, e_disTrans.cols(), 0, e_disTrans.rows() );
    ceres::BiCubicInterpolator< ceres::Grid2D<double,1> > interpolated_imb_disTrans( grid );


    // opt var
    Eigen::Matrix4d ref_T_curr_optvar = initial_guess____ref_T_curr; //Eigen::Matrix4d::Identity();
    double ref_quat_curr[10], ref_t_curr[10];
    PoseManipUtils::eigenmat_to_raw_xyzw( ref_T_curr_optvar, ref_quat_curr, ref_t_curr );



    // camera params (needed for reprojection as grid is indexed by image co-ordinates)
    std::vector<double> parameterVec;
    cam->writeParameters( parameterVec );
    double fx=parameterVec.at(4);
    double fy=parameterVec.at(5);
    double cx=parameterVec.at(6);
    double cy=parameterVec.at(7);
    __EdgeAlignment__solve_debug(
    cout << "fx=" << fx << "\t";
    cout << "fy=" << fy << "\t";
    cout << "cx=" << cx << "\t";
    cout << "cy=" << cy << "\n"; )



    // Residues for each 3d points
    ceres::Problem problem;
    auto robust_loss = new ceres::CauchyLoss(.2);

    VectorXi edge_pt_used = VectorXi::Zero( cX.cols() );
    for( int i=0 ; i< cX.cols() ; i+=30 )
    {
        // ceres::CostFunction * cost_function = EAResidue::Create( K, a_X.col(i), interpolated_imb_disTrans);

        ceres::CostFunction * cost_function = EAResidue::Create( fx,fy,cx,cy, cX(0,i),cX(1,i),cX(2,i), interpolated_imb_disTrans);
        problem.AddResidualBlock( cost_function, robust_loss, ref_quat_curr, ref_t_curr );

        // note that this point was used for residue computation
        edge_pt_used(i) = 1;
    }

    // If you are using Eigen’s Quaternion object, whose layout is x,y,z,w, then you should use EigenQuaternionParameterization.
    ceres::LocalParameterization * eigenquaternion_parameterization = new ceres::EigenQuaternionParameterization;
    // ceres::LocalParameterization * quaternion_parameterization = new ceres::QuaternionParameterization;
    problem.SetParameterization( ref_quat_curr, eigenquaternion_parameterization );


    // Run
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;

    ElapsedTime t_easolver( "EAResidue Solver");
    __EdgeAlignment__solve_debug(options.minimizer_progress_to_stdout = false; );
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );
    __EdgeAlignment__solve_debug( std::cout << summary.FullReport() << "\n"; )
    __EdgeAlignment__solve( std::cout << summary.BriefReport() << endl;
    cout << TermColor::uGREEN() << t_easolver.toc() << TermColor::RESET() << endl; )

    // Retrive final
    PoseManipUtils::raw_xyzw_to_eigenmat( ref_quat_curr, ref_t_curr, ref_T_curr_optvar );
    __EdgeAlignment__solve(
    cout << "Initial Guess : " << PoseManipUtils::prettyprintMatrix4d( initial_guess____ref_T_curr ) << endl;
    cout << "Final Guess : " << PoseManipUtils::prettyprintMatrix4d( ref_T_curr_optvar ) << endl; )


    __EdgeAlignment__solve_imshow(
    MatrixXd ref_uv_final = reproject( cX, ref_T_curr_optvar );
    cv::Mat dst_final;
    #if 1 //make this to 1 to plot of input image, 0 to plot on distance transform of ref image
    MiscUtils::plot_point_sets( im_ref, ref_uv_final, dst_final, cv::Scalar(0,0,255), false, "final" );
    #else
    MiscUtils::plot_point_sets( cm_disTrans, ref_uv_final, dst_final, cv::Scalar(0,0,255), false, "final" );
    #endif
    MiscUtils::append_status_image( dst_final, "^^^im_ref;reprojecting 3d pts of im_curr on im_ref using pose after optimization of rel-pose;ref_T_curr_optvar="+PoseManipUtils::prettyprintMatrix4d(ref_T_curr_optvar) );

    string brief_report = summary.BriefReport();
    std::replace( brief_report.begin(), brief_report.end(), ',', ';');

    MiscUtils::append_status_image( dst_final, brief_report );
    cv::imshow( "dst_final", dst_final );
    )

    out_optimized__ref_T_curr = ref_T_curr_optvar;


    //...DONE


    // bool make_representation_image = true;
    if( this->make_representation_image )
    {
        cout << TermColor::iWHITE() << "[EdgeAlignment::solve] You asked to be make debug image. For production runs disable this\n" << TermColor::RESET();
        // [ CURR ]
        // [ REF with edgepts of curr with initial_guess____ref_T_curr ]
        // [ REF with edgepts of curr with ref_T_curr_optvar ]

        //
        // [CURR]
        //
        cv::Mat im_curr_resized_1, im_curr_resized;
        cv::resize(im_curr, im_curr_resized_1, cv::Size(), 0.5, 0.5 );
        if( im_curr_resized_1.channels() == 1 )
            cv::cvtColor( im_curr_resized_1, im_curr_resized, cv::COLOR_GRAY2BGR );
        else
            im_curr_resized_1.copyTo(im_curr_resized);
        MiscUtils::append_status_image( im_curr_resized, "^^im_curr", .3  );





        //
        // [ REF with edgepts of curr with initial_guess____ref_T_curr ]
        //
        MatrixXd ref_uv = reproject( cX, initial_guess____ref_T_curr );
        cv::Mat dst_2;
        #if 1 //make this to 1 to plot of input image, 0 to plot on distance transform of ref image

        #if 0 // make this to zero to mark points which were used for residue and which were not used. 1 will plot all the points in same color
        MiscUtils::plot_point_sets( im_ref, ref_uv, dst_2, cv::Scalar(0,0,255), false );
        #else
        vector<cv::Scalar> per_pt_color;
        vector<bool> per_pt_sta;
        for( int h=0 ; h<ref_uv.cols() ; h++ ) {
            if( edge_pt_used(h) > 0 ) {
                per_pt_color.push_back( cv::Scalar(0,0,255) );
                per_pt_sta.push_back(true);
            }
            else {
                per_pt_color.push_back( cv::Scalar(0,255,255) );
                per_pt_sta.push_back(false);
            }
        }
        // MiscUtils::plot_point_sets( im_ref, ref_uv, dst_2, per_pt_color, 1.0 );
        MiscUtils::plot_point_sets( im_ref, ref_uv, dst_2, cv::Scalar(0,0,255), false, "all edgepts" );
        MiscUtils::plot_point_sets_masked( dst_2, ref_uv, per_pt_sta, dst_2, cv::Scalar(0,255,255), false, ";edgepts used for residue computations" );
        #endif


        #else
        MiscUtils::plot_point_sets( cm_disTrans, ref_uv, dst, cv::Scalar(0,0,255), false, "initial" );
        #endif

        string initial_pose_str = PoseManipUtils::prettyprintMatrix4d(initial_guess____ref_T_curr);
        std::replace( initial_pose_str.begin(), initial_pose_str.end(), ':', ';');

        MiscUtils::append_status_image( dst_2, "^^^im_ref;reprojecting 3d pts of im_curr on im_ref;   using initial guess of rel-pose;initial_guess____ref_T_curr="+initial_pose_str, 1.0 );
        cv::Mat dst_2_resized;
        cv::resize(dst_2, dst_2_resized, cv::Size(), 0.5, 0.5 );


        //
        // [ REF with edgepts of curr with ref_T_curr_optvar ]
        //
        MatrixXd ref_uv_final = reproject( cX, ref_T_curr_optvar );
        cv::Mat dst_final;
        #if 1 //make this to 1 to plot of input image, 0 to plot on distance transform of ref image
        MiscUtils::plot_point_sets( im_ref, ref_uv_final, dst_final, cv::Scalar(0,0,255), false );
        #else
        MiscUtils::plot_point_sets( cm_disTrans, ref_uv_final, dst_final, cv::Scalar(0,0,255), false, "final" );
        #endif

        string final_pose_str =PoseManipUtils::prettyprintMatrix4d(ref_T_curr_optvar);
        std::replace( final_pose_str.begin(), final_pose_str.end(), ':', ';');
        MiscUtils::append_status_image( dst_final, "^^^im_ref;reprojecting 3d pts of im_curr on im_ref   using pose after optimization of rel-pose;ref_T_curr_optvar="+final_pose_str, 1.0 );

        string brief_report = summary.BriefReport();
        std::replace( brief_report.begin(), brief_report.end(), ',', ';');

        MiscUtils::append_status_image( dst_final, brief_report, 1.0 );
        cv::Mat dst_final_resized;
        cv::resize(dst_final, dst_final_resized, cv::Size(), 0.5, 0.5 );



        // vertical concat
        cv::Mat _tmpx, _tmp_final ;
        MiscUtils::vertical_side_by_side( dst_2_resized, dst_final_resized, _tmpx );
        // cout << "im_curr_resized: " << MiscUtils::cvmat_info( im_curr_resized ) << endl;
        // cout << "_tmpx: " << MiscUtils::cvmat_info( _tmpx ) << endl;
        MiscUtils::vertical_side_by_side( im_curr_resized, _tmpx, _tmp_final );

        this->representation_image = _tmp_final;
        this->is_representation_image_ready = true;
        // cv::imshow( "_tmpx", _tmpx  );
        // cv::imshow( "_tmp_final", _tmp_final  );
        // cv::waitKey(0);
    }


    // Will give true for CONVERGED, NO_CONVERGE. Will only give false for FAIL . This is less strict
    #if 0
    // return summary.IsSolutionUsable();
    #else
    if( summary.termination_type == ceres::TerminationType::CONVERGENCE )
        return true;
    else return false;
    #endif



}



//utils

// #define get_distance_transform_debug(msg) msg;
#define get_distance_transform_debug(msg);
void EdgeAlignment::get_distance_transform( const cv::Mat& input, cv::Mat& out_distance_transform, cv::Mat& out_edge_map )
{
    // Thresholds that influcence this:
    // a. Gaussian Blur size
    // b. Threshold for the gradient map. How you compute gradient also matter. here i m using Laplacian operator. Results will differ with Sobel for example.
    // c. Window size for median blur
    // d. Params for distance transform computation.

    //
    // Edge Map
    //

    cv::Mat _blur, _gray;
    cv::GaussianBlur( input, _blur, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
    if( _blur.channels() == 1 )
        _gray = _blur;
    else
        cv::cvtColor( _blur, _gray, CV_RGB2GRAY );

    #if 0 // Laplacian
    cv::Mat _laplacian, _laplacian_8uc1;
    cv::Laplacian( _gray, _laplacian, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( _laplacian, _laplacian_8uc1 );
    get_distance_transform_debug( cv::imshow( "_laplacian_8uc1", _laplacian_8uc1) );
    out_edge_map = _laplacian_8uc1;

    //
    // Threshold gradients
    // TODO - use cv::Threshold
    cv::Mat B = cv::Mat::ones( _laplacian.rows, _laplacian.cols, CV_8UC1 ) * 255;
    for( int v=0 ; v<_laplacian.rows ; v++ )
    {
        for( int u=0 ; u<_laplacian.cols ; u++ )
        {
            if( _laplacian_8uc1.at<uchar>(v,u) > 25 )
            {
                B.at<uchar>(v,u) = 0;
            }
        }
    }

    //
    // Suppress noise with median filter
    cv::Mat B_filtered;
    cv::medianBlur( B, B_filtered, 3 );
    get_distance_transform_debug( cv::imshow( "edge map", B_filtered ) );
    #endif


    #if 1 // Canny
    cv::Mat B_filtered;

    cv::Mat dst, detected_edges;
    // int edgeThresh = 1;
    int lowThreshold=30;
    int ratio = 3;
    int kernel_size = 3;
    cv::Canny( _blur, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    dst = cv::Scalar::all(0);

    _blur.copyTo( dst, detected_edges);
    out_edge_map = dst;


    get_distance_transform_debug( cv::imshow( "edge map", dst ); )
    get_distance_transform_debug(
    cout << "dst: " << MiscUtils::cvmat_info( dst ) << MiscUtils::cvmat_minmax_info(dst) <<  endl;
    )

    // B_filtered = 255 - dst;
    cv::threshold( dst, B_filtered, 10, 255, cv::THRESH_BINARY_INV );
    get_distance_transform_debug( cv::imshow( "B_filtered edge map", B_filtered ) );

    #endif


    //
    // Distance Transform
    //
    cv::Mat dist;
    cv::distanceTransform(B_filtered, dist, cv::DIST_L2, 3);
    cv::normalize(dist, dist, 0, 1., cv::NORM_MINMAX);
    get_distance_transform_debug( cout << "dist : " << MiscUtils::cvmat_info(dist ) << endl; )
    get_distance_transform_debug( imshow("Distance Transform Image", dist) );

    out_distance_transform = dist;

}



// #define __EdgeAlignment__get_cX_at_edge_pts( msg ) msg;
#define __EdgeAlignment__get_cX_at_edge_pts( msg ) ;
MatrixXd EdgeAlignment::get_cX_at_edge_pts( const cv::Mat im, const cv::Mat depth_map   )
{
    assert( !im.empty() && !depth_map.empty() );
    assert( im.rows == depth_map.rows && im.cols == depth_map.cols );
    assert( depth_map.type() == CV_16UC1 || depth_map.type() == CV_32FC1 );
    assert( cam );

    //---- get edgemap with canny
    cv::Mat _blur, _gray;
    cv::GaussianBlur( im, _blur, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
    if( _blur.channels() == 1 )
        _gray = _blur;
    else
        cv::cvtColor( _blur, _gray, CV_RGB2GRAY );


    cv::Mat dst, detected_edges;
    // int edgeThresh = 1;
    int lowThreshold=40;
    int ratio = 3;
    int kernel_size = 3;
    cv::Canny( _blur, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    dst = cv::Scalar::all(0);

    _blur.copyTo( dst, detected_edges);

    __EdgeAlignment__get_cX_at_edge_pts( cv::imshow( "edge map get_cX", dst ); )



    //---- loop over all the pixels and process only at edge points
    vector<cv::Point3f> vec_of_pt;
    for( int v=0 ; v< im.rows ; v++ )
    {
        for( int u=0 ; u<im.cols ; u++ )
        {
            float depth_val;
            if( depth_map.type() == CV_16UC1 ) {
                depth_val = .001 * depth_map.at<uint16_t>( v, u );
            }
            else if( depth_map.type() == CV_32FC1 ) {
                // just assuming the depth values are in meters when CV_32FC1
                depth_val = depth_map.at<float>(v, u );
            }
            else {
                cout << "[EdgeAlignment::get_cX_at_edge_pts]depth type is neighter of CV_16UC1 or CV_32FC1\n";
                throw "[EdgeAlignment::get_cX_at_edge_pts]depth type is neighter of CV_16UC1 or CV_32FC1\n";
            }
            // cout << "at u=" << u << ", v=" << v << "\tdepth_val = " << depth_val << endl;


            if( dst.at<uchar>(v,u) < 10 || depth_val < 0.5 || depth_val > 5. )
                continue;


            Vector3d _1P;
            Vector2d _1p; _1p << u, v;
            cam->liftProjective( _1p, _1P );

            cv::Point3f pt;
            pt.x = depth_val * _1P(0);
            pt.y = depth_val * _1P(1);
            pt.z = depth_val;

            vec_of_pt.push_back( pt );
        }
    }

    MatrixXd cX;
    MiscUtils::point3f_2_eigen( vec_of_pt, cX );
    __EdgeAlignment__get_cX_at_edge_pts(
    cout << "cX.shape=" << cX.rows() << "x" << cX.cols() << endl;
    cout << "vec_of_pt.size() = " << vec_of_pt.size() << endl; )
    return cX;

}





Eigen::MatrixXd EdgeAlignment::reproject( const Eigen::MatrixXd& a_X, const Eigen::Matrix4d& b_T_a )
{
    assert( cam );
    assert( a_X.rows() == 4 && a_X.cols() > 0 );
    Eigen::MatrixXd b_X = b_T_a * a_X;

    MatrixXd uv = MatrixXd::Constant( 3, b_X.cols(), 1.0 );
    for( int i=0 ; i<b_X.cols() ; i++ )
    {

        Vector2d p_dst;
        cam->spaceToPlane( b_X.col(i).topRows(3), p_dst  );

        uv.col(i).topRows(2) = p_dst;
    }
    return uv;


}
