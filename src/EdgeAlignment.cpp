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

// #define __EdgeAlignment__solve_profiling( msg ) msg;
#define __EdgeAlignment__solve_profiling( msg ) ;
bool EdgeAlignment::solve( const Matrix4d& initial_guess____ref_T_curr, Matrix4d& out_optimized__ref_T_curr )
{
    //----- distance transform will be made with edgemap of reference image
    ElapsedTime t_distanceTransform( "Distance Transform");
    cv::Mat disTrans, edge_map;
    get_distance_transform( im_ref, disTrans, edge_map );
    Eigen::MatrixXd e_disTrans;
    cv::cv2eigen( disTrans, e_disTrans );


    __EdgeAlignment__solve_profiling(
    cout << TermColor::uGREEN() <<  t_distanceTransform.toc() << "\n" << TermColor::RESET(); )
    __EdgeAlignment__solve(
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
     cv::Mat im_curr_edgemap, im_curr_selected_pts_edgemap;
     MatrixXd im_curr_uv;
     MatrixXd cX = get_cX_at_edge_pts( im_curr, depth_curr, im_curr_edgemap, im_curr_selected_pts_edgemap ); //cX will be 4xN

    __EdgeAlignment__solve_profiling(
     cout << TermColor::uGREEN() << t_3dpts.toc() << TermColor::RESET() << "\ttotal edge pts=" << cX.cols() << endl;
    )
     __EdgeAlignment__solve(
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
     ElapsedTime t_setup_opt_problem( "Setup Non-linear Least Squares Opt Problem");
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
    for( int i=0 ; i< cX.cols() ; i++) // 30 )
    {
        // ceres::CostFunction * cost_function = EAResidue::Create( K, a_X.col(i), interpolated_imb_disTrans);

        ceres::CostFunction * cost_function = EAResidue::Create(
            fx,fy,cx,cy,
            // cX(0,i),cX(1,i),cX(2,i),
            cX.col(i),
            interpolated_imb_disTrans);
        problem.AddResidualBlock( cost_function, robust_loss, ref_quat_curr, ref_t_curr );

        // note that this point was used for residue computation
        edge_pt_used(i) = 1;
    }

    // If you are using Eigen’s Quaternion object, whose layout is x,y,z,w, then you should use EigenQuaternionParameterization.
    ceres::LocalParameterization * eigenquaternion_parameterization = new ceres::EigenQuaternionParameterization;
    // ceres::LocalParameterization * quaternion_parameterization = new ceres::QuaternionParameterization;
    problem.SetParameterization( ref_quat_curr, eigenquaternion_parameterization );

    __EdgeAlignment__solve_profiling( cout << TermColor::uGREEN() << t_setup_opt_problem.toc() << TermColor::RESET() << endl; )

    // Run
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    // options.minimizer_progress_to_stdout = true;


    ElapsedTime t_easolver( "EAResidue Solver");
    __EdgeAlignment__solve_debug(options.minimizer_progress_to_stdout = false; );
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );
    // std::cout << summary.FullReport() << endl;;

    __EdgeAlignment__solve_profiling( std::cout << summary.BriefReport() << endl;
    cout << TermColor::uGREEN() << t_easolver.toc() << TermColor::RESET() << endl; )

    // Retrive final
    PoseManipUtils::raw_xyzw_to_eigenmat( ref_quat_curr, ref_t_curr, ref_T_curr_optvar );
    __EdgeAlignment__solve(
    cout << "Initial Guess : " << PoseManipUtils::prettyprintMatrix4d( initial_guess____ref_T_curr ) << endl;
    cout << "Final Guess : " << PoseManipUtils::prettyprintMatrix4d( ref_T_curr_optvar ) << endl; )

    out_summary = summary;

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

        cv::Mat im_curr_edgemap_resized, im_curr_selected_pts_edgemap_resized ;
        cv::resize(im_curr_edgemap, im_curr_edgemap_resized, cv::Size(), 0.5, 0.5 ); // all edges
        cv::resize(im_curr_selected_pts_edgemap, im_curr_selected_pts_edgemap_resized, cv::Size(), 0.5, 0.5 ); //selected edges


        MiscUtils::mask_overlay( im_curr_resized_1, im_curr_edgemap_resized, im_curr_resized, cv::Scalar(0,255,255) );
        MiscUtils::mask_overlay( im_curr_resized, im_curr_selected_pts_edgemap_resized, cv::Scalar(0,0,255) );
        MiscUtils::append_status_image( im_curr_resized, "^^im_curr, all edge-pts in yellow, selected edge-pts ("+ to_string(cX.cols()) +") in red", .3  );




        //
        // [ REF with edgepts of curr with initial_guess____ref_T_curr ]
        //
        MatrixXd ref_uv = reproject( cX, initial_guess____ref_T_curr );
        cv::Mat dst_2;
        #if 1 //make this to 1 to plot of input image, 0 to plot on distance transform of ref image

        #if 0 // make this to zero to mark points which were used for residue and which were not used. 1 will plot all the points in same color
        MiscUtils::plot_point_sets( im_ref, ref_uv, dst_2, cv::Scalar(0,0,255), false );
        #else
        // vector<cv::Scalar> per_pt_color;
        // vector<bool> per_pt_sta;
        // for( int h=0 ; h<ref_uv.cols() ; h++ ) {
        //     if( edge_pt_used(h) > 0 ) {
        //         per_pt_color.push_back( cv::Scalar(0,0,255) );
        //         per_pt_sta.push_back(true);
        //     }
        //     else {
        //         per_pt_color.push_back( cv::Scalar(0,255,255) );
        //         per_pt_sta.push_back(false);
        //     }
        // }
        // MiscUtils::plot_point_sets( im_ref, ref_uv, dst_2, per_pt_color, 1.0 );
        MiscUtils::plot_point_sets( im_ref, ref_uv, dst_2, cv::Scalar(0,0,255), false, "" );
        // MiscUtils::plot_point_sets( im_ref, ref_uv, dst_2, cv::Scalar(0,0,255), false, "all edgepts" );
        // MiscUtils::plot_point_sets_masked( dst_2, ref_uv, per_pt_sta, dst_2, cv::Scalar(0,255,255), false, ";edgepts used for residue computations" );
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

// #define __EdgeAlignment__solve4DOF( msg ) msg;
#define __EdgeAlignment__solve4DOF( msg ) ;
bool EdgeAlignment::solve4DOF( const Matrix4d& initial_guess____ref_T_curr,
    const Matrix4d& imu_T_cam, const Matrix4d& vio_w_T_ref, const Matrix4d& vio_w_T_curr,
    Matrix4d& optimized__ref_T_curr  )
{
    //----- distance transform will be made with edgemap of reference image
    cv::Mat disTrans, edge_map;
    get_distance_transform( im_ref, disTrans, edge_map );
    Eigen::MatrixXd e_disTrans;
    cv::cv2eigen( disTrans, e_disTrans );

    // cv::imshow( "edge_map of ref image", edge_map );




    //---- 3d points of current represented in curr-camera frame-of-ref
    cv::Mat im_curr_edgemap, im_curr_selected_pts_edgemap;
    // MatrixXd im_curr_uv;
    MatrixXd cX = get_cX_at_edge_pts( im_curr, depth_curr, im_curr_edgemap, im_curr_selected_pts_edgemap ); //cX will be 4xN
    // cout << "cX.shape=" << cX.rows() << "x" << cX.cols() << endl;


    //----
    //---- Setup optimization problem
    //----

    //--make grid
    ceres::Grid2D<double,1> grid( e_disTrans.data(), 0, e_disTrans.cols(), 0, e_disTrans.rows() );
    ceres::BiCubicInterpolator< ceres::Grid2D<double,1> > interpolated_imb_disTrans( grid );


    //--optimization variables
    //  rel_yaw of imu, rel_tx of imu, rel_ty of imu
    double refimu_ypr_currimu[3], refimu_tr_currimu[3]; //only yaw and tx,ty,tz are optimization variables

    //--set opt variables to values
    //      i) yaw, tx, ty, tz from initial guess; ii) pitch and roll from vio
    Matrix4d initial_guess____refimu_T_currimu =  imu_T_cam * initial_guess____ref_T_curr * imu_T_cam.inverse();
    Matrix4d vio_ref_T_curr =  vio_w_T_ref.inverse() * vio_w_T_curr;
    Matrix4d vio_refimu_T_currimu = imu_T_cam * vio_w_T_ref.inverse() * vio_w_T_curr * imu_T_cam.inverse();

    {
        //i)
        // yaw, tx, ty, tz from initial guess;
        double _tmp_ypr[3], _tmp_tr[3];
        PoseManipUtils::eigenmat_to_rawyprt( initial_guess____refimu_T_currimu,  _tmp_ypr, _tmp_tr );
        refimu_ypr_currimu[0] = _tmp_ypr[0];
        refimu_tr_currimu[0] = _tmp_tr[0];
        refimu_tr_currimu[1] = _tmp_tr[1];
        refimu_tr_currimu[2] = _tmp_tr[2];

    }


    {
        //ii)
        // pitch and roll from vio
        double _tmp_ypr[3], _tmp_tr[3];
        PoseManipUtils::eigenmat_to_rawyprt( vio_refimu_T_currimu,  _tmp_ypr, _tmp_tr );
        refimu_ypr_currimu[1] = _tmp_ypr[1];
        refimu_ypr_currimu[2] = _tmp_ypr[2];
    }

    __EdgeAlignment__solve4DOF(
    cout << "[solve4DOF]refimu_ypr_currimu: ";
    cout << "ypr=" << refimu_ypr_currimu[0] <<"," << refimu_ypr_currimu[1] <<", " <<  refimu_ypr_currimu[2] <<"\t";
    cout << "t=" << refimu_tr_currimu[0] <<"," << refimu_tr_currimu[1] <<"," << refimu_tr_currimu[2] <<"\t";
    cout << endl;
    )


    //-- camera params (needed for reprojection as grid is indexed by image co-ordinates)
    std::vector<double> parameterVec;
    cam->writeParameters( parameterVec );
    double fx=parameterVec.at(4);
    double fy=parameterVec.at(5);
    double cx=parameterVec.at(6);
    double cy=parameterVec.at(7);
    __EdgeAlignment__solve4DOF(
    cout << "[solve4DOF]";
    cout << "fx=" << fx << "\t";
    cout << "fy=" << fy << "\t";
    cout << "cx=" << cx << "\t";
    cout << "cy=" << cy << "\n";
    )




    cv::Mat rep_im_3;
    if( this->make_representation_image )
    {
        // plot on im_ref <--- PI( initial_guess * cX )
        Matrix4d _tmp;
        // PoseManipUtils::rawyprt_to_eigenmat( refimu_ypr_currimu, refimu_tr_currimu, _tmp  );
        rawyprt_to_eigenmat( refimu_ypr_currimu[0], refimu_ypr_currimu[1],  refimu_ypr_currimu[2],
                            refimu_tr_currimu[0], refimu_tr_currimu[1], refimu_tr_currimu[2],
                            _tmp
                        );
        Matrix4d touse__ref_T_curr = imu_T_cam.inverse() * _tmp * imu_T_cam;
        __EdgeAlignment__solve4DOF(
            cout << "[solve4DOFtouse__ref_T_curr:  " << PoseManipUtils::prettyprintMatrix4d( touse__ref_T_curr ) << endl;
        )
        MatrixXd ref_uv = reproject( cX, touse__ref_T_curr );
        cv::Mat dst;
        MiscUtils::plot_point_sets( im_ref, ref_uv, dst, cv::Scalar(0,0,255), false );

        stringstream buffer;
        buffer << "^^^im_ref;reprojecting 3d pts of im_curr on im_ref;  using initial guess of rel-pose;";
        buffer << "  y,tx,ty,tz are optvars init from guess; pitch,roll from vio_refimu_T_currimu;";

        string tmp_str_a = PoseManipUtils::prettyprintMatrix4d(touse__ref_T_curr);
        std::replace( tmp_str_a.begin(), tmp_str_a.end(), ':', ';');
        buffer << ";touse__ref_T_curr=" << tmp_str_a << ";";

        string tmp_str_b = PoseManipUtils::prettyprintMatrix4d(_tmp);
        std::replace( tmp_str_b.begin(), tmp_str_b.end(), ':', ';');
        buffer << ";touse__refimu_T_currimu=" << tmp_str_b << ";";

        MiscUtils::append_status_image( dst, buffer.str(), 1.0 );

        // MiscUtils::append_status_image( dst, "touse__ref_T_curr="+PoseManipUtils::prettyprintMatrix4d(touse__ref_T_curr) );
        // MiscUtils::append_status_image( dst, "touse__refimu_T_currimu="+PoseManipUtils::prettyprintMatrix4d(_tmp) );
        // cv::imshow( "(before)reprojecting 3d pts of curr on ref using initial guess of rel-pose", dst );
        // char key = cv::waitKey(0);

        // exit(1);
        cv::resize(dst, rep_im_3, cv::Size(), 0.5, 0.5 ); //selected edges
    }





    //-- residue terms
    ceres::Problem problem;
    auto robust_loss = new ceres::CauchyLoss(.1);
    for( int i=0 ; i<cX.cols() ; i++ )
    {
        ceres::CostFunction * cost_function = EA4DOFResidue::Create(
            fx,fy,cx,cy,
            cX(0,i),cX(1,i),cX(2,i),
            // cX.col(i),
            interpolated_imb_disTrans,
            refimu_ypr_currimu[1], refimu_ypr_currimu[2],
            imu_T_cam, 1.0
            );
        problem.AddResidualBlock( cost_function, /*NULL*/ robust_loss, &refimu_ypr_currimu[0], &refimu_tr_currimu[0] );

    }

    //-- parameterization for angle (yaw)
    ceres::LocalParameterization* angle_local_parameterization =
                AngleLocalParameterization::Create();
    problem.SetParameterization( &refimu_ypr_currimu[0], angle_local_parameterization );




    //-- Solve
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );
    // std::cout << summary.FullReport();
    __EdgeAlignment__solve4DOF( std::cout << "[solve4DOF]" << summary.BriefReport() << endl; );

    out_summary = summary;


    //-- retrive solution
    Matrix4d out_refimu_T_currimu;
    // PoseManipUtils::rawyprt_to_eigenmat( refimu_ypr_currimu, refimu_tr_currimu, _tmp  );
    rawyprt_to_eigenmat( refimu_ypr_currimu[0], refimu_ypr_currimu[1],  refimu_ypr_currimu[2],
                        refimu_tr_currimu[0], refimu_tr_currimu[1], refimu_tr_currimu[2],
                        out_refimu_T_currimu
                    );
    optimized__ref_T_curr = imu_T_cam.inverse() * out_refimu_T_currimu * imu_T_cam; //output




    //...DONE

    // plottting
    if( this->make_representation_image )
    {
        cout << TermColor::iWHITE() << "[EdgeAlignment::solve4DOF] You asked to be make debug image. For production runs disable this\n" << TermColor::RESET();

        // [ CURR ]
        // [ REF with edgepts of curr with initial_guess____ref_T_curr ]
        // [ REF with edgepts of curr with ref_T_curr_optvar ]

        //
        // [CURR]
        //
        cv::Mat im_curr_resized_1, im_curr_resized;
        cv::resize(im_curr, im_curr_resized_1, cv::Size(), 0.5, 0.5 );

        cv::Mat im_curr_edgemap_resized, im_curr_selected_pts_edgemap_resized ;
        cv::resize(im_curr_edgemap, im_curr_edgemap_resized, cv::Size(), 0.5, 0.5 ); // all edges
        cv::resize(im_curr_selected_pts_edgemap, im_curr_selected_pts_edgemap_resized, cv::Size(), 0.5, 0.5 ); //selected edges


        MiscUtils::mask_overlay( im_curr_resized_1, im_curr_edgemap_resized, im_curr_resized, cv::Scalar(0,255,255) );
        MiscUtils::mask_overlay( im_curr_resized, im_curr_selected_pts_edgemap_resized, cv::Scalar(0,0,255) );
        MiscUtils::append_status_image( im_curr_resized, "^^im_curr; all edge-pts in yellow; selected edge-pts ("+ to_string(cX.cols()) +") in red", 0.5  );


        string string____vio_refimu_T_currimu = PoseManipUtils::prettyprintMatrix4d( vio_refimu_T_currimu );
        std::replace( string____vio_refimu_T_currimu.begin(), string____vio_refimu_T_currimu.end(), ':', ';');
        MiscUtils::append_status_image( im_curr_resized, "vio_refimu_T_currimu="+string____vio_refimu_T_currimu, 0.5  );




        //
        // [ REF with edgepts of curr with initial_guess____ref_T_curr ]
        //
        MatrixXd ref_uv = reproject( cX, initial_guess____ref_T_curr );
        cv::Mat dst_2;
        MiscUtils::plot_point_sets( im_ref, ref_uv, dst_2, cv::Scalar(0,0,255), false, "" );



        // reproject in blue using the odometry
        MatrixXd ref_uv_using_odom = reproject( cX, vio_ref_T_curr );
        MiscUtils::plot_point_sets( dst_2, ref_uv_using_odom, cv::Scalar(255,0,0), false, "" );
        MiscUtils::append_status_image( dst_2, "in blue, reproject cX using vio_ref_T_curr", 1.0);




        string initial_pose_str = PoseManipUtils::prettyprintMatrix4d(initial_guess____ref_T_curr);
        std::replace( initial_pose_str.begin(), initial_pose_str.end(), ':', ';');

        string initial_pose_str_imuframe = PoseManipUtils::prettyprintMatrix4d( initial_guess____refimu_T_currimu );
        std::replace( initial_pose_str_imuframe.begin(), initial_pose_str_imuframe.end(), ':', ';');

        MiscUtils::append_status_image( dst_2, "^^^im_ref;reprojecting 3d pts of im_curr on im_ref;   using initial guess of rel-pose;;initial_guess____ref_T_curr="+initial_pose_str+";;initial_guess____refimu_T_currimu="+initial_pose_str_imuframe, 1.0 );




        cv::Mat dst_2_resized;
        cv::resize(dst_2, dst_2_resized, cv::Size(), 0.5, 0.5 );


        //
        // [REF with input used for the 4DOF optimization, ie pitch and roll from vio, yaw, tx,ty,tz from initial guess]
        //          done before setting up the optimization variable, result in `rep_im_3`



        //
        // [REF after 4DOF optimization]
        Matrix4d _tmp;
        cv::Mat rep_image_4;
        // PoseManipUtils::rawyprt_to_eigenmat( refimu_ypr_currimu, refimu_tr_currimu, _tmp  );
        rawyprt_to_eigenmat( refimu_ypr_currimu[0], refimu_ypr_currimu[1],  refimu_ypr_currimu[2],
                            refimu_tr_currimu[0], refimu_tr_currimu[1], refimu_tr_currimu[2],
                            _tmp
                        );
        Matrix4d touse__ref_T_curr = imu_T_cam.inverse() * _tmp * imu_T_cam;
        // cout << "touse__ref_T_curr:  " << PoseManipUtils::prettyprintMatrix4d( touse__ref_T_curr ) << endl;
        // MatrixXd
        ref_uv = reproject( cX, touse__ref_T_curr );
        cv::Mat dst;
        MiscUtils::plot_point_sets( im_ref, ref_uv, dst, cv::Scalar(0,0,255), false );

        stringstream buffer;
        buffer << "^^^im_ref;reprojecting 3d pts of im_curr on im_ref; after 4DOF optimization;";


        string brief_report = summary.BriefReport();
        std::replace( brief_report.begin(), brief_report.end(), ',', ';');
        buffer << ";" << brief_report << ";";

        string string____touse__ref_T_curr = PoseManipUtils::prettyprintMatrix4d(touse__ref_T_curr) ;
        std::replace( string____touse__ref_T_curr.begin(), string____touse__ref_T_curr.end(), ':', ';');
        buffer << ";touse__ref_T_curr(final output)="+string____touse__ref_T_curr << ";";

        string string____touse__refimu_T_currimu = PoseManipUtils::prettyprintMatrix4d(_tmp) ;
        std::replace( string____touse__refimu_T_currimu.begin(), string____touse__refimu_T_currimu.end(), ':', ';');
        buffer << ";touse__refimu_T_currimu="+string____touse__refimu_T_currimu<< ";";
        MiscUtils::append_status_image( dst, buffer.str() , 1.0 );

        // MiscUtils::append_status_image( dst, "^^^im_ref;reprojecting 3d pts of im_curr on im_ref using initial guess of rel-pose;");
        // MiscUtils::append_status_image( dst, "touse__ref_T_curr="+PoseManipUtils::prettyprintMatrix4d(touse__ref_T_curr) );
        // MiscUtils::append_status_image( dst, "touse__refimu_T_currimu="+PoseManipUtils::prettyprintMatrix4d(_tmp) );
        // cv::imshow( "(after)reprojecting 3d pts of curr on ref using initial guess of rel-pose", dst );
        // char key = cv::waitKey(0);
        cv::resize(dst, rep_image_4, cv::Size(), 0.5, 0.5 ); //selected edges



        // concatenate  [1 ; 2 ]
        cv::Mat tmp_1_and_2;
        MiscUtils::vertical_side_by_side( im_curr_resized, dst_2_resized, tmp_1_and_2 );


        // concatenate [ 3 ; 4]
        cv::Mat tmp_3_and_4;
        MiscUtils::vertical_side_by_side( rep_im_3, rep_image_4, tmp_3_and_4 );


        //  [ 1_and_2 , 3_and_4]
        cv::Mat _4_x_4_;
        if( tmp_1_and_2.rows > tmp_3_and_4.rows )
        {
            int diff = tmp_1_and_2.rows - tmp_3_and_4.rows;
            cv::Mat padding = cv::Mat::zeros( diff, tmp_3_and_4.cols, tmp_3_and_4.type() );

            cv::Mat _padded_3_and_4;
            MiscUtils::vertical_side_by_side( tmp_3_and_4, padding, _padded_3_and_4 );

            MiscUtils::side_by_side( tmp_1_and_2, _padded_3_and_4 , _4_x_4_);
        }

        if( tmp_1_and_2.rows < tmp_3_and_4.rows )
        {
            int diff = - tmp_1_and_2.rows + tmp_3_and_4.rows;
            cv::Mat padding = cv::Mat::zeros( diff, tmp_1_and_2.cols, tmp_1_and_2.type() );

            cv::Mat _padded_1_and_2;
            MiscUtils::vertical_side_by_side( tmp_1_and_2, padding, _padded_1_and_2 );

            MiscUtils::side_by_side( _padded_1_and_2, tmp_3_and_4, _4_x_4_ );
        }

        if( tmp_1_and_2.rows == tmp_3_and_4.rows )
        {
            MiscUtils::side_by_side( tmp_1_and_2, tmp_3_and_4, _4_x_4_ );

        }

        //
        // cv::imshow( "1 and 2", tmp_1_and_2 );
        // cv::imshow( "3 and 4", tmp_3_and_4 );
        // cv::imshow( "4x4", _4_x_4_ );
        // cv::waitKey(0);


        // MiscUtils::imshow( "edge_map of ref image", edge_map , 0.5);
        // MiscUtils::imshow( "edge_map of curr image", im_curr_edgemap, 0.5 );



        // finally,
        this->representation_image = _4_x_4_;
        this->is_representation_image_ready = true;

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
    #if 0
    cv::GaussianBlur( input, _blur, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
    #else
    _blur = input;
    #endif
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
    // out_edge_map = dst;
    out_edge_map = detected_edges;


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



// beware enabling this will also imshow
// #define __EdgeAlignment__get_cX_at_edge_pts( msg ) msg;
#define __EdgeAlignment__get_cX_at_edge_pts( msg ) ;

// #define __EdgeAlignment__get_cX_at_edge_pts_profiling( msg ) msg;
#define __EdgeAlignment__get_cX_at_edge_pts_profiling( msg ) ;
MatrixXd EdgeAlignment::get_cX_at_edge_pts( const cv::Mat im, const cv::Mat depth_map,
            cv::Mat& out_edgemap, cv::Mat& out_selected_pts_edgemap )
{
    assert( !im.empty() && !depth_map.empty() );
    assert( im.rows == depth_map.rows && im.cols == depth_map.cols );
    assert( depth_map.type() == CV_16UC1 || depth_map.type() == CV_32FC1 );
    assert( cam );

    //---- get edgemap with canny
    __EdgeAlignment__get_cX_at_edge_pts_profiling( ElapsedTime t_canny( "Canny Edge Detection"); )
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

    _blur.copyTo( dst, detected_edges); // dest , mask
    out_edgemap = detected_edges;
    __EdgeAlignment__get_cX_at_edge_pts_profiling( cout << TermColor::uGREEN() << t_canny.toc() << TermColor::RESET() << endl; )

    __EdgeAlignment__get_cX_at_edge_pts( cv::imshow( "edge map get_cX_dst", dst ); )
    __EdgeAlignment__get_cX_at_edge_pts( cv::imshow( "edge map get_cX_detected_edges", detected_edges ); )

    // cout << "dst:" << MiscUtils::cvmat_info( dst ) << "\t" << MiscUtils::cvmat_minmax_info( dst ) << endl;
    // cout << "detected_edges:" << MiscUtils::cvmat_info( detected_edges ) << "\t" << MiscUtils::cvmat_minmax_info( detected_edges ) << endl;

    //---- convolution of the edge mask to get weights
    __EdgeAlignment__get_cX_at_edge_pts_profiling( ElapsedTime t_conv("edge_importance_computation"); )
    cv::Mat edge_importance;
    cv::Mat kernel = cv::Mat::ones( 21, 21 , CV_32FC1 ) / (21*21);
    cv::filter2D( detected_edges, edge_importance, CV_8UC1, kernel );
    __EdgeAlignment__get_cX_at_edge_pts_profiling( cout << TermColor::uGREEN() << t_conv.toc() << TermColor::RESET() << endl; )
    __EdgeAlignment__get_cX_at_edge_pts(
    cout << "edge_importance:" << MiscUtils::cvmat_info( edge_importance ) << "\t" << MiscUtils::cvmat_minmax_info( edge_importance ) << endl;
    cv::Mat  edge_importance_false_color_map;
    cv::applyColorMap( edge_importance, edge_importance_false_color_map, cv::COLORMAP_JET  );
    cv::imshow( "edge map get_cX_edge_importance_false_color_map", edge_importance_false_color_map );
    cv::imshow( "edge map get_cX_edge_importance", edge_importance );
    )
    out_selected_pts_edgemap = cv::Mat::zeros( out_edgemap.rows, out_edgemap.cols, CV_8UC1 );


    //---- loop over all the pixels and process only at edge pointst
    __EdgeAlignment__get_cX_at_edge_pts_profiling( ElapsedTime t_process_selected( "loop over all the pixels and process only at edge point"); )
    vector<cv::Point3f> vec_of_pt;
    for( int v=0 ; v< im.rows ; v++ )
    {
        for( int u=0 ; u<im.cols ; u++ )
        {

            //skip because this is not an edge point
            if( dst.at<uchar>(v,u) < 10 )
                continue;


            float depth_val;
            {
                // Depth value computation
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
            }


            // skip based on depth far and near
            if( depth_val < 0.5 || depth_val > 5. )
                continue;


            // skip points which are un-important (ones which have too many neighbours as edgepts).
            if( edge_importance.at<uchar>(v,u) > 50 )
                continue;


            Vector3d _1P;
            Vector2d _1p; _1p << u, v;
            cam->liftProjective( _1p, _1P );

            cv::Point3f pt;
            pt.x = depth_val * _1P(0);
            pt.y = depth_val * _1P(1);
            pt.z = depth_val;
            vec_of_pt.push_back( pt );

            out_selected_pts_edgemap.at<uchar>( v, u ) = 255;

        }
    }
    __EdgeAlignment__get_cX_at_edge_pts_profiling( cout << TermColor::uGREEN() << t_process_selected.toc() << TermColor::RESET() << endl; )


    // --- randomly drop, only retain 1500
    const int n_retain = 1000;
    auto rng = std::default_random_engine {};
    std::shuffle(std::begin(vec_of_pt), std::end(vec_of_pt), rng);

    std::vector<cv::Point3f> slice_of_x;
    if( vec_of_pt.size() > n_retain )
        slice_of_x = std::vector<cv::Point3f>(vec_of_pt.begin(), vec_of_pt.begin() + n_retain );
    else
        slice_of_x = vec_of_pt;




    __EdgeAlignment__get_cX_at_edge_pts_profiling( ElapsedTime t_point3f_2_eigen( "point3f_2_eigen"); )
    MatrixXd cX;
    MiscUtils::point3f_2_eigen( slice_of_x, cX );
    __EdgeAlignment__get_cX_at_edge_pts(
    cout << "vec_of_pt.size() = " << vec_of_pt.size() << endl;
    cout << "slice_of_x.size() = " << slice_of_x.size() << endl;
    cout << "cX.shape=" << cX.rows() << "x" << cX.cols() << endl;
    )
    __EdgeAlignment__get_cX_at_edge_pts_profiling( cout << TermColor::uGREEN() << t_point3f_2_eigen.toc() << TermColor::RESET() << endl; )



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




bool EdgeAlignment::save_to_disk( const string PREFIX, const Matrix4d& initial_guess____ref_T_curr ) const
{
    cout << TermColor::iWHITE() << "[EdgeAlignment::save_to_disk] PREFIX=" << PREFIX << TermColor::RESET() << endl;

    // save images
    cv::imwrite( PREFIX+"_im_ref.jpg", im_ref );
    cv::imwrite( PREFIX+"_im_curr.jpg", im_curr );

    // save depth
    cv::FileStorage storage(PREFIX+"_depth_curr.yaml", cv::FileStorage::WRITE);
    storage << "depth_curr" << depth_curr;


    // save initial_guess____ref_T_curr
    cv::Mat initial_guess____ref_T_curr__opencv;
    cv::eigen2cv( initial_guess____ref_T_curr, initial_guess____ref_T_curr__opencv );
    storage << "initial_guess____ref_T_curr" << initial_guess____ref_T_curr__opencv ;

    storage.release();
    cout << TermColor::iWHITE() << "[EdgeAlignment::save_to_disk] DONE" << TermColor::RESET() << endl;

    return true;
}
