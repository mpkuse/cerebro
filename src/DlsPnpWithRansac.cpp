
#include "DlsPnpWithRansac.h"


#ifdef __USE_THEIASFM
// Theia's ICP
// [Input]
//      uv_X: a 3d point cloud expressed in some frame-of-ref, call it frame-of-ref of `uv`
//      uvd_Y: a 3d point cloud expressed in another frame-of-ref, call it frame-of-ref of `uvd`
// [Output]
//      uvd_T_uv: Relative pose between the point clouds
// [Note]
//      uv_X <---> uvd_Y
// #define ____P3P_ICP_( msg ) msg;
#define ____P3P_ICP_( msg ) ;
float StaticTheiaPoseCompute::P3P_ICP( const vector<Vector3d>& uv_X, const vector<Vector3d>& uvd_Y,
    Matrix4d& uvd_T_uv, string & p3p__msg )
{
    if( uv_X.size() < 20 ) {
        ____P3P_ICP_( cout << "[StaticTheiaPoseCompute::P3P_ICP] Too few input points. You provided " << uv_X.size() << endl; )
        return -1; // if you give me too few points, i return error.
    }

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
        cout << TermColor::RED() << "theia::AlignPointCloudsUmeyama scales doesn't look good;this usually implies that estimation is bad.        scale= " << ___s << endl;
        p3p__msg += "p3p_ICP: scale=" +to_string( ___s )+" {uvd}_T_{uv} : " + PoseManipUtils::prettyprintMatrix4d( uvd_T_uv );
        p3p__msg += "p3p done in (ms)" + to_string(elapsed_time_p3p)+";    theia::AlignPointCloudsUmeyama scales doesn't look good, this usually implies that estimation is bad. scale= " + to_string(___s);
        return -1;
    }

    p3p__msg += "p3p done in (ms)" + to_string(elapsed_time_p3p)+";    p3p_ICP: {uvd}_T_{uv} : " + PoseManipUtils::prettyprintMatrix4d( uvd_T_uv );
    p3p__msg += ";weight="+to_string( min( ___s, 1.0/___s ) );
    return  min( ___s, 1.0/___s );
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
float StaticTheiaPoseCompute::PNP( const std::vector<Vector3d>& w_X, const std::vector<Vector2d>& c_uv_normalized,
    Matrix4d& c_T_w,
    string& pnp__msg )
{
    if( w_X.size() < 20 ) {
        ___P_N_P__( cout << "[StaticTheiaPoseCompute::PNP] Too few input points. You provided " << w_X.size() << endl; )
        return -1; // if you give me too few points, i return error.
    }
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
    params.error_thresh = 0.03;
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

#endif



// #define __StaticCeresPoseCompute_PNP_(msg) msg;
#define __StaticCeresPoseCompute_PNP_(msg) ;
float StaticCeresPoseCompute::PNP( const std::vector<Vector3d>& w_X, const std::vector<Vector2d>& c_uv_normalized,
    Matrix4d& c_T_w,
    string& pnp__msg )
{
    assert( w_X.size() == c_uv_normalized.size() && w_X.size() > 8 );
    if( w_X.size() != c_uv_normalized.size() ) {
        cout << "[StaticCeresPoseCompute::PNP] w_X.size() != c_uv_normalized.size()\n";
        exit(  1 );
    }
    __StaticCeresPoseCompute_PNP_(
    cout << "w_X.size()=" << w_X.size() << "  c_uv_normalized.size()=" << c_uv_normalized.size() << endl;
    )

    // setup ceres problem for PNP
    //      minimize_{R,t} \sum_i (  PI( c_(R|t)_w * w_X[i] ) - u[i] )
    ceres::Problem problem;

    // optimization variables
    double ypr[3], tr[3];
    PoseManipUtils::eigenmat_to_rawyprt( c_T_w, ypr, tr );
    // double yaw=ypr[0], pitch=ypr[1], roll=ypr[2], tx=tr[0], ty=tr[1], tz=tr[2];

    for( int i=0 ; i<w_X.size() ; i++ )
    {
        // cout << w_X[i].transpose() << " <---> " << c_uv_normalized[i].transpose() << endl;
        ceres::CostFunction * cost_function = PNPEulerAngleError::Create( w_X[i], c_uv_normalized[i] );
        problem.AddResidualBlock( cost_function, new ceres::HuberLoss(0.1), &ypr[0], &ypr[1], &ypr[2], &tr[0], &tr[1], &tr[2] );
        // problem.AddResidualBlock( cost_function, NULL, &ypr[0], &ypr[1], &ypr[2], &tr[0], &tr[1], &tr[2] );

    }


    // Local Parameterization (Angle)
    ceres::LocalParameterization *angle_parameterization = AngleLocalParameterization::Create();
    problem.SetParameterization( &ypr[0], angle_parameterization );
    problem.SetParameterization( &ypr[1], angle_parameterization );
    problem.SetParameterization( &ypr[2], angle_parameterization );

    // Set as constant
    problem.SetParameterBlockConstant( &ypr[1] ); //pitch
    problem.SetParameterBlockConstant( &ypr[2] ); //roll


    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    // options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    ceres::Solver::Summary summary;
    __StaticCeresPoseCompute_PNP_(
    cout << "yaw,pitch,roll=" << ypr[0] << " " << ypr[1] << " " << ypr[2] << "; tx,ty,tz=" << tr[0] << " "<< tr[1] << " " << tr[2] << endl;
    cout << "Ceres::Solve()\n";
    )

    pnp__msg += "initial_guess: "+PoseManipUtils::prettyprintMatrix4d( c_T_w ) + ";";
    ceres::Solve(options, &problem, &summary);
    __StaticCeresPoseCompute_PNP_(
    cout << "yaw,pitch,roll=" << ypr[0] << " " << ypr[1] << " " << ypr[2] << "; tx,ty,tz=" << tr[0] << " "<< tr[1] << " " << tr[2] << endl;
    )
    // cout << summary.FullReport() << endl;
    cout << summary.BriefReport() << endl;
    pnp__msg += summary.BriefReport() + ";";


    PoseManipUtils::rawyprt_to_eigenmat( ypr, tr, c_T_w );
    pnp__msg += "final: "+PoseManipUtils::prettyprintMatrix4d( c_T_w ) + ";";

    return 1.0;
}






#define __StaticCeresPoseCompute_P3P_(msg) msg;
// #define __StaticCeresPoseCompute_P3P_(msg) ;
float StaticCeresPoseCompute::P3P_ICP( const std::vector<Vector3d>& a_X, const std::vector<Vector3d>& b_X,
    Matrix4d& b_T_a,
    string& p3p__msg )
{
    assert( a_X.size() == b_X.size() && a_X.size() > 8 );
    if( a_X.size() != b_X.size() ) {
        cout << "[StaticCeresPoseCompute::P3P] a_X.size() != b_X.size()\n";
        exit(  1 );
    }
    __StaticCeresPoseCompute_P3P_(
    cout << "a_X.size()=" << a_X.size() << "  b_X.size()=" << b_X.size() << endl;
    )

    // setup ceres problem for PNP
    //      minimize_{R,t} \sum_i (   c_(R|t)_w * a_X[i] - b_X[i]   )
    ceres::Problem problem;

    // optimization variables
    double ypr[3], tr[3];
    PoseManipUtils::eigenmat_to_rawyprt( b_T_a, ypr, tr );
    // double yaw=ypr[0], pitch=ypr[1], roll=ypr[2], tx=tr[0], ty=tr[1], tz=tr[2];

    for( int i=0 ; i<a_X.size() ; i++ )
    {
        // cout << a_X[i].transpose() << " <---> " << b_X[i].transpose() << endl;
        ceres::CostFunction * cost_function = P3PEulerAngleError::Create( a_X[i], b_X[i] );
        problem.AddResidualBlock( cost_function, new ceres::HuberLoss(0.1), &ypr[0], &ypr[1], &ypr[2], &tr[0], &tr[1], &tr[2] );
        // problem.AddResidualBlock( cost_function, NULL, &ypr[0], &ypr[1], &ypr[2], &tr[0], &tr[1], &tr[2] );

    }


    // Local Parameterization (Angle)
    ceres::LocalParameterization *angle_parameterization = AngleLocalParameterization::Create();
    problem.SetParameterization( &ypr[0], angle_parameterization );
    problem.SetParameterization( &ypr[1], angle_parameterization );
    problem.SetParameterization( &ypr[2], angle_parameterization );

    // Set as constant
    problem.SetParameterBlockConstant( &ypr[1] ); //pitch
    problem.SetParameterBlockConstant( &ypr[2] ); //roll


    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    // options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    ceres::Solver::Summary summary;
    __StaticCeresPoseCompute_P3P_(
    cout << "yaw,pitch,roll=" << ypr[0] << " " << ypr[1] << " " << ypr[2] << "; tx,ty,tz=" << tr[0] << " "<< tr[1] << " " << tr[2] << endl;
    cout << "Ceres::Solve()\n";
    )
    p3p__msg += "initial bTa: "+PoseManipUtils::prettyprintMatrix4d( b_T_a ) + ";";
    ceres::Solve(options, &problem, &summary);
    __StaticCeresPoseCompute_P3P_(
    cout << "yaw,pitch,roll=" << ypr[0] << " " << ypr[1] << " " << ypr[2] << "; tx,ty,tz=" << tr[0] << " "<< tr[1] << " " << tr[2] << endl;
    )
    __StaticCeresPoseCompute_P3P_(
    // cout << summary.FullReport() << endl;
    cout << summary.BriefReport() << endl;
    )
    p3p__msg += summary.BriefReport() + ";";

    PoseManipUtils::rawyprt_to_eigenmat( ypr, tr, b_T_a );
    p3p__msg += "final: "+PoseManipUtils::prettyprintMatrix4d( b_T_a ) + ";";

    return 1.0;
}
