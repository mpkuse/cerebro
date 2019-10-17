#include "PoseComputation.h"

// #define _PoseComputation__closedFormSVD_(msg) msg;
#define _PoseComputation__closedFormSVD_(msg) ;

// #define _PoseComputation__closedFormSVD_debug(msg) msg;
#define _PoseComputation__closedFormSVD_debug(msg) ;
bool PoseComputation::closedFormSVD( const MatrixXd& aX, const MatrixXd& bX, Matrix4d& a_T_b )
{
    assert( aX.rows() == 4 && bX.rows() == 4 && aX.cols() == bX.cols() && aX.cols() > 0 );
    _PoseComputation__closedFormSVD_( ElapsedTime _t; _t.tic();
    cout << TermColor::iGREEN() << "=== PoseComputation::closedFormSVD input size = " << aX.rows() << "x" << aX.cols() << TermColor::RESET() << endl;
    )

    // centroids
    VectorXd cen_aX = aX.rowwise().mean();
    VectorXd cen_bX = bX.rowwise().mean();
    _PoseComputation__closedFormSVD_debug(
    cout << "centroids computed: ";
    cout << "cen_aX = " << cen_aX.rows() << "x" << cen_aX.cols() << "\t" << cen_aX.transpose() << "\n";
    cout << "cen_bX = " << cen_bX.rows() << "x" << cen_bX.cols() << "\t" << cen_bX.transpose() << "\n";
    )


    // aX_cap:= aX - cen_aX
    // bX_cap:= bX - cen_bX
    // H := aX_cap * bX_cap.transpose()
    Matrix3d H = (bX.colwise() - cen_bX).topRows(3) * (aX.colwise() - cen_aX).topRows(3).transpose();
    _PoseComputation__closedFormSVD_debug(cout << "H=" << H.rows() << "x" << H.cols() << endl;)

    // U,S,Vt = svd( H )
    JacobiSVD<Matrix3d> svd( H, ComputeFullU | ComputeFullV);

    _PoseComputation__closedFormSVD_debug(
    cout << "Singular=\n" << svd.singularValues() << endl;
    cout << "U=\n" << svd.matrixU()  << endl;
    cout << "V=\n" << svd.matrixV()  << endl;
    )

    // R := V * Ut. if det(R) is -1, then use R = [v1, v2, -v3] * Ut
    Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    _PoseComputation__closedFormSVD_debug(
    cout << "R=\n" << R << endl;
    cout << "R.det=" << R.determinant() << endl;)

    // assert( abs(R.determinant()-1.0)<1e-6 );
    if( abs(R.determinant()+1.0)<1e-6 ) // then determinant is -1
    {
        Matrix3d _V = svd.matrixV();
        for( int fd=0;fd<3;fd++)
            _V(fd,2) = -_V(fd,2);
        _PoseComputation__closedFormSVD_debug( cout << "_V=\n" << _V; )
        R = _V * svd.matrixU().transpose();
    }

    // translation : mean(aX) - R mean(bX)
    Vector3d tr = aX.rowwise().mean().topRows(3) - R * bX.rowwise().mean().topRows(3);
    _PoseComputation__closedFormSVD_debug(cout << "Translation=" << tr << endl;)


    a_T_b = Matrix4d::Identity();
    a_T_b.topLeftCorner(3,3) = R;
    a_T_b.col(3).topRows(3) = tr;

    _PoseComputation__closedFormSVD_(
    cout << "a_T_b=\n" << a_T_b << endl;
    cout << TermColor::BLUE() << "[PoseComputation::closedFormSVD]computation done in ms=" << _t.toc_milli() << TermColor::RESET() << endl; )
    return true;
}


// #define _PoseComputation__closedFormSVD_weighted___(msg) msg;
#define _PoseComputation__closedFormSVD_weighted___(msg) ;
bool PoseComputation::closedFormSVD( const MatrixXd& aX, const MatrixXd& bX, const VectorXd& sf, Matrix4d& a_T_b )
{
    assert( aX.rows() == 4 && bX.rows() == 4 && aX.cols() == bX.cols() && aX.cols() > 0 );
    assert( sf.size() == bX.cols() );


        //--- centroids (weighted avg)
        Vector4d cen_aX = Vector4d::Zero();
        Vector4d cen_bX = Vector4d::Zero();

        for( int r=0 ; r<4 ; r++ )
            cen_aX.row(r) = (aX.row(r) * sf) / sf.sum();

        for( int r=0 ; r<4 ; r++ )
            cen_bX.row(r) = (bX.row(r) * sf )/ sf.sum();

        _PoseComputation__closedFormSVD_weighted___(
        cout << "centroids computed:\n";
        cout << "cen_aX = " << cen_aX.rows() << "x" << cen_aX.cols() << "\t" << cen_aX.transpose() << "\n";
        cout << "cen_bX = " << cen_bX.rows() << "x" << cen_bX.cols() << "\t" << cen_bX.transpose() << "\n";
        )



        // aX_cap:= aX - cen_aX
        // bX_cap:= bX - cen_bX
        // H := aX_cap * bX_cap.transpose()
        Matrix3d H = (bX.colwise() - cen_bX).topRows(3) * sf.asDiagonal() * (aX.colwise() - cen_aX).topRows(3).transpose();
        _PoseComputation__closedFormSVD_weighted___( cout << "H:\n" << H << endl; )


        // U,S,Vt = svd( H )
        JacobiSVD<Matrix3d> svd( H, ComputeFullU | ComputeFullV);

        _PoseComputation__closedFormSVD_weighted___(
        cout << "Singular=\n" << svd.singularValues() << endl;
        cout << "U=\n" << svd.matrixU()  << endl;
        cout << "V=\n" << svd.matrixV()  << endl;
        )

        // R := V * Ut. if det(R) is -1, then use R = [v1, v2, -v3] * Ut
        Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
        _PoseComputation__closedFormSVD_weighted___(
        cout << "R=\n" << R << endl;
        cout << "R.det=" << R.determinant() << endl;)

        // assert( abs(R.determinant()-1.0)<1e-6 );
        if( abs(R.determinant()+1.0)<1e-6 ) // then determinant is -1
        {
            Matrix3d _V = svd.matrixV();
            for( int fd=0;fd<3;fd++)
                _V(fd,2) = -_V(fd,2);
            _PoseComputation__closedFormSVD_weighted___( cout << "_V=\n" << _V; )
            R = _V * svd.matrixU().transpose();
        }

        // translation : mean(aX) - R mean(bX)
        Vector3d tr = Vector3d::Zero();
        // tr = aX.rowwise().mean().topRows(3) - R * bX.rowwise().mean().topRows(3);
        MatrixXd bX_rot = R * bX.topRows(3);
        for( int g=0;g<3;g++ ) { //weighted sum
            double t1 = (aX.row(g) * sf) ;
            double t2 = (bX_rot.row(g) * sf);
            tr(g) = t1 / sf.sum() -  t2 / sf.sum();
        }
        _PoseComputation__closedFormSVD_weighted___( cout << "Translation=" << tr.transpose() << endl; )


        a_T_b = Matrix4d::Identity();
        a_T_b.topLeftCorner(3,3) = R;
        a_T_b.col(3).topRows(3) = tr;

        _PoseComputation__closedFormSVD_weighted___(
        cout << "a_T_b=\n" << a_T_b << endl;
        cout << TermColor::BLUE() << "[PoseComputation::closedFormSVD:weighted]computation done in ms=" << _t.toc_milli() << TermColor::RESET() << endl;
        )
        return true;
}




void PoseComputation::testTransform( const MatrixXd& aX, const MatrixXd& bX, const Matrix4d& a_T_b )
{
    assert( aX.rows() == 4 && bX.rows() == 4 && aX.cols() == bX.cols() && aX.cols() > 0 );

    MatrixXd diff = aX - a_T_b * bX;

    cout << "[PoseComputation::testTransform]\n";
    cout << "\tn_pts = " << aX.cols() << endl ;
    int n = aX.cols();
    cout << "\tdel_X = " << sqrt(diff.row(0) * diff.row(0).transpose())/n << "\t";
    cout << "\tdel_Y = " << sqrt(diff.row(1) * diff.row(1).transpose())/n << "\t";
    cout << "\tdel_Z = " << sqrt(diff.row(2) * diff.row(2).transpose())/n << "\n";

    // for( int i=0 ; i<n ; i++ ) {
    // cout << "#" << i  << " ";
    // cout << " norm=" << setw(4) << setprecision(3) << diff.col(i).head(3).norm() ;
    // cout << "\tdx,dy,dz=" << diff.col(i).head(3).transpose();
    // cout << endl;;
    // }
}


float PoseComputation::alternatingMinimization( const MatrixXd& aX, const MatrixXd& bX, Matrix4d& a_T_b, VectorXd& switch_weights )
{
    assert( aX.rows() == 4 && bX.rows() == 4 && aX.cols() == bX.cols() && aX.cols() > 0 );
    int N = aX.cols();
    assert( switch_weights.size() == N );

    cout << TermColor::iGREEN() << "[PoseComputation::alternatingMinimization] STARTS, N=" << N << "\n" << TermColor::RESET();

    // a. start with some initial guess on switch_weights
    // b. loop until convergence
    //      1. get optimal value of a_T_b assume switch_weights as constant
    //      2. using the a_T_b from previous step and using it as constant, get optimal value of the switch weights


    // cout <<"initially:\n";
    // print_info_on_switch_weights( switch_weights );


    const double lambda = 1.5;
    int max_iterations = 5;
    VectorXi quantile;
    bool converged = false;

    int n_quantiles = 4; //dont change this as it is liked to the threshold. This means the histogram has to be make with this many bins.

    for( int itr = 0 ; itr<max_iterations ; itr++ )
    {
        cout << TermColor::RED() << "---" << TermColor::RESET() << "itr=" << itr << endl;

        //-- 1. get optimal value of a_T_b assume switch_weights as constant
        closedFormSVD( aX, bX, switch_weights, a_T_b );
        cout << "a_T_b = "<< PoseManipUtils::prettyprintMatrix4d( a_T_b ) << endl;


        //-- 2. using the a_T_b from previous step and using it as constant,
        //      get optimal value of the switch weights
        MatrixXd D = aX - a_T_b * bX;
        for( int k=0; k<N ; k++ ) {
            double delta_sqr = D.col(k).topRows(3).squaredNorm();
            switch_weights(k) = lambda / (lambda + delta_sqr );
        }


        //-- print info on `switch_weights`

        quantile_info_on_switch_weights( switch_weights, quantile, n_quantiles );
        // print_info_on_switch_weights( switch_weights );

        cout << "quantile: " << quantile.cast<float>().transpose() / float(N)  << endl;
        if( quantile(n_quantiles-1) / float(N) > 0.75 ) {
            converged = true;
            break;
        }

    }

    if( converged == true )
        cout << TermColor::GREEN() << "[PoseComputation::alternatingMinimization] Converged\n" << TermColor::RESET();
    else
        cout << TermColor::RED() << "[PoseComputation::alternatingMinimization] NOT Converged\n" << TermColor::RESET();



    cout << TermColor::iGREEN() << "[PoseComputation::alternatingMinimization] END\n" << TermColor::RESET();
    if( converged == true )
        return quantile(n_quantiles-1) / float(N);
    else
        return -1.0;
}


void PoseComputation::print_info_on_switch_weights( const VectorXd& switch_weights)
{
    int N = switch_weights.size();
    int n_quantiles = 4;
    // int * quantile = new int[n_quantiles];
    VectorXi quantile = VectorXi::Zero( n_quantiles );
    for( int i=0; i<N ; i++ ) {
        quantile( (int) ( max( 0.0, (switch_weights(i)-0.001)) * n_quantiles)  )++;

        if( switch_weights(i) < 0.75 ) cout << TermColor::RED() ;
        cout << "s[" << std::setw(4) <<  i << "] = " << std::setw(4) << std::setprecision(2) << switch_weights(i) << "\t";
        cout << TermColor::RESET();
        if( i%10==0) cout << endl;
    }
    cout << std::setprecision(18) << endl;


    cout << "Quantiles range=[0,4] for the switch constraints, n_quantiles=" << n_quantiles<< ", total_points=" << N << ":\n";
    for( int i=0 ; i<n_quantiles; i++ ) cout << "\tquantile[" << i << "] = " << std::setw(5) << quantile[i] << "\tfrac=" << std::setprecision(2) <<  float(quantile[i])/N << endl;

}

void PoseComputation::quantile_info_on_switch_weights( const VectorXd& switch_weights, VectorXi& quantile, const int n_quantiles )
{
    int N = switch_weights.size();
    // int * quantile = new int[n_quantiles];
    quantile = VectorXi::Zero( n_quantiles );
    for( int i=0; i<N ; i++ ) {
        quantile( (int) ( max( 0.0, (switch_weights(i)-0.001)) * n_quantiles)  )++;

    }

}
