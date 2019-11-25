#pragma once
// Form a local bundle and compte the relative pose between 2 sequences
//     a0---a1---a2----...... ---an
//
//      b0--b1--b2--...bm
//  Also have correspondences and depths at several randomly picked pairs



#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>
#include <memory> //for std::shared_ptr
#include <map>
using namespace std;

// My utilities (mpkuse)
#include "utils/TermColor.h"
#include "utils/ElapsedTime.h"
#include "utils/PoseManipUtils.h"
#include "utils/RawFileIO.h"

#include "utils/MiscUtils.h"
#include "utils/PointFeatureMatching.h"

// JSON
#include "utils/nlohmann/json.hpp"
using json = nlohmann::json;

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

// Ceres
#include <ceres/ceres.h>
using namespace ceres;


// Camodocal
#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"



class LocalBundle
{
public:
    LocalBundle();
    ~LocalBundle() {
        // free
        deallocate_optimization_vars();
    }


    //-----------------//
    //----- Input -----//
    //-----------------//

    // The Odometry data for each frame in sequence-x. x can be a, b, c....
    void inputOdometry( int seqJ, vector<Matrix4d> _x0_T_c );

    // The initial guess between the sequences
    void inputInitialGuess( int seqa, int seqb, Matrix4d a0_T_b0 );

    // the relatuve pose between the sequences from using osometry
    void inputOdometry_a0_T_b0( int seqa, int seqb, Matrix4d odom_a0_T_b0 );


    void inputFeatureMatches( int seq_a, int seq_b,
        const vector<MatrixXd> all_normed_uv_a, const vector<MatrixXd> all_normed_uv_b );
    void inputFeatureMatchesDepths( int seq_a, int seq_b,
        const vector<VectorXd> all_d_a, const vector<VectorXd> all_d_b, const vector<VectorXd> all_sf );
    void inputFeatureMatchesPoses( int seq_a, int seq_b,
        const vector<Matrix4d> all_a0_T_a, const vector<Matrix4d> all_b0_T_b );


    // only for debug
    void inputFeatureMatchesImIdx( int seq_a, int seq_b, vector< std::pair<int,int> > all_pair_idx );
    void inputOdometryImIdx( int seqJ, vector<int> odom_seqJ_idx );
    void inputSequenceImages( int seqJ, vector<cv::Mat> images_seq_j );

    void print_inputs_info() const;

    // save state to json
    json odomSeqJ_toJSON( int j ) const ;
    json matches_SeqPair_toJSON( int seq_a, int seq_b ) const ;
    void toJSON( const string BASE) const;


    // read state from json
    bool odomSeqJ_fromJSON(const string BASE, int j);
    bool matches_SeqPair_fromJSON(const string BASE, int seqa, int seqb);
    void fromJSON( const string BASE ) ;


    //------------------//
    //----- Solver -----//
    //------------------//

public:
    void solve();

private:
    std::map< int, double * > opt_var_qxqyqzqw, opt_var_xyz;
    bool m_opt_vars_allocated = false;
    void allocate_and_init_optimization_vars();
    void deallocate_optimization_vars();

    double * get_raw_ptr_to_opt_variable_q(int seqID, int u ) const;
    double * get_raw_ptr_to_opt_variable_t(int seqID, int u ) const;

    void set_params_constant_for_seq( int seqID, ceres::Problem& problem );
    void add_odometry_residues( ceres::Problem& problem );
    void add_correspondence_residues( ceres::Problem& problem );

    #if 0 //TODO removal, donest work as expected,
    void add_batched_correspondence_residues( ceres::Problem& problem );
    #endif


    //--------------------//
    //----- Retrive ------//
    //-------------------//
public:
    // Call this after optimization. Used to get the relative pose between the series.
    //  Prams
    //      seqID_0, seqID_1: The sequence id. Usually you want to put 0, 1 respectively.
    //      frame_0, frame_1: frame_0 is the index of frame in seqID_0. Similarly, frame_1 is the index of frame in seqID_1.
    //  return
    //      frame0_T_frame1
    Matrix4d retrive_optimized_pose( int seqID_0, int frame_0, int seqID_1, int frame_1 ) const;


public:
    //------------------------//
    //----- Verification -----//
    //------------------------//
    void reprojection_test_for_this_image_pair( const camodocal::CameraPtr camera, int im_pair_idx );
    void reprojection_test( const camodocal::CameraPtr camera );


    void reprojection_error_for_this_image_pair( const camodocal::CameraPtr camera, int im_pair_idx );
    void reprojection_error( const camodocal::CameraPtr camera );


    // save images to file

    void reprojection_debug_images_for_this_image_pair( const camodocal::CameraPtr camera, int im_pair_idx,
        cv::Mat& _dst_observed_correspondence_,  cv::Mat& _dst_image_a, cv::Mat& _dst_image_b );
    void reprojection_debug_images_to_disk( const camodocal::CameraPtr camera, const string PREFIX );


private:
    map< int , vector<Matrix4d> > x0_T_c; //pose of camera wrt its 1st frame (ie. 0th frame)

    map< std::pair<int,int> ,  Matrix4d > a0_T_b0; //initial guess of the relative pose between frame-0 of 2 sequences.
    map< std::pair<int,int> ,  Matrix4d > odom__a0_T_b0;

    map< std::pair<int,int>,  vector<MatrixXd>   > normed_uv_a;
    map< std::pair<int,int>,  vector<VectorXd>   > d_a;
    map< std::pair<int,int>,  vector<Matrix4d>   > a0_T_a;

    map< std::pair<int,int>,  vector<MatrixXd>   > normed_uv_b;
    map< std::pair<int,int>,  vector<VectorXd>   > d_b;
    map< std::pair<int,int>,  vector<Matrix4d>   > b0_T_b;

    // debug
    map< int, vector<int> > seq_x_idx;
    map< std::pair<int,int> , vector< std::pair<int,int> >  >all_pair_idx;
    map< int, vector<cv::Mat> > seq_x_images;

};






class SixDOFError
{
public:
    SixDOFError( const Matrix4d& _observed__c1_T_c2, const double _weight=1.0 ) : observed__c1_T_c2( _observed__c1_T_c2 )
    {
        observed_c1_q_c2 = Quaterniond( _observed__c1_T_c2.topLeftCorner<3,3>() );
        observed_c1_t_c2 << _observed__c1_T_c2(0,3), _observed__c1_T_c2(1,3), _observed__c1_T_c2(2,3);

        weight = _weight;
    }


    // q1, t1 : w_T_c1
    // q2, t2 : w_T_c2
    template <typename T>
    bool operator() ( const T* const q1, const T* const t1,   const T* const q2, const T* const t2, T* residue_ptr ) const
    {
        // Eigen:
        // Note the order of the arguments: the real w coefficient first,
        // while internally the coefficients are stored in the following order: [x, y, z, w]

        // q1,t1 --> w_T_c1
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_1( t1 );
        Eigen::Map<const Eigen::Quaternion<T> > q_1( q1 );

        // q2,t2 --> w_T_c2
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_2( t2 );
        Eigen::Map<const Eigen::Quaternion<T> > q_2( q2 );

        // relative transforms between the 2 frames
        Quaternion<T> q_1_inverse = q_1.conjugate();
        Quaternion<T> q_12_estimated = q_1_inverse * q_2;
        Eigen::Matrix<T,3,1> p_12_estimated = q_1_inverse * (p_2 - p_1);

        // compute error between orientations estimates
        Quaternion<T> delta_q = q_12_estimated.conjugate() * observed_c1_q_c2.cast<T>();
        Eigen::Matrix<T,3,1> delta_t = q_12_estimated.conjugate() * ( observed_c1_t_c2.cast<T>() - p_12_estimated );


        Eigen::Map< Eigen::Matrix<T,6,1> > residuals( residue_ptr );
        residuals.block(0,0,  3,1) =  delta_t;
        residuals.block(3,0,  3,1) =  T(5.0) * delta_q.vec();


        // Dynamic Covariance Scaling
        // T phi = T(5.0);
        // T s = T(2.)*phi / ( phi + residuals.squaredNorm() );
        T s = T(1.0);
        residuals *= (s * T(weight));
        return true;

    }


    static ceres::CostFunction* Create( const Matrix4d& _observed__c1_T_c2, const double weight=1.0 )
    {
      return ( new ceres::AutoDiffCostFunction<SixDOFError,6,4,3,4,3>
        (
          new SixDOFError(_observed__c1_T_c2, weight )
        )
      );
    }


private:


    Matrix4d observed__c1_T_c2;
    Quaterniond observed_c1_q_c2;
    Eigen::Matrix<double,3,1> observed_c1_t_c2;

    double weight;

};



class ProjectionError
{
public:

    ProjectionError( const Vector4d l_X, const Vector3d m_u ): l_3d( l_X), m_2d(m_u)
    {
    }

    // minimize || PI( w_T_l * w_T_m * l_3d ) - m_u ||
    // q1, t1 : w_T_l
    // q2, t2 : w_T_m
    template <typename T>
    bool operator() ( const T* const q1, const T* const t1,   const T* const q2, const T* const t2, T* residue_ptr ) const
    {
        // q1,t1 --> w_T_l
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_1( t1 );
        Eigen::Map<const Eigen::Quaternion<T> > q_1( q1 );
        Eigen::Matrix<T,4,4> w_T_l = Eigen::Matrix<T,4,4>::Identity();
        w_T_l.topLeftCorner(3,3) = q_1.toRotationMatrix();
        w_T_l.col(3).topRows(3) = p_1;

        // q2,t2 --> w_T_m
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_2( t2 );
        Eigen::Map<const Eigen::Quaternion<T> > q_2( q2 );
        Eigen::Matrix<T,4,4> w_T_m = Eigen::Matrix<T,4,4>::Identity();
        w_T_m.topLeftCorner(3,3) = q_2.toRotationMatrix();
        w_T_m.col(3).topRows(3) = p_2;

        // Eigen::Matrix<T,4,1> l_3d_homogeneous;
        // l_3d_homogeneous << l_3d.cast<T>(), T(1.0);
        // auto m_3d = w_T_m.inverse() * w_T_l * l_3d_homogeneous;

        auto m_3d = w_T_m.inverse() * w_T_l * l_3d.cast<T>();
        residue_ptr[0] = T(m_2d(0)) - m_3d(0)/m_3d(2);
        residue_ptr[1] = T(m_2d(1)) - m_3d(1)/m_3d(2);

        #if 1 //if you enable switch constraint remember to set 3 instead of 2 to output number of residues in Create()
        T lambda = T(.5);
        T delta = residue_ptr[0]*residue_ptr[0] + residue_ptr[1]*residue_ptr[1];
        T s = lambda / ( T(1.0) + delta );
        residue_ptr[0] *=s;
        residue_ptr[1] *=s;
        residue_ptr[2] = lambda * (   T(1.0) - s  );
        #endif

        #if 0
        // scale the loss to make it more important relative to odometry loss.
        if( m_3d(2) > T(0.2)  && m_3d(2) < T(3) ) {
            residue_ptr[0] *= T(4.);
            residue_ptr[1] *= T(4.);
        }

        if( m_3d(2) > T(3) && m_3d(2) < T(5) ) {
            residue_ptr[0] *= T(2.);
            residue_ptr[1] *= T(2.);
        }
        #endif


        return true;

    }


        static ceres::CostFunction* Create( const Vector4d l_X, const Vector3d m_u, const double weight=1.0 )
        {
          return ( new ceres::AutoDiffCostFunction<ProjectionError,3,4,3,4,3>
            (
              new ProjectionError(l_X, m_u )
            )
          );
        }


private:
    // now in homogeneous co-ordinates
    Vector4d l_3d;
    Vector3d m_2d;

};


// TODO removal
#if 0
class BatchProjectionError
{
public:

    BatchProjectionError( const MatrixXd l_X, const MatrixXd m_u, const VectorXd ptwise_weights, const double _lambda_ ):
        l_3d( l_X ), m_2d( m_u ), pwt( ptwise_weights ), lambda( _lambda_ )
    {
        // l_X: 4xN
        // m_u: 3xN
        assert( l_X.rows() == 4 && m_u.rows() == 3 );
        assert( l_X.cols() == m_u.cols() && l_X.cols() == ptwise_weights.size() && l_X.cols() > 0 );

    }

    // minimize \sum_i || PI( w_T_l * w_T_m * l_3d[i] ) - m_u[i] ||
    // q1, t1 : w_T_l
    // q2, t2 : w_T_m
    template <typename T>
    bool operator() ( const T* const q1, const T* const t1,   const T* const q2, const T* const t2, T* residue_ptr ) const
    {
        // q1,t1 --> w_T_l
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_1( t1 );
        Eigen::Map<const Eigen::Quaternion<T> > q_1( q1 );
        Eigen::Matrix<T,4,4> w_T_l = Eigen::Matrix<T,4,4>::Identity();
        w_T_l.topLeftCorner(3,3) = q_1.toRotationMatrix();
        w_T_l.col(3).topRows(3) = p_1;

        // q2,t2 --> w_T_m
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_2( t2 );
        Eigen::Map<const Eigen::Quaternion<T> > q_2( q2 );
        Eigen::Matrix<T,4,4> w_T_m = Eigen::Matrix<T,4,4>::Identity();
        w_T_m.topLeftCorner(3,3) = q_2.toRotationMatrix();
        w_T_m.col(3).topRows(3) = p_2;


        auto m_3d = w_T_m.inverse() * w_T_l * l_3d.cast<T>();

        // TODO try sqrt()
        int n =0;
        for( auto i=0 ; i<m_2d.cols() ; i++ )
        {
            if( abs(m_3d(2,i)) < T(1e-3) )
                continue;
            auto del_X = T( m_2d(0,i) ) - m_3d(0,i)/m_3d(2,i);
            auto del_Y = T( m_2d(1,i) ) - m_3d(1,i)/m_3d(2,i);

            residue_ptr[0] += (del_X*del_X + del_Y*del_Y) * T(pwt(i));
            n++;
        }
        residue_ptr[0] /= T( n );
        residue_ptr[0] *= T(lambda);

        // residue_ptr[0] = m_3d(0,0) - T( lambda );

        return true;
    }

    static ceres::CostFunction* Create( MatrixXd l_X, MatrixXd m_u, VectorXd ptwise_weights,  double _lambda_ )
    {
        assert( l_X.rows() == 4 && m_u.rows() == 3 );
        assert( l_X.cols() == m_u.cols() && l_X.cols() == ptwise_weights.size() && l_X.cols() > 0 );
        assert( _lambda_ > 0 );
        return ( new ceres::AutoDiffCostFunction<BatchProjectionError,1,4,3,4,3>
          (
            new BatchProjectionError(l_X, m_u, ptwise_weights, _lambda_ )
          )
        );
    }



private:
    MatrixXd l_3d, m_2d;
    VectorXd pwt;
    double lambda;

};
#endif
