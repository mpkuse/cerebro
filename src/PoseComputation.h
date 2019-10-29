#pragma once
// This class will contain all my pose computation methods, global as well as refinement based
// or any other based on normals

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>
#include <memory> //for std::shared_ptr
using namespace std;

// My utilities (mpkuse)
#include "utils/TermColor.h"
#include "utils/ElapsedTime.h"
#include "utils/PoseManipUtils.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

// Ceres
#include <ceres/ceres.h>
using namespace ceres;

class PoseComputation
{
public:
    // Given two point sets computes the relative transformation using a
    // closed form SVD based method. implementation is based on
    // `A  Comparison  of Four  Algorithms  forEstimating  3-D Rigid  Transformations,
    // BMVC1995 (closed form 3d align SVD, 4 algos compared, especially see section 2.1, Arun et al. TPAMI1987 )`
    // Solves: minimize_{a_T_b}   || aX - a_T_b*bX ||_2
    // NOTE: This does not need an initial guess. so b_T_a is a just output.
    //      aX : 3xN or 4xN 3d points in co-ordinates system of `a`
    //      bX : 3xN or 4xN 3d points in co-ordinates system of `b`
    //      b_T_a [output] : resulting pose between the co-ordinates. Pose of a as observed from b.
    static bool closedFormSVD( const MatrixXd& aX, const MatrixXd& bX, Matrix4d& a_T_b );
    static bool closedFormSVD( const MatrixXd& aX, const MatrixXd& bX, const VectorXd& sf, Matrix4d& a_T_b ); //same as above but is equiped to take it weight of each sample

    // Given 2 point sets and an initial guess of the alignment iteratively refine the estimate
    // The problem is setup as non-linear least squares with robust norm and switching constraints.
    // Solves: minimize_{a_T_b, s1,s2,...sn} \sum_i s_i * || aX_i - a_T_b*bX_i ||_2 +  \lambda*(1-s_i)
    //      s1,s2,... are scalars between [0,1], \lambda is a fixed scalar
    //      aX : 3xN or 4xN 3d points in co-ordinates system of `a`
    //      bX : 3xN or 4xN 3d points in co-ordinates system of `b`
    //      b_T_a [input/output] : initial guess. resulting pose between the co-ordinates. Pose of a as observed from b.
    //      sf [input/output]: The switches after the optimization has terminated. If the size of this is equal to N
    //                       then will use this to initialize the switch flags. else will use 1.0 as initialization for the switches
    // ^^ implement if need be, see github.com/mpkuse/gmm_pointcloud_align
    static bool refine( const MatrixXd& aX, const MatrixXd& bX, Matrix4d& a_T_b, VectorXd& sf );
    static bool refine_weighted( const MatrixXd& aX, const MatrixXd& bX, Matrix4d& a_T_b, const VectorXd& sf );



    // Given 2 point sets and a transform, test how good the transform is
    static void testTransform( const MatrixXd& aX, const MatrixXd& bX, const Matrix4d& a_T_b );

    // This implements the alternating minimization approach. (see http://curtis.ml.cmu.edu/w/courses/index.php/Alternating_Minimization)
    // Particularly the 5 point property garuntees the convergence of AM method. (http://www.mit.edu/~6.454/www_fall_2002/shaas/Csiszar.pdf)
    // a. start with some initial guess on switch_weights
    // b. loop until convergence
    //      1. get optimal value of a_T_b assume switch_weights as constant
    //      2. using the a_T_b from previous step and using it as constant, get optimal value of the switch weights
    //  Params:
    //      aX [input] : 3xN or 4xN 3d points in co-ordinates system of `a`
    //      bX [input] : 3xN or 4xN 3d points in co-ordinates system of `b`
    //      a_T_b [output]: no need to supply an initial guess.
    //      switch_weights[input/output] : will use this as initial weights, this will at the end containt the final weights
    //  Returns:
    //      In case of convergence, the fraction of switches (0.75-1.0), think of the output as the confidence. In case of non-convergence returns -1.
    static float alternatingMinimization( const MatrixXd& aX, const MatrixXd& bX, Matrix4d& a_T_b, VectorXd& switch_weights );

private:
    static void print_info_on_switch_weights( const VectorXd& switch_weights);
    static void quantile_info_on_switch_weights( const VectorXd& switch_weights, VectorXi& quantile, const int n_quantiles );


};




//---------------------------------------------------//
//------------ Ceres Cost Functions -----------------//
//---------------------------------------------------//


class EuclideanDistanceResidue {
public:
    EuclideanDistanceResidue( const Vector3d& Xi, const Vector3d& Xid, const double wt=1.0 )
    {
        this->Xi = Xi;
        this->Xid = Xid;
        this->weight = sqrt(wt);
    }

    template <typename T>
    bool operator()( const T* const q, const T* const t , T* residual ) const {
        // Optimization variables
        Quaternion<T> eigen_q( q[0], q[1], q[2], q[3] );
        Eigen::Matrix<T,3,1> eigen_t;
        eigen_t << t[0], t[1], t[2];


        // Known Constant
        Eigen::Matrix<T,3,1> eigen_Xi, eigen_Xid;
        eigen_Xi << T(Xi(0)), T(Xi(1)), T(Xi(2));
        eigen_Xid << T(Xid(0)), T(Xid(1)), T(Xid(2));



        // Error term
        Eigen::Matrix<T,3,1> e;
        e = eigen_Xi - (  eigen_q.toRotationMatrix() * eigen_Xid + eigen_t );

        residual[0] = T(this->weight) * e(0);
        residual[1] = T(this->weight) * e(1);
        residual[2] = T(this->weight) * e(2);

        return true;
    }



    static ceres::CostFunction* Create(const Vector3d& _Xi, const Vector3d& Xid, const double wt=1.0)
    {
        return ( new ceres::AutoDiffCostFunction<EuclideanDistanceResidue,3,4,3>
        (
        new EuclideanDistanceResidue(_Xi,Xid, wt)
        )
        );
    }

    private:
    Vector3d Xi, Xid;
    double weight;
};





class EuclideanDistanceResidueSwitchingConstraint  {
public:
    // lambda is the switch-penalty.
    EuclideanDistanceResidueSwitchingConstraint( const Vector3d& Xi, const Vector3d& Xid, const double lambda_=3.0 )
    {
        this->Xi = Xi;
        this->Xid = Xid;
        this->lambda = lambda_;
    }

    template <typename T>
    bool operator()( const T* const q, const T* const t , const T* const s, T* residual ) const {
        // Optimization variables
        Quaternion<T> eigen_q( q[0], q[1], q[2], q[3] );
        Eigen::Matrix<T,3,1> eigen_t;
        eigen_t << t[0], t[1], t[2];


        // Known Constant
        Eigen::Matrix<T,3,1> eigen_Xi, eigen_Xid;
        eigen_Xi << T(Xi(0)), T(Xi(1)), T(Xi(2));
        eigen_Xid << T(Xid(0)), T(Xid(1)), T(Xid(2));



        // Error term

        Eigen::Matrix<T,3,1> e;
        e = eigen_Xi - (  eigen_q.toRotationMatrix() * eigen_Xid + eigen_t );

        residual[0] = s[0] * e(0);
        residual[1] = s[0] * e(1);
        residual[2] = s[0] * e(2);
        residual[3] = T(lambda) * (T(1.0) - s[0]);

        return true;
    }

    static ceres::CostFunction* Create(const Vector3d& _Xi, const Vector3d& Xid)
    {
        return ( new ceres::AutoDiffCostFunction<EuclideanDistanceResidueSwitchingConstraint,4,   4,3,1>
          (
            new EuclideanDistanceResidueSwitchingConstraint(_Xi,Xid)
          )
        );
    }

private:
    Vector3d Xi, Xid;
    double lambda;
};
