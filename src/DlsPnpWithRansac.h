#pragma once

/* This is a RanSAC wrapper for theia::DlsPnp. Based on sample code on theia-sfm website.
    The official website has some obivious issues.

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 4th Jan, 2018
*/


#include "utils/TermColor.h"
#include "utils/ElapsedTime.h"
#include "utils/PoseManipUtils.h"

#include <theia/theia.h>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;


/////////// DlsPnp-RANSAC ///////////////////
// Data
struct CorrespondencePair_3d2d {
    Vector3d a_X; // 3d point expressed in co-ordinate system of `image-a`
    Vector2d uv_d; //feature point in normalized-image co-ordinates. of the other view (b)
};

// Model
struct RelativePose {
    Matrix4d b_T_a;
};


class DlsPnpWithRansac: public theia::Estimator<CorrespondencePair_3d2d, RelativePose> {
public:
    // Number of points needed to estimate a line.
    double SampleSize() const  { return 15; }

    // Estimate a pose from N points
    bool EstimateModel( const std::vector<CorrespondencePair_3d2d>& data,
                            std::vector<RelativePose>* models) const {

        std::vector<Vector2d> feature_position;
        std::vector<Vector3d> world_point;
        for( int i=0 ; i<data.size() ; i++ ) {
            feature_position.push_back( data[i].uv_d );
            world_point.push_back( data[i].a_X );
        }

        std::vector<Quaterniond> solution_rotations;
        std::vector<Vector3d> solution_translations;

        theia::DlsPnp( feature_position, world_point, &solution_rotations, &solution_translations );
        if( solution_rotations.size() == 1 ) {
            RelativePose r;
            r.b_T_a = Matrix4d::Identity();
            r.b_T_a.topLeftCorner(3,3) = solution_rotations[0].toRotationMatrix();
            r.b_T_a.col(3).topRows(3) = solution_translations[0];
            models->push_back( r );
        }
        else {
            return false;
        }
    }


    double Error( const CorrespondencePair_3d2d& point, const RelativePose& r ) const {
        Vector3d  b_X = r.b_T_a.topLeftCorner(3,3) * point.a_X + r.b_T_a.col(3).topRows(3);
        double depth = b_X(2);
        b_X /= b_X(2);

        double reproj_error = abs(b_X(0) - point.uv_d(0)) + abs(b_X(1) - point.uv_d(1));

        // I want nearby points to have smaller Error than far off points.
        // So artificially increase the error when depth is higher for the point.
        double f = 1.;
        #if 0
        if( depth < 1. )
            f = .5;
        if( depth >=1 && depth < 3 )
            f = 5.0;
        if( depth >=3 && depth < 8 )
            f = 2.;
        if( depth >= 8. && depth < 15 )
            f = 1.7;
        if( depth >= 15 )
            f = 0.1;
        #endif

        return reproj_error*f;
    }
};
/////////// END DlsPnp-RANSAC ///////////////////





class StaticTheiaPoseCompute
{
public:
    static float P3P_ICP( const vector<Vector3d>& uv_X, const vector<Vector3d>& uvd_Y,
        Matrix4d& uvd_T_uv, string & p3p__msg );

    static float PNP( const std::vector<Vector3d>& w_X, const std::vector<Vector2d>& c_uv_normalized,
        Matrix4d& c_T_w,
        string& pnp__msg );

};
