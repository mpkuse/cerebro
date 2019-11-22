#pragma once
#include <iostream>

#include <string>
#include <vector>
#include <map>

#include <mutex>
#include <cassert>


#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

#include "utils/TermColor.h"

using namespace std;


class HypothesisManager
{
public:
    HypothesisManager();

    // i<--->j with dot product score and n_nn indicates which nearest neighbour this was the nearest-neighbour, 2nd nearest, 3rd nearest and so on.
    //     Will return true if new hypothesis was added. If no new hypothesis was added will return false
    bool add_node( int i, int j, double dot_product_score, int n_nn );



    void print_hyp_q_all() const;
    int n_hypothesis() const; //number of hypothesis in the list
    bool hypothesis_i(int i, int& seq_a_start, int&  seq_a_end, int&  seq_b_start, int&  seq_b_end ) const; //return the ith hypothesis

private:
    std::map< int , float > M;
    int n_accum = 0;
    int i_start = 0;
    int i_latest = 0;
    int n_greater_than_thresh = 0;



private:
    // tune these:

    int W = 15; //< W is grid size
    float THRESH = 0.85; //< threshold on the dot product
    float LOW_THRESH = 0.75;
    float BOOST_FACTOR_IF_GREATER_THAN_THRESH = 1.5; //< how much to boost the vote if the dot product is higher than threshold
    int FLUSH_AFTER_N_ACCUMULATES = 50; // you usually want to keep this at n_nn x W.

    float MANDATE_SCORE_THRESH = .2*FLUSH_AFTER_N_ACCUMULATES; //40.; //If total voting score is greater than this amount in `FLUSH_AFTER_N_ACCUMULATES` then conclude that this is a loopmatch


private:
    mutable std::mutex mutex_hyp_q;
    std::vector< std::vector<int> > hyp_q; //vector[][4] 4 numbers in each
    bool digest(); // the processing at FLUSH_AFTER_N_ACCUMULATES. Will return true if new hypothesis was added. If no new hypothesis was added will return false


public:
    void set_computed_pose( int i, const Matrix4d a_T_b, const string info_str );
    bool is_computed_pose_available( int i ) const ;
    Matrix4d get_computed_pose( int i ) const ;
    string get_computed_pose_info_string( int i ) const;

private:
    map<int,bool> m_computed_pose;
    map<int,Matrix4d> computed_pose_a_T_b;
    map<int,string> computed_pose_info;

};
