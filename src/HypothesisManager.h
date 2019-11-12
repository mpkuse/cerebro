#pragma once
#include <iostream>

#include <string>
#include <vector>
#include <map>

#include <mutex>
#include <cassert>


#include "utils/TermColor.h"

using namespace std;


class HypothesisManager
{
public:
    HypothesisManager();

    // i<--->j with dot product score and n_nn indicates which nearest neighbour this was the nearest-neighbour, 2nd nearest, 3rd nearest and so on.
    void add_node( int i, int j, double dot_product_score, int n_nn );


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

    int W = 30; //< W is grid size
    float THRESH = 0.85; //< threshold on the dot product
    float LOW_THRESH = 0.75;
    float BOOST_FACTOR_IF_GREATER_THAN_THRESH = 1.2; //< how much to boost the vote if the dot product is higher than threshold
    int FLUSH_AFTER_N_ACCUMULATES = 150; // you usually want to keep this at n_nn x W.

    float MANDATE_SCORE_THRESH = 40.; //If total voting score is greater than this amount in `FLUSH_AFTER_N_ACCUMULATES` then conclude that this is a loopmatch


private:
    mutable std::mutex mutex_hyp_q;
    std::vector< std::vector<int> > hyp_q; //vector[][4] 4 numbers in each
    void digest(); // the processing at FLUSH_AFTER_N_ACCUMULATES.


#if 0
public:
    void set_computed_pose( int i, Matrix4d a_T_b, string info_str );

private:
    bool m_computed_pose = false;
    Matrix4d computed_pose;
    string computed_pose_info;
#endif 
};
