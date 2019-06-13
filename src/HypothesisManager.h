#pragma once
// This class will manage all the loop hypothesis.
// This is for implementing the heuristics for multi hypothesis tracking.
// A new graph-node is created when I have a new putative matching pair.
// This matching pair is associated with an existing hypothesis or a new
// hypothesis is created. Allthe hypothesis will have a time-to-live.

#include <iostream>
#include <string>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <tuple>

#include "utils/TermColor.h"
using namespace std;

class Hypothesis
{
public:
    Hypothesis(int a, int b, float prod )
    {
        list_of_nodes_in_this_hypothesis.push_back( std::make_tuple(a,b, prod) );
    }
    std::vector< std::tuple<int,int, float> >  list_of_nodes_in_this_hypothesis;


    void print_active_hypothesis(  ) {

        cout << "len(list) = " << list_of_nodes_in_this_hypothesis.size() << "\t";
        vector<int> tmp_a, tmp_b;
        for( auto it=  list_of_nodes_in_this_hypothesis.begin() ;
                  it!= list_of_nodes_in_this_hypothesis.end() ;
                  it++ )
        {
            tmp_a.push_back( std::get<0>(*it) );
            tmp_b.push_back( std::get<1>(*it) );
        }

        // cout <<
        const auto result_a = std::minmax_element( tmp_a.begin(), tmp_a.end() );
        const auto result_b = std::minmax_element( tmp_b.begin(), tmp_b.end() );

        cout << *result_a.first << "---->" << *result_a.second  << "  ";
        cout << *result_b.first << "---->" << *result_b.second  << "  ";
        cout << endl;

        #if 0 //full details
        for( auto it=  list_of_nodes_in_this_hypothesis.begin() ;
                  it!= list_of_nodes_in_this_hypothesis.end() ;
                  it++ )
        {
            cout << " (" << std::get<0>(*it) << "," << std::get<1>(*it) << ") ";
        }
        cout << endl;
        #endif
    }


};

class HypothesisManager
{
public:
    HypothesisManager();
    ~HypothesisManager();

    // We add a node in the graph for every putative loop candidate.
    // a,b:  Indicates a loop connection between node[a] and node[b] with a dot_product=dot_prod
    bool add_node( int a, int b, float dot_prod );


    //---
    // Info function
    //---
    // int n_active_hypothesis();



private:
    std::vector<Hypothesis> active_hyp;

};
