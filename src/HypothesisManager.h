#pragma once
// This class will manage all the loop hypothesis.
// This is for implementing the heuristics for multi hypothesis tracking.
// A new graph-node is created when I have a new putative matching pair.
// This matching pair is associated with an existing hypothesis or a new
// hypothesis is created. Allthe hypothesis will have a time-to-live.

#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <tuple>

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include "utils/TermColor.h"
using namespace std;

class Hypothesis
{
public:
    Hypothesis(int a, int b, float prod )
    {
        list_of_nodes_in_this_hypothesis.push_back( std::make_tuple(a,b, prod) );
        time_to_live = 20;
    }
    std::vector< std::tuple<int,int, float> >  list_of_nodes_in_this_hypothesis;

    string info_string()
    {
        std::stringstream ss;
        ss << "ttl=" << time_to_live << " ";
        ss << "\tlen(list) = " << list_of_nodes_in_this_hypothesis.size() << "\t";

        //starts and ends of the hyp
        vector<int> tmp_a, tmp_b;
        for( auto it=  list_of_nodes_in_this_hypothesis.begin() ;
                  it!= list_of_nodes_in_this_hypothesis.end() ;
                  it++ )
        {
            tmp_a.push_back( std::get<0>(*it) );
            tmp_b.push_back( std::get<1>(*it) );
        }

        const auto result_a = std::minmax_element( tmp_a.begin(), tmp_a.end() );
        const auto result_b = std::minmax_element( tmp_b.begin(), tmp_b.end() );

        ss << *result_a.first << "---->" << *result_a.second  << "  ";
        ss << *result_b.first << "---->" << *result_b.second  << "  ";

        return ss.str();
    }

    #if 0
    void print_active_hypothesis( const string header_str ) {

        ofstream myfile;
        myfile.open ("/dev/pts/1");

        myfile << header_str << "\t\t";
        myfile << "ttl=" << time_to_live << " ";
        myfile << "\tlen(list) = " << list_of_nodes_in_this_hypothesis.size() << "\t";
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

        myfile << *result_a.first << "---->" << *result_a.second  << "  ";
        myfile << *result_b.first << "---->" << *result_b.second  << "  ";
        myfile << endl;

        #if 0 //full details
        for( auto it=  list_of_nodes_in_this_hypothesis.begin() ;
                  it!= list_of_nodes_in_this_hypothesis.end() ;
                  it++ )
        {
            myfile << " (" << std::get<0>(*it) << "," << std::get<1>(*it) << ") ";
        }
        myfile << endl;
        #endif

        myfile.close();
    }
    #endif


public:
    void decrement_ttl()
    {
        if( time_to_live <= 0 )
            return;
        time_to_live--;
    }

    void increment_ttl()
    {
        // linear increment
        time_to_live++;


        if( time_to_live > 100 )
            time_to_live++;

        // multiplicative increment
        // time_to_live += int(.1*time_to_live);

    }
    int get_ttl() { return time_to_live; }

    bool is_hypothesis_active() { if(time_to_live<=0) return false; else return true; }
    int n_elements_in_list() { return list_of_nodes_in_this_hypothesis.size(); }

private:
    int time_to_live;

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
    // Info functions
    //---
    // int n_active_hypothesis();


    //---
    // Loop through each active hypothesis and decrement the time-to-live
    // This is intented to be called every few milisecs.
    void digest();



private:
    mutable mutex mx;
    std::vector<Hypothesis> active_hyp;


//----------------------------
//--- Monitoring Thread
//----------------------------
public:
    void monitoring_thread();
    void monitoring_thread_enable() { b_monitoring_thread = true;}
    void monitoring_thread_disable() {b_monitoring_thread = false;}

private:
    atomic<bool> b_monitoring_thread;

};
