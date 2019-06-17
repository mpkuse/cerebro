#pragma once

/** Cerebro Class
        This class is suppose to be the main-brain of this package.
        It has to run its own threads (should not block)

        It can access DataManager::camera, DataManager::imu_T_cam, DataManager::data_map.

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 29th Oct, 2018
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <vector>

#include "PinholeCamera.h"
#include "DataManager.h"
#include "ProcessedLoopCandidate.h"
#include "DlsPnpWithRansac.h"
#include "HypothesisManager.h"

#include "utils/TermColor.h"
#include "utils/ElapsedTime.h"
#include "utils/Plot2Mat.h"

#include "utils/CameraGeometry.h"
#include "utils/PointFeatureMatching.h"


#include "utils/nlohmann/json.hpp"
using json = nlohmann::json;

// ROS-Service Defination
#include <cerebro/WholeImageDescriptorCompute.h>

// #include <theia/theia.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;


//comment this out to remove dependence on faiss.
// If using faiss, also remember to link to libfaiss.so. See my CMakeList file to know how to do it.
// #define HAVE_FAISS

#ifdef HAVE_FAISS
// faiss is only used for generating loopcandidates.
// only function that uses faiss is `Cerebro::faiss__naive_loopcandidate_generator()`
// Although faiss is strictly not needed, it is a much // faster way for nearest neighbour search. If you don't want it
// just use the naive implementation: descrip_N__dot__descrip_0_N().
//    Note: Cerebro::descrip_N__dot__descrip_0_N() and Cerebro::faiss__naive_loopcandidate_generator()
//    are exactly same in functionality.
#include <faiss/IndexFlat.h>
#endif //HAVE_FAISS

class Cerebro
{

    //-------------- Constructor --------------------//
public:
    Cerebro( ros::NodeHandle& nh  ); // TODO removal of nh argument.
    void setDataManager( DataManager* dataManager );
    void setPublishers( const string base_topic_name );

private:
    // global private variables
    bool m_dataManager_available=false;
    DataManager * dataManager;
    ros::NodeHandle nh; ///< This is here so that I can set new useful publishers

    bool m_pub_available = false;
    ros::Publisher pub_loopedge;

    //-------------- END Constructor --------------------//




    //--------------- Descriptor Computation Thread ------------------//
public:
    // This monitors the dataManager->data_map and makes sure the descriptor are uptodate.
    // descriptors are computed by an external ros-service. in the future can have
    // more similar threads to compute object bounding boxes, text and other perception related services.
    void descriptor_computer_thread_enable() { b_descriptor_computer_thread = true; }
    void descriptor_computer_thread_disable() { b_descriptor_computer_thread = false; }
    void descriptor_computer_thread();

private:
    atomic<bool> b_descriptor_computer_thread;
    atomic<bool> connected_to_descriptor_server;
    atomic<bool> descriptor_size_available;
    atomic<int> descriptor_size;


    // Storage for Intelligence
    mutable std::mutex m_wholeImageComputedList;
    vector<ros::Time> wholeImageComputedList; ///< A list of stamps where descriptors are computed and available.
public:
    const int wholeImageComputedList_size() const; //size of the list. threadsafe
    const ros::Time wholeImageComputedList_at(int k) const; //< returns kth element of the list. threadsafe
    //--------------- END Descriptor Computation Thread ------------------//




    //---------------- Populate Loop Candidates --------------------//
public:
    // This is supposed to be run in a separate thread.
    void run_thread_enable() { b_run_thread = true; }
    void run_thread_disable() { b_run_thread = false; }
    void run(); //< The loopcandidate (geometrically unverified) producer.


    void descrip_N__dot__descrip_0_N(); //< Naive method of dot product DIY
    #ifdef HAVE_FAISS
    void faiss__naive_loopcandidate_generator(); //< similar to descrip_N__dot__descrip_0_N() but uses facebook's faiss

    void faiss_clique_loopcandidate_generator();


    void faiss_multihypothesis_tracking();
    #endif //HAVE_FAISS



private:
    // private things to run thread
    atomic<bool> b_run_thread;
    bool wait_until__connectedToDescServer_and_descSizeAvailable( int timeout_in_sec );  //blocking call


public:
    // These are loop candidates. These calls are thread-safe
    //  producer: `run()`
    //  user: `Visualization::publish_loopcandidates`
    const int foundLoops_count() const ;
    const std::tuple<ros::Time, ros::Time, double> foundLoops_i( int i) const;
    json foundLoops_as_JSON();

private:
    mutable std::mutex m_foundLoops;
    vector< std::tuple<ros::Time, ros::Time, double> > foundLoops; // a list containing loop pairs. this is populated by `run()`


    //---------------- END Populate Loop Candidates --------------------//



    //------------------ Geometry Thread ---------------------------//
    // calls this->foundLoops_count() and this->foundLoops_i() and uses dataManager
    // to geometric verify and to compute the poses of loop-pairs.
public:
    void loopcandidate_consumer_enable() { b_loopcandidate_consumer=true; }
    void loopcandidate_consumer_disable() { b_loopcandidate_consumer=false; }
    void loopcandiate_consumer_thread();

    const int processedLoops_count() const;
    const ProcessedLoopCandidate& processedLoops_i( int i ) const;
    // TODO: have a function which returns a json of the info in processedloopcandi_list.





private:
    atomic<bool> b_loopcandidate_consumer;

    // helpers

    // Processed foundLoops_i[ j ] and writes the info in the object `proc_candi`
    // bool process_loop_candidate_imagepair( int j, ProcessedLoopCandidate& proc_candi );

    // This function processes the jth loopcandidate and fills in the ProcessedLoopCandidate.
    // The return status means that some poses were computed. It doesn't mean the poses were consistent.
    // Infact, nothing about consistency is performed here. It just computes relative poses using 3 indipendent way.
    bool process_loop_candidate_imagepair_consistent_pose_compute( int j, ProcessedLoopCandidate& proc_candi ); //< enhanced version of the above

    bool init_stereogeom(); // expected to be called in loopcandiate_consumer_thread. this sets the variable `stereogeom`
    bool retrive_stereo_pair( DataNode* node, cv::Mat& left_image, cv::Mat& right_image, bool bgr2gray=true );
    std::shared_ptr<StereoGeometry> stereogeom;


    mutable std::mutex m_processedLoops;
    vector< ProcessedLoopCandidate > processedloopcandi_list;



    //------------------ END Geometry Thread ---------------------------//


    //--------------- kidnaped identification thread ------------------//
public:
    void kidnaped_thread( int loop_rate_hz=5);
    void kidnaped_thread_enable() {b_kidnaped_thread_enable=true;};
    void kidnaped_thread_disable() {b_kidnaped_thread_enable=false;};

    bool is_kidnapped();  // gives the current (kidnap) status. threadsafe
    bool kidnap_info( int i, ros::Time& start_, ros::Time& end_ ); //< returns true if i was a valid kidnap index. return false if it was an invalid index, ie. such kidnap didnt exist
    json kidnap_info_as_json();
    int n_kidnaps(); //< will with return length of `start_of_kidnap` current state has to be inferred by call to `is_kidnapped()`

    // kidnap callbacks
    void kidnap_bool_callback( const std_msgs::BoolConstPtr& rcvd_header ) ;
    void kidnap_header_callback( const std_msgs::HeaderConstPtr& rcvd_header ) ;

private:
    atomic<bool> b_kidnaped_thread_enable;

    std::mutex mutex_kidnap;
    atomic< bool > state_is_kidnapped;
    vector< ros::Time > start_of_kidnap;
    vector< ros::Time > end_of_kidnap;

    ros::Publisher rcvd_flag_pub;
    ros::Publisher kidnap_indicator_header_pub;
    bool is_kidnapn_indicator_set = false;
};
