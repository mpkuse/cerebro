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
#include <iostream>
#include <vector>

#include "PinholeCamera.h"
#include "DataManager.h"
#include "utils/TermColor.h"
#include "utils/ElapsedTime.h"

#include "utils/CameraGeometry.h"
#include "utils/PointFeatureMatching.h"

#include "utils/nlohmann/json.hpp"
using json = nlohmann::json;

// ROS-Service Defination
#include <cerebro/WholeImageDescriptorCompute.h>


class Cerebro
{

    //-------------- Constructor --------------------//
public:
    Cerebro( ros::NodeHandle& nh  ); // TODO removal of nh argument.
    void setDataManager( DataManager* dataManager );

private:
    // global private variables
    bool m_dataManager_available=false;
    DataManager * dataManager;
    ros::NodeHandle nh; ///< I see no reason to have this at all. TODO: removal.

    //-------------- END Constructor --------------------//



    //---------------- Populate Loop Candidates --------------------//
public:
    // This is supposed to be run in a separate thread.
    void run_thread_enable() { b_run_thread = true; }
    void run_thread_disable() { b_run_thread = false; }
    void run();

    void descrip_N__dot__descrip_0_N();


private:
    // private things to run thread
    atomic<bool> b_run_thread;


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





private:
    atomic<bool> b_loopcandidate_consumer;

    // helpers
    void process_loop_candidate_imagepair( int j );

    bool init_stereogeom(); // expected to be called in loopcandiate_consumer_thread. this sets the variable `stereogeom`
    bool retrive_stereo_pair( DataNode* node, cv::Mat& left_image, cv::Mat& right_image, bool bgr2gray=true );
    std::shared_ptr<StereoGeometry> stereogeom;



    //------------------ END Geometry Thread ---------------------------//


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
    std::mutex m_wholeImageComputedList;
    vector<ros::Time> wholeImageComputedList; ///< A list of stamps where descriptors are computed and available.
    //--------------- END Descriptor Computation Thread ------------------//
};
