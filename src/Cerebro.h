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

#include "utils/nlohmann/json.hpp"
using json = nlohmann::json;

// ROS-Service Defination
#include <cerebro/WholeImageDescriptorCompute.h>


class Cerebro
{
public:
    Cerebro( ros::NodeHandle& nh  ); // TODO removal of nh argument.
    void setDataManager( DataManager* dataManager );

private:
    // global private variables
    bool m_dataManager_available=false;
    DataManager * dataManager;
    ros::NodeHandle nh; ///< I see no reason to have this at all. TODO: removal.

public:
    // This is supposed to be run in a separate thread.
    void run_thread_enable() { b_run_thread = true; }
    void run_thread_disable() { b_run_thread = false; }
    void run();

public: 
    const int foundLoops_count() const ;
    const std::tuple<ros::Time, ros::Time, double> foundLoops_i( int i) const;
    json foundLoops_as_JSON();


private:
    // private things to run thread
    atomic<bool> b_run_thread;



public:
    // This monitors the dataManager->data_map and makes sure the descriptor are uptodate.
    // descriptors are computed by an external ros-service. in the future can have
    // more similar threads to compute object bounding boxes, text and other perception related services.
    void descriptor_computer_thread_enable() { b_descriptor_computer_thread = true; }
    void descriptor_computer_thread_disable() { b_descriptor_computer_thread = false; }
    void descriptor_computer_thread();

private:
    atomic<bool> b_descriptor_computer_thread;


// Storage for Intelligence
private:
    std::mutex m_wholeImageComputedList;
    vector<ros::Time> wholeImageComputedList; ///< A list of stamps where descriptors are computed and available.

    mutable std::mutex m_foundLoops;
    vector< std::tuple<ros::Time, ros::Time, double> > foundLoops; // a list containing loop pairs
};
