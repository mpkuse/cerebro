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

// ROS-Service Defination
#include <cerebro/WholeImageDescriptorCompute.h>


class Cerebro
{
public:
    Cerebro( ros::NodeHandle& nh  );

    void setDataManager( DataManager* dataManager );

    // std::map< ros::Time, VectorXd >& getDescriptorMapRef() { return descriptors_vec; }

    // This is supposed to be run in a separate thread.
    void run_thread_enable() { b_run_thread = true; }
    void run_thread_disable() { b_run_thread = false; }
    void run();

    // This monitors the dataManager->data_map and makes sure the descriptor are uptodate.
    // descriptors are computed by an external ros-service. in the future can have
    // more similar threads to compute object bounding boxes and other perception related services.
    void descriptor_computer_thread_enable() { b_descriptor_computer_thread = true; }
    void descriptor_computer_thread_disable() { b_descriptor_computer_thread = false; }
    void descriptor_computer_thread();

private:
    bool m_dataManager_available=false;
    DataManager * dataManager;
    ros::NodeHandle nh;

    atomic<bool> b_run_thread;

    // std::map< ros::Time, double > descriptors;
    // std::map< ros::Time, VectorXd > descriptors_vec; //TODO: Removal. Should put/get the descriptor to the DataNode.
    atomic<bool> b_descriptor_computer_thread;


private:
    std::mutex m_wholeImageComputedList; 
    vector<ros::Time> wholeImageComputedList; ///< A list of stamps where descriptors are computed and available.
};
