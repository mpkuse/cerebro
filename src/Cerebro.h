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


#include "PinholeCamera.h"
#include "DataManager.h"

class Cerebro
{
public:
    Cerebro();

    void setDataManager( DataManager* dataManager );

    // This is supposed to be run in a separate thread.
    void run_thread_enable() { b_run_thread = true; }
    void run_thread_disable() { b_run_thread = false; }
    void run();

    // This monitors the dataManager->data_map and makes sure the descriptor are uptodate.
    // descriptors are computed by an external ros-service.
    void descriptor_computer_thread_enable() { b_descriptor_computer_thread = true; }
    void descriptor_computer_thread_disable() { b_descriptor_computer_thread = false; }
    void descriptor_computer_thread();

private:
    bool m_dataManager_available=false;
    DataManager * dataManager;

    atomic<bool> b_run_thread;

    std::map< ros::Time, double > descriptors;
    atomic<bool> b_descriptor_computer_thread;

};
