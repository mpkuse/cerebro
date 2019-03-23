#pragma once

/**     Visualization class
        This holds a reference to the DataManager. Will publish messages
        to be seen with rviz.

        Author  : Manohar Kuse<mpkuse@connect.ust.hk>
        Created : 29th Oct, 2018

        based on
        https://github.com/mpkuse/nap/blob/master-desktop/src/DataManager_rviz_visualization.cpp

*/



#include <ros/ros.h>
#include <ros/package.h>

// ros msg
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

#include "utils/RosMarkerUtils.h"
#include "utils/TermColor.h"


#include "PinholeCamera.h"
#include "DataManager.h"
#include "Cerebro.h"


class Visualization
{
public:
    Visualization( ros::NodeHandle &nh );

    void setDataManager( DataManager* dataManager );
    void setCerebro( Cerebro* cerebro );

    void setVizPublishers( const string base_topic_name );

    // This is supposed to be run in a separate thread.
    void run_thread_enable() { b_run_thread = true; }
    void run_thread_disable() { b_run_thread = false; }
    void run( const int looprate  );

private:
    ros::NodeHandle nh;

    bool m_dataManager_available=false;
    DataManager * dataManager;

    bool m_cerebro_available=false;
    Cerebro * cerebro;

    atomic<bool> b_run_thread;


private:
    void publish_frames(); //< publishes last 10 frames as markers. (occasionally all)
    void publish_loopcandidates(); //< publishes last 10 loop candidates. cerebro->foundLoops_count().
    void publish_processed_loopcandidates(); //< uses Cerebro::processedLoops_i() and publishes only the newly found loops. cerebro->processedLoops_i( i )
    void publish_test_string();


    //publishers
private:
    ros::Publisher chatter_pub;
    ros::Publisher framedata_pub;
    ros::Publisher imagepaire_pub;

};
