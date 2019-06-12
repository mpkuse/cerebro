#pragma once

// Getting lot of deallocation issues when storing images inside of DataNode.
// This class now stores the image data and useless images are stored to
// file to conserve ram.
//
//      Author  : Manohar Kuse <mpkuse@connect.ust.hk>
//      Created : 12th June, 2019
//

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility> //std::pair
#include <fstream>

// threading
#include <thread>
#include <mutex>
#include <atomic>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>

#include "utils/TermColor.h"
#include "utils/ElapsedTime.h"
#include "utils/RawFileIO.h"

using namespace std;

enum MEMSTAT { AVAILABLE_ON_RAM, AVAILABLE_ON_DISK, UNAVAILABLE };

class ImageDataManager
{
public:
    ImageDataManager();
    ~ImageDataManager();
    bool setImage( const string ns, const ros::Time t, const cv::Mat img );
    bool setNewImageFromMsg( const string ns, const sensor_msgs::ImageConstPtr msg );
    bool getImage( const string ns, const ros::Time t, cv::Mat& outImg ) const;

    bool rmImage( const string ns, const ros::Time t );
    bool stashImage( const string ns, const ros::Time t );

    bool isImageRetrivable( const string ns, const ros::Time t );

    bool print_status( string fname );

private:
    mutable std::mutex m;
    const string STASH_DIR;

    // key: (namespace, t)
    std::map< std::pair<string , ros::Time>, MEMSTAT > status; //status at each timestamp
    std::map< std::pair<string , ros::Time>, cv::Mat > image_data;
};
