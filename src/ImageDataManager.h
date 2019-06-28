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


#include "utils/nlohmann/json.hpp"
using json = nlohmann::json;

using namespace std;

enum MEMSTAT { AVAILABLE_ON_RAM, AVAILABLE_ON_DISK, UNAVAILABLE, AVAILABLE_ON_RAM_DUETO_HIT };

class ImageDataManager
{
public:
    ImageDataManager();
    bool initStashDir( bool clear_dir, const string __dir=string("/tmp/cerebro_stash/")  ); //< clear_dir: setting this to true will cause `rm -rf DIR && mkdir DIR` ;;;__dir: The directory usually set it to '/tmp/cerebro_stash' .

    ~ImageDataManager();
    bool setImage( const string ns, const ros::Time t, const cv::Mat img );
    bool setNewImageFromMsg( const string ns, const sensor_msgs::ImageConstPtr msg );
    bool getImage( const string ns, const ros::Time t, cv::Mat& outImg );

    bool rmImage( const string ns, const ros::Time t );
    bool stashImage( const string ns, const ros::Time t );

    bool isImageRetrivable( const string ns, const ros::Time t ) const;

    bool print_status( string fname ) const;
    bool print_status(  ) const;

    // - go over all status and stash all the images that remain on RAM.
    // Also retuns the map status as json object
    json stashAll();
    bool loadStateFromDisk( const json json_obj );


private:
    mutable std::mutex m;
    string STASH_DIR = string(""); bool m_STASH_DIR = false;
    const string key_to_imagename( const string ns, const ros::Time t ) const
    {
        return STASH_DIR+"/"+ns+"__"+to_string(t.toNSec())+".jpg";
    }

    const string memstat_to_str( MEMSTAT m ) const
    {
        if( m == MEMSTAT::AVAILABLE_ON_RAM )
            return "AVAILABLE_ON_RAM";

        if( m == MEMSTAT::AVAILABLE_ON_DISK )
            return "AVAILABLE_ON_DISK";

        if( m == MEMSTAT::UNAVAILABLE )
            return "UNAVAILABLE";

        if( m == MEMSTAT::AVAILABLE_ON_RAM_DUETO_HIT )
            return "AVAILABLE_ON_RAM_DUETO_HIT";

        return "N.A.";
    }

    // key: (namespace, t)
    std::map< std::pair<string , ros::Time>, MEMSTAT > status; //status at each timestamp (entris not be esared)
    std::map< std::pair<string , ros::Time>, cv::Mat > image_data;


    // when getImage finds that AVAILABLE_ON_DISK, it loads the image (from disk) in the map image_data.
    // Also it stores say 10 in this map. This 10 means that the image be deleted again if 10 consecutive
    // getImage requests dont request this image.
    // CACHE-Algorithm: 5-minute-rule.
    std::map<  std::pair<string , ros::Time>, int > hit_count;
    int decrement_hit_counts_and_deallocate_expired(); //returns how many expired


    bool rm_stash_dir_in_destructor = true;

public:
    const string getStashDir() const { return STASH_DIR; }
    void set_rm_stash_dir_in_destructor_as_false() {rm_stash_dir_in_destructor=false; }
    void set_rm_stash_dir_in_destructor_as_true()  {rm_stash_dir_in_destructor=true; }

    void ensure_init() const {
        if( !m_STASH_DIR) {
            ROS_ERROR( "m_STASH_DIR is false. This means you have not initialized the ImageDataManager. You need to call the function initStashDir() befre you can start using it\n");
            exit(1);
        }
    }
};
