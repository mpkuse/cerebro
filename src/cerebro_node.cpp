/** Main for cerebro node.

        This contains the main for the cerebro. It subscribes to various topics
        from VINS-estimator and setups of threads a) vis-thread, b) redis-association-thread, c) etc

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 26th Oct, 2018
**/


#include <ros/ros.h>
#include <ros/package.h>


#include "PinholeCamera.h"
#include "camodocal/camera_models/Camera.h" // this is in include/camodocal. src files in src/utils/camodocal_src
#include "camodocal/camera_models/CameraFactory.h"
#include <boost/filesystem.hpp>


#include "DataManager.h"
#include "Cerebro.h"
#include "Visualization.h"

#include "utils/nlohmann/json.hpp"
using json = nlohmann::json;


int main( int argc, char ** argv )
{
    //--- ROS INIT ---//
    ros::init( argc, argv, "cerebro_node" );
    ros::NodeHandle nh("~");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);


    //-- Set debug directory --//
    // string debug_output_dir;
    // nh.getParam( "debug_output_dir", debug_output_dir );
    // ROS_WARN( "debug_output_dir : %s", debug_output_dir.c_str() );


    //--- Config File ---//
    string config_file;
    if( !nh.getParam( "config_file", config_file ) )
    {
        ROS_ERROR( "[cerebro_node] config_file cmdline parameter required to read the camera matrix.\nThis is fatal error\n..quit..." );
        exit(1);
    }
    ROS_WARN( "Config File Name : %s", config_file.c_str() );

    // check if file exists - depends on boost
    if ( !boost::filesystem::exists( config_file ) )
    {
      std::cout << "[cerebro_node] Can't find my file: " << config_file << std::endl;
      ROS_ERROR_STREAM( "[cerebro_node] Can't find config_file:" << config_file );
      exit(1);
    }

    camodocal::CameraPtr abstract_camera;
    abstract_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file);
    assert( abstract_camera && "Even after loading yaml the camera seem invalid. Something wrong\n");



    // PinholeCamera camera = PinholeCamera( config_file );
    // camera.printCameraInfo(2); // Not in use, using camodocal camera instead.


    //--- DataManager ---//
    DataManager dataManager = DataManager(nh);
    // dataManager.setCamera(camera);
    dataManager.setAbstractCamera( abstract_camera );



    //--- Subscribers ---// TODO: move all subscribers this to dataManager class ?

    // [A]
    // w_T_c: Pose of camera (all, only a subset are keyframes) in world-cordinate system.
    string camera_pose_topic = string("/vins_estimator/camera_pose");
    ROS_INFO( "Subscribe to camera_pose_topic: %s", camera_pose_topic.c_str() );
    ros::Subscriber sub_odometry = nh.subscribe( camera_pose_topic, 1000, &DataManager::camera_pose_callback, &dataManager );


    // // [A.1] TODO: not needed. consider not subscribing
    // // Marker for keyframes. Caution, these poses are of imu (NOT of camera).
    // string keyframe_pose_topic = string("/vins_estimator/keyframe_pose");
    // ROS_INFO( "Subscribe to keyframe_camera_pose_topic: %s", keyframe_pose_topic.c_str() );
    // ros::Subscriber sub_kf_odometry = nh.subscribe( keyframe_pose_topic, 1000, &DataManager::keyframe_pose_callback, &dataManager );



    // [B]
    // Raw Image (all). The topic's name is in the config_file
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if( !fs.isOpened() )
    {
        ROS_ERROR_STREAM( "[cerebro_node] cannot open config_file: "<< config_file << "\nThe file seems to be stated on the cmdline but it cannot be opened, possibly file is not found on the stated path or it does not have read permission\n...fatal...quit()\n" );
        exit(1);
    }
    string raw_image_topic;
    if( !fs["image_topic"].isString() )
    {
        ROS_ERROR_STREAM( "[cerebro_node] cannot find key 'image_topic' in config_file("<< config_file << ")\n...fatal...quit" );
        exit(1);
    }
    fs["image_topic"] >> raw_image_topic;
    ROS_INFO( "Subscribe to raw_image_topic: %s", raw_image_topic.c_str() );
    ros::Subscriber sub_image = nh.subscribe( raw_image_topic, 100, &DataManager::raw_image_callback, &dataManager );


    // [B.1]
    // Additional Image topic (stereo pair)
    string raw_image_topic_1;
    ros::Subscriber sub_image_1;
    if( !fs["image_topic_1"].isString() )
    {
        ROS_WARN_STREAM( "[cerebro_node] cannot find key `image_topic_1` in config_file=" << config_file << ". This was optional so nothing to worry if you don't need the stereo pair" );
    }
    else {
        fs["image_topic_1"] >> raw_image_topic_1;
        ROS_INFO( "Subscribe to image_topic_1: %s", raw_image_topic_1.c_str() );
        sub_image_1 = nh.subscribe( raw_image_topic_1, 100, &DataManager::raw_image_callback_1, &dataManager );
    }


    // [B.2]
    // Additional Cameras (yaml)
    string camera_yaml_1;
    if( !fs["camera_yaml_1"].isString() )
    {
        ROS_WARN_STREAM( "[cerebro_node] cannot find key `camera_yaml_1` in config_file=" << config_file << ". This was optional so nothing to worry if you don't need the 2nd camera" );
    }
    else {
        fs["camera_yaml_1"] >> camera_yaml_1;
        ROS_INFO(  "camera_yaml_1 : %s", camera_yaml_1.c_str() );

        vector<string> ___path = MiscUtils::split( config_file, '/' );
        // does::> $(python) '/'.join( ___path[0:-1] )
        string ___camera_yaml_1_path = string("");
        for( int _i = 0 ; _i<___path.size()-1 ; _i++ ) {
            ___camera_yaml_1_path += ___path[_i] + "/";
        }
        ___camera_yaml_1_path += "/"+camera_yaml_1;
        cout << "camera_yaml_1_fullpath=" << ___camera_yaml_1_path << endl;


        // check if file exists - depends on boost
        if ( !boost::filesystem::exists( ___camera_yaml_1_path ) )
        {
          std::cout << "[cerebro_node] Can't find my file for additional camera: " << ___camera_yaml_1_path << std::endl;
          ROS_ERROR_STREAM( "[cerebro_node] Can't find my file for additional camera" << ___camera_yaml_1_path );
          exit(1);
        }

        // Make Abstract Camera
        camodocal::CameraPtr abstract_camera_1;
        cout << "Load file : " << ___camera_yaml_1_path << endl;
        abstract_camera_1 = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(___camera_yaml_1_path);
        assert( abstract_camera_1 && "Even after loading yaml the camera_1 seem invalid. Something wrong\n");

        dataManager.setAbstractCamera( abstract_camera_1, 1 );
    }



    // [C]
    // imu_T_cam : imu camera extrinsic calib. Will store this just in case there is a need
    string extrinsic_cam_imu_topic = string("/vins_estimator/extrinsic");
    ROS_INFO( "Subscribe to extrinsic_cam_imu_topic: %s", extrinsic_cam_imu_topic.c_str() );
    ros::Subscriber sub_extrinsic = nh.subscribe( extrinsic_cam_imu_topic, 1000, &DataManager::extrinsic_cam_imu_callback, &dataManager );


    // [D]
    // PointCloud (all): has 5 channels
    string ptcld_topic = string("/vins_estimator/keyframe_point");
    ROS_INFO( "Subscribe to ptcld_topic: %s", ptcld_topic.c_str() );
    ros::Subscriber sub_ptcld = nh.subscribe( ptcld_topic, 1000, &DataManager::ptcld_callback, &dataManager );

    // [E]
    // Tracked features. these contains a) unvn in msg->points b) chnl[0] id of the point c) chnl[1], chnl[2] := (u,v) d) chnl[3], chnl[4] not very sure.
    // string tracked_feat_topic = string("/feature_tracker/feature");
    // ROS_INFO( "Subscribe to tracked_feat_topic: %s", tracked_feat_topic.c_str() );
    // ros::Subscriber sub_tracked_feat = nh.subscribe( tracked_feat_topic, 1000, &DataManager::tracked_feat_callback, &dataManager );



    //--- Start Threads ---//

    // [A]
    // Data associate thread: looks at the callback buffers and sets the data in the std::map
    dataManager.data_association_thread_enable();
    std::thread t1( &DataManager::data_association_thread, &dataManager, 50 );
    // TODO Another thread in class dataManager which will deallocate images in nonkeyframes.


    // [B]
    // Cerebro threads
    Cerebro cer( nh );
    cer.setDataManager( &dataManager );
    cer.run_thread_enable();
    std::thread t2( &Cerebro::run, &cer );

    // [C]
    // Descriptor Computation Thread.
    cer.descriptor_computer_thread_enable();
    std::thread desc_th( &Cerebro::descriptor_computer_thread, &cer ); //runs @20hz

    // [D]
    // Visualization
    Visualization viz(nh);
    viz.setDataManager( &dataManager );
    viz.setCerebro( &cer );
    viz.setVizPublishers( "/cerebro_node/viz/" );
    viz.run_thread_enable();
    std::thread t3( &Visualization::run, &viz, 25 ); //TODO something wrong with the logic in publish. another solution could be we keep #seq in DataNode.


    fs.release();
    ros::spin();

    dataManager.data_association_thread_disable();
    cer.run_thread_disable();
    cer.descriptor_computer_thread_disable();
    viz.run_thread_disable();

    t1.join(); cout << "t1.join()\n";
    t2.join(); cout << "t2.join()\n";
    desc_th.join(); cout << "desc_th.join()\n";
    t3.join(); cout << "t3.join()\n";


    #if 0
    {
        // A demo of how to look inside dataManager.
        std::map< ros::Time, DataNode* > data_map = dataManager.getDataMapRef();
        // auto descriptor_map = cer.getDescriptorMapRef();
        for( auto it = data_map.begin() ; it!= data_map.end() ; it++ )
        {
            cout << TermColor::RED() << "---" << TermColor::RESET() << endl;
            cout << "Map-Key: " << it->first << "\tseq=" << std::distance( data_map.begin(), it ) << "\t" << it->first - dataManager.getPose0Stamp() << endl;
            cout << "Map-Value:\n";
            it->second->prettyPrint(  );

            if( false && it->second->isImageAvailable() ) {
                cv::imshow( "win", it->second->getImage() );
                cv::waitKey(30);
            }
        }

        // Printing Global Variables
        cout << "Pose0 : isAvailable=" << dataManager.isPose0Available() << "\t";
        cout << "stamp=" << dataManager.getPose0Stamp() ;
        cout << endl;

        cout << "Camera:\n" ;
        dataManager.getCameraRef().printCameraInfo(2);

        cout << "IMUCamExtrinsic : isAvailable=" << dataManager.isIMUCamExtrinsicAvailable() << "\t";
        cout << "last updated : " << dataManager.getIMUCamExtrinsicLastUpdated() << "\t" << dataManager.getIMUCamExtrinsicLastUpdated() -dataManager.getPose0Stamp() ;
        cout << "\nimu_T_cam : \n" << PoseManipUtils::prettyprintMatrix4d( dataManager.getIMUCamExtrinsic() ) << endl;
    }
    #endif




    ///////////////////////
    // Actual Logging.  //
    //////////////////////
    #define __LOGGING__ 1 // make this 1 to enable logging. 0 to disable logging. rememeber to catkin_make after this change
    #if __LOGGING__
        // Write json log
        string save_dir = "/Bulk_Data/_tmp/";

        //
        // $ rm -rf ${save_dir}
        // $ mkdir ${save_dir}
        #include <cstdlib>
        string system_cmd;

        system_cmd = string("rm -rf "+save_dir).c_str();
        cout << TermColor::YELLOW() << system_cmd << TermColor::RESET() << endl;
        const int rm_dir_err = system( system_cmd.c_str() );
        if ( rm_dir_err == -1 )
        {
            cout << TermColor::RED() << "[cerebro_node] Error removing directory!\n" << TermColor::RESET() << endl;
            exit(1);
        }

        system_cmd = string("mkdir -p "+save_dir).c_str();
        cout << TermColor::YELLOW() << system_cmd << TermColor::RESET() << endl;
        const int dir_err = system( system_cmd.c_str() );
        if ( dir_err == -1 )
        {
            cout << TermColor::RED() << "[cerebro_node] Error creating directory!\n" << TermColor::RESET() << endl;
            exit(1);
        }
        // done emptying the directory.


        RawFileIO::write_string( save_dir+"/log.json", dataManager.metaDataAsJson() );
        RawFileIO::write_string( save_dir+"/log.txt", dataManager.metaDataAsFlatFile() );


        std::map< ros::Time, DataNode* > data_map = dataManager.getDataMapRef();
        for( auto it = data_map.begin() ; it!= data_map.end() ; it++ )
        {
            int seq_id = std::distance( data_map.begin() , it );

            string fname = save_dir+"/"+to_string(seq_id );
            if( it->second->isImageAvailable() )
                RawFileIO::write_image( fname+".jpg", it->second->getImage()  );

            // Save VINS pose
            if( it->second->isPoseAvailable() ) {
                RawFileIO::write_EigenMatrix( fname+".wTc", it->second->getPose() );
            }

            // Save Point Cloud
            if( it->second->isPtCldAvailable() ) {
                RawFileIO::write_EigenMatrix( fname+".wX.pointcloud", it->second->getPointCloud() );
                if( it->second->isPoseAvailable() ) {
                    RawFileIO::write_EigenMatrix( fname+".cX.pointcloud", it->second->getPose().inverse() *  it->second->getPointCloud() );
                }
                else
                    ROS_WARN( "ptclod is available but pose is not available at seq_id=%d", seq_id );
            }


            // Save Tracked Points
            if( it->second->isUVAvailable() ) {
                RawFileIO::write_EigenMatrix( fname+".unvn", it->second->getUnVn() );
                RawFileIO::write_EigenMatrix( fname+".uv", it->second->getUV() );
                RawFileIO::write_EigenMatrix( fname+".id", it->second->getFeatIds() );
            }

            // Save Whole Image Descriptor
            if( it->second->isWholeImageDescriptorAvailable() ) {
                RawFileIO::write_EigenMatrix( fname+".imgdesc", it->second->getWholeImageDescriptor() );
            }
        }


        // Save Camera Matrix and IMUCamExtrinsic
        vector<short> cam_keys = dataManager.getAbstractCameraKeys();
        for( short _icam=0 ; _icam < cam_keys.size() ; _icam++ )
        {
            short key=cam_keys[_icam];
            if( dataManager.isAbstractCameraSet(key) ) {
                auto abstract_camera = dataManager.getAbstractCameraRef(key);
                RawFileIO::write_string( save_dir+"/cameraIntrinsic."+std::to_string(key)+".info", abstract_camera->parametersToString() );
                abstract_camera->writeParametersToYamlFile( save_dir+"/cameraIntrinsic."+std::to_string(key)+".yaml" );
            }
        }


        // Save Camera-IMU Extrinsics
        cout << "IMUCamExtrinsic : isAvailable=" << dataManager.isIMUCamExtrinsicAvailable() << "\t";
        cout << "last updated : " << dataManager.getIMUCamExtrinsicLastUpdated() << "\t" << dataManager.getIMUCamExtrinsicLastUpdated() -dataManager.getPose0Stamp() ;
        cout << "\nimu_T_cam : \n" << PoseManipUtils::prettyprintMatrix4d( dataManager.getIMUCamExtrinsic() ) << endl;
        if( dataManager.isIMUCamExtrinsicAvailable() ) {
            RawFileIO::write_EigenMatrix( save_dir+"/IMUCamExtrinsic.imu_T_cam", dataManager.getIMUCamExtrinsic() );

            std::stringstream buffer;
            buffer << "IMUCamExtrinsicLastUpdated: "<< dataManager.getIMUCamExtrinsicLastUpdated() << endl;
            buffer << "IMUCamExtrinsicLastUpdated Rel: "<< dataManager.getIMUCamExtrinsicLastUpdated() -dataManager.getPose0Stamp()  << endl;
            RawFileIO::write_string( save_dir+"/IMUCamExtrinsic.info", buffer.str() );
        }
        else {
            ROS_ERROR( "[cerebro_node.cpp] IMUCamExtrinsic appear to be not set...something seem wrong\n" );
        }


        // Save foundLoops from Cerebro class
        json jsonout_obj = cer.foundLoops_as_JSON();
        RawFileIO::write_string( save_dir+"/loopcandidates_liverun.json", jsonout_obj.dump(4) );


    #endif




    return 0 ;

}
