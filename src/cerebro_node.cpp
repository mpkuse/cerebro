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

    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if( !fs.isOpened() )
    {
        ROS_ERROR_STREAM( "[cerebro_node] cannot open config_file: "<< config_file << "\nThe file seems to be stated on the cmdline but it cannot be opened, possibly file is not found on the stated path or it does not have read permission\n...fatal...quit()\n" );
        exit(1);
    }



    // --- Get yaml config for primary camera. (cam0) ---//
    camodocal::CameraPtr abstract_camera;
    // abstract_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file);
    // assert( abstract_camera && "Even after loading yaml the camera seem invalid. Something wrong\n");
    {
        string cam0_calib;
        if( !fs["cam0_calib"].isString() )
        {
            ROS_WARN_STREAM( "[cerebro_node] cannot find key `cam0_calib` in config_file=" << config_file  );
        }
        else {
            fs["cam0_calib"] >> cam0_calib;
            ROS_INFO(  "cam0_calib : %s", cam0_calib.c_str() );

            vector<string> ___path = MiscUtils::split( config_file, '/' );
            // does::> $(python) '/'.join( ___path[0:-1] )
            string ___cam0_path = string("");
            for( int _i = 0 ; _i<___path.size()-1 ; _i++ ) {
                ___cam0_path += ___path[_i] + "/";
            }
            ___cam0_path += "/"+cam0_calib;
            cout << "cam0_fullpath=" << ___cam0_path << endl;


            // check if file exists - depends on boost
            if ( !boost::filesystem::exists( ___cam0_path ) )
            {
              std::cout << "[cerebro_node] Can't find my file for primary camera: " << ___cam0_path << std::endl;
              ROS_ERROR_STREAM( "[cerebro_node] Can't find my file for primary camera" << ___cam0_path );
              exit(1);
            }

            // Make Abstract Camera
            // camodocal::CameraPtr abstract_camera_1;
            cout << TermColor::GREEN() << "Load file : " << ___cam0_path << TermColor::RESET() << endl;
            abstract_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(___cam0_path);
            assert( abstract_camera && "Even after loading yaml the camera seem invalid. Something wrong\n");


            // dataManager.setAbstractCamera( abstract_camera_1, 1 );
        }
    }




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

    string raw_image_topic;
    if( !fs["image0_topic"].isString() )
    {
        ROS_ERROR_STREAM( "[cerebro_node] cannot find key 'image0_topic' in config_file("<< config_file << ")\n...fatal...quit" );
        exit(1);
    }
    fs["image0_topic"] >> raw_image_topic;
    ROS_INFO( "Subscribe to raw_image_topic: %s", raw_image_topic.c_str() );
    ros::Subscriber sub_image = nh.subscribe( raw_image_topic, 10, &DataManager::raw_image_callback, &dataManager );

    // [B.1]
    // Additional Image topic (stereo pair)
    string raw_image_topic_1;
    ros::Subscriber sub_image_1;
    if( !fs["image1_topic"].isString() )
    {
        ROS_WARN_STREAM( "[cerebro_node] cannot find key `image1_topic` in config_file=" << config_file << ". This was optional so nothing to worry if you don't need the stereo pair" );
        exit(1);
    }
    else {
        fs["image1_topic"] >> raw_image_topic_1;
        ROS_INFO( "Subscribe to image_topic_1: %s", raw_image_topic_1.c_str() );
        sub_image_1 = nh.subscribe( raw_image_topic_1, 10, &DataManager::raw_image_callback_1, &dataManager );
    }



    // [B.2]
    // Additional Cameras (yaml)
    string camera_yaml_1;
    if( !fs["cam1_calib"].isString() )
    {
        ROS_WARN_STREAM( "[cerebro_node] cannot find key `cam1_calib` in config_file=" << config_file << ". This was optional so nothing to worry if you don't need the 2nd camera" );
        exit(1);
    }
    else {
        fs["cam1_calib"] >> camera_yaml_1;
        ROS_INFO(  "cam1_calib : %s", camera_yaml_1.c_str() );

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
        cout << TermColor::GREEN() << "Load file : " << ___camera_yaml_1_path << TermColor::RESET() << endl;
        abstract_camera_1 = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(___camera_yaml_1_path);
        assert( abstract_camera_1 && "Even after loading yaml the camera_1 seem invalid. Something wrong\n");

        dataManager.setAbstractCamera( abstract_camera_1, 1 );
    }


    // [B.3]
    // Camera baseline. Set stereobaseline transform ie. right_T_left from yaml file
    if(  !fs["extrinsic_1_T_0"].isString() )
    {
        ROS_WARN_STREAM( "[cerebro_node] cannot find key `extrinsic_1_T_0` in config file=" << config_file << " this was not needed for monocular camera, but needed for a stereo camera" );
        cout << TermColor::RED() <<  "[cerebro_node] cannot find key `extrinsic_1_T_0` in config file=" << config_file << " this was not needed for monocular camera, but needed for a stereo camera" << TermColor::RESET() << endl;
        // exit(1);


        #if 1
        // look for the existence of `body_T_cam0` and `body_T_cam1`
        cout << TermColor::YELLOW() << "Since, the key `extrinsic_1_T_0`, I am looking for the existence of `body_T_cam0` and `body_T_cam1`" << TermColor::RESET() << endl;

        if( fs["body_T_cam0"].empty() || fs["body_T_cam1"].empty() ) {
            cout << TermColor::RED() << "Either of the keys `body_T_cam0` or `body_T_cam1` cannot be found in config file. FATAL ERROR\n" << TermColor::RESET() ;
            exit(1);
        }


        cout << TermColor::iGREEN() <<  "The keys `body_T_cam0` and `body_T_cam1` exists, compute extrinsic_1_T_0 from these two as ` ee_body_T_cam1.inverse() * ee_body_T_cam0;`\n" << TermColor::RESET() ;
        cv::Mat body_T_cam0, body_T_cam1;
        fs["body_T_cam0"] >> body_T_cam0;
        fs["body_T_cam1"] >> body_T_cam1;
        Matrix4d ee_body_T_cam0, ee_body_T_cam1;
        cv::cv2eigen( body_T_cam0,  ee_body_T_cam0 );
        cv::cv2eigen( body_T_cam1,  ee_body_T_cam1 );

        Matrix4d cam1_T_cam0 = ee_body_T_cam1.inverse() * ee_body_T_cam0;
        cout << "Matrix4d cam1_T_cam0 = " << PoseManipUtils::prettyprintMatrix4d(cam1_T_cam0) << endl;
        {
        cout << "raw matrix of cam1_T_cam0:\n" << cam1_T_cam0 << "----" <<  endl;

        double q_xyzw[5], t_xyz[5];
        PoseManipUtils::eigenmat_to_raw_xyzw( cam1_T_cam0, q_xyzw, t_xyz );
        cout << "q_xyzw: " << q_xyzw[0] << "," << q_xyzw[1] << "," << q_xyzw[2] << "," << q_xyzw[3] << endl;
        cout << " t_xyz: " << t_xyz[0] << "," << t_xyz[1] << "," << t_xyz[2]  << endl;
        }
        dataManager.setCameraRelPose( cam1_T_cam0, std::make_pair(1,0) );

        #endif

    }
    else {
        // Extract the fname from the config_file
        string extrinsic_1_T_0;
        fs["extrinsic_1_T_0"] >> extrinsic_1_T_0;
        ROS_INFO(  "in config_file=%s; extrinsic_1_T_0 : %s", config_file.c_str(), extrinsic_1_T_0.c_str() );

        // Make full path from fname
        vector<string> ___path = MiscUtils::split( config_file, '/' );
        // does::> $(python) '/'.join( ___path[0:-1] )
        string ___extrinsic_1_T_0_path = string("");
        for( int _i = 0 ; _i<___path.size()-1 ; _i++ ) {
            ___extrinsic_1_T_0_path += ___path[_i] + "/";
        }
        ___extrinsic_1_T_0_path += "/"+extrinsic_1_T_0;
        cout << "___extrinsic_1_T_0_fullpath=" << ___extrinsic_1_T_0_path << endl;


        // Open fullpath of extrinsic.yaml
        cout << "opencv yaml reading: open file: " << ___extrinsic_1_T_0_path << endl;
        cv::FileStorage fs(___extrinsic_1_T_0_path, cv::FileStorage::READ);

        if (!fs.isOpened())
        {
            ROS_ERROR_STREAM(  "config_file asked to open extrinsicbasline file but it cannot be opened.\nTHIS IS FATAL, QUITING" );
            exit(1);
        }

        cout << TermColor::GREEN() << "successfully opened file "<< ___extrinsic_1_T_0_path << TermColor::RESET() << endl;
        cv::FileNode n = fs["transform"];
        if( n.empty() ) {
            cout << TermColor::RED() << "I was looking for the key `transform` in the file but it doesnt seem to exist. FATAL ERROR" << TermColor::RESET() << endl;
            exit(1);
        }
        Vector4d q_xyzw;
        q_xyzw << (double)n["q_x"] , (double)n["q_y"] ,(double) n["q_z"] ,(double) n["q_w"];

        Vector3d tr_xyz;
        tr_xyz << (double)n["t_x"] , (double)n["t_y"] ,(double) n["t_z"];

        cout << "--values from file--\n" << TermColor::iGREEN();
        cout << "q_xyzw:\n" << q_xyzw << endl;
        cout << "tr_xyz:\n" << tr_xyz << endl;
        cout << TermColor::RESET() << endl;

        Matrix4d _1_T_0;
        PoseManipUtils::raw_xyzw_to_eigenmat( q_xyzw, tr_xyz/1000., _1_T_0 ); cout << "translation divided by 1000 to convert from mm (in file) to meters (as needed)\n";
        // cout << TermColor::iBLUE() << "_1_T_0:\n" <<  _1_T_0  << TermColor::RESET() << endl;
        cout << TermColor::iBLUE() << "_1_T_0: " << PoseManipUtils::prettyprintMatrix4d( _1_T_0 ) << TermColor::RESET() << endl;
        cout << "_1_T_0:\n" << _1_T_0 << endl;


        dataManager.setCameraRelPose( _1_T_0, std::make_pair(1,0) );


        #if 0
        // verify if it was correctly set --> Works! Verified !
        cout << "isCameraRelPoseSet ? " << dataManager.isCameraRelPoseSet( std::make_pair(1,0) ) << endl;
        cout << "isCameraRelPoseSet ? " << dataManager.isCameraRelPoseSet( std::make_pair(6,10) ) << endl;
        // cout << "getCameraRelPose:\n" << dataManager.getCameraRelPose( std::make_pair(1,0) ) << endl;
        cout << "getCameraRelPose:\n" << PoseManipUtils::prettyprintMatrix4d( dataManager.getCameraRelPose( std::make_pair(1,0) ) ) << endl;
        auto ___all_available_keys = dataManager.getCameraRelPoseKeys();
        cout << "# relative poses available = " << ___all_available_keys.size() << endl;
        #endif
    }



    // [C]
    // imu_T_cam : imu camera extrinsic calib. Will store this just in case there is a need
    string extrinsic_cam_imu_topic = string("/vins_estimator/extrinsic");
    ROS_INFO( "Subscribe to extrinsic_cam_imu_topic: %s", extrinsic_cam_imu_topic.c_str() );
    ros::Subscriber sub_extrinsic = nh.subscribe( extrinsic_cam_imu_topic, 1000, &DataManager::extrinsic_cam_imu_callback, &dataManager );


    // [D]
    // PointCloud (all): has 5 channels
    string ptcld_topic = string("/vins_estimator/keyframe_point");
    // string ptcld_topic = string("/feature_tracker/feature");
    ROS_INFO( "Subscribe to ptcld_topic: %s", ptcld_topic.c_str() );
    ros::Subscriber sub_ptcld = nh.subscribe( ptcld_topic, 1000, &DataManager::ptcld_callback, &dataManager );

    // [E]
    // Tracked features. these contains a) unvn in msg->points b) chnl[0] id of the point c) chnl[1], chnl[2] := (u,v) d) chnl[3], chnl[4] not very sure.
    // string tracked_feat_topic = string("/feature_tracker/feature");
    // ROS_INFO( "Subscribe to tracked_feat_topic: %s", tracked_feat_topic.c_str() );
    // ros::Subscriber sub_tracked_feat = nh.subscribe( tracked_feat_topic, 1000, &DataManager::tracked_feat_callback, &dataManager );



    // set this to 1 to enable loading state, set this to 0 to not load state
    // If you dont loadStateFromDisk, make sure you initialize the ImageDataManager.
    #define __LOAD_STATE__ 0
    #if __LOAD_STATE__
    //--- Load State From Disk
    dataManager.loadStateFromDisk( "/Bulk_Data/cerebro_chkpts" );
    // cout << "PREMATURE EXIT\n";
    // exit(1);
    #else
    dataManager.getImageManagerRef()->initStashDir(true); // this will use the default /tmp/cerebro_stash as the stashing directory
    #endif


    //--- Start Threads ---//

    // [A]
    // Data associate thread: looks at the callback buffers and sets the data in the std::map
    dataManager.data_association_thread_enable();
    // dataManager.data_association_thread_disable();
    std::thread data_association_th( &DataManager::data_association_thread, &dataManager, 15 );

    dataManager.trial_thread_enable();
    dataManager.trial_thread_disable();
    std::thread dm_trial_th( &DataManager::trial_thread, &dataManager ); //< this threads prints datamanagers status to /dev/pts/20 modify as need be.

    // [A.1] Another thread in class dataManager which will deallocate images in nonkeyframes.
    dataManager.clean_up_useless_images_thread_enable();
    // dataManager.clean_up_useless_images_thread_disable();
    std::thread dm_cleanup_th( &DataManager::clean_up_useless_images_thread, &dataManager );


    Cerebro cer( nh );
    cer.setDataManager( &dataManager );
    cer.setPublishers( "/cerebro" );




    // [B.00]
    // Kidnap Message Publisher. See also comments below for `kidnap message subscriber`
    // Setup Publisher for sending kidnaped message to
    // a) vins_estimator and b) pose_graph solver
    string pub_topic_test = "/feature_tracker/rcvd_flag";
    ROS_INFO( "main : Publisher pub_topic_test: %s", pub_topic_test.c_str() );
    ros::Publisher rcvd_flag_pub = nh.advertise<std_msgs::Bool>(pub_topic_test, 1000);
    // We publish std_msgs::Header to the pose-graph-solver.
    // The purpose is that, this will serve as carrier of timestamp according to which
    // we can eliminate the odometry edges from the cost function.
    string pub_topic_header = "/feature_tracker/rcvd_flag_header";
    ROS_INFO( "main : Publisher pub_topic_header: %s", pub_topic_header.c_str() );
    ros::Publisher kidnap_indicator_header_pub = nh.advertise<std_msgs::Header>(pub_topic_header, 1000);
    dataManager.setKidnapIndicatorPublishers( rcvd_flag_pub, kidnap_indicator_header_pub );


    // [B.01]
    // Kidnap message subscriber. There are two kinds of kidnap messages
    // a. Bool b. Header. Look at the documentation of the kidnaped thread to know the details.
    // This has been done only for compatibility. I (mpkuse) prefer the Header because it
    // can also contain the timestamp when kidnap started and ended.
    string kidnap_bool_topic = "/feature_tracker/rcvd_flag";
    ROS_INFO( "Subscribe to kidnap_bool_topic: %s", kidnap_bool_topic.c_str() );
    ros::Subscriber sub_kidnap_bool = nh.subscribe( kidnap_bool_topic, 1000, &Cerebro::kidnap_bool_callback, &cer );

    string kidnap_header_topic = "/feature_tracker/rcvd_flag_header";
    ROS_INFO( "Subscribe to kidnap_header_topic: %s", kidnap_header_topic.c_str() );
    ros::Subscriber sub_kidnap_header = nh.subscribe( kidnap_header_topic, 1000, &Cerebro::kidnap_header_callback, &cer );


    // [B]
    // Descriptor Computation Thread.
    //      It monitors data_map. If new keyframes are available then
    //      it queries the ros-service with the image to get the whole-image-descriptor.
    //      This whole image descriptor is stored in `node->setWholeImageDescriptor`
    //      and the index of computation stored in `wholeImageComputedList`.
    cer.descriptor_computer_thread_enable();
    // cer.descriptor_computer_thread_disable();
    std::thread desc_th( &Cerebro::descriptor_computer_thread, &cer ); //runs @20hz


    // [C]
    // loopcandidates producer
    //      This thread looks at `wholeImageComputedList`, if there is something new
    //      in it (new descriptors) it does dot(  0-->T-50, T ), dot(  0-->T-50, T-1 )
    //      and dot(  0-->T-50, T-2 ). If threshold-test and locality-test both
    //      comeout to be +ve it declares 'loop found' and queues up this pair into the
    //      list `foundLoops`
    cer.run_thread_enable();
    // cer.run_thread_disable();
    std::thread dot_product_th( &Cerebro::run, &cer ); //< descrip_N__dot__descrip_0_N. runs @ 10hz


    // [C.1]
    // loopcandidates consumer
    //      Monitors the list `foundLoops` aka the putative loop candidates.
    //      If new candidates are present in the list it computes the relative-pose
    //      using GMSMatcher and theia-sfm's pnp. The depth is computed from stereogeom (class StereoGeometry)
    cer.loopcandidate_consumer_enable();
    // cer.loopcandidate_consumer_disable();
    std::thread loopcandidate_consumer_th( &Cerebro::loopcandiate_consumer_thread, &cer ); // runs @1hz

    // [D]
    // Kidnap Identification Thread
    cer.kidnaped_thread_enable();
    // cer.kidnaped_thread_disable();
    std::thread kidnap_th( &Cerebro::kidnaped_thread, &cer, 5 );

    // [E]
    // Visualization -
    Visualization viz(nh);
    viz.setDataManager( &dataManager );
    viz.setCerebro( &cer );
    viz.setVizPublishers( "/cerebro_node/viz/" );
    viz.run_thread_enable();
    // viz.run_thread_disable();
    std::thread viz_th( &Visualization::run, &viz, 10 ); //TODO something wrong with the logic in publish. another solution could be we keep #seq in DataNode.


    fs.release();

    ros::spin();

    // seem like user has pressed CTRL+C, so, stop all threads.
    dataManager.data_association_thread_disable();
    dataManager.trial_thread_disable();
    dataManager.clean_up_useless_images_thread_disable();
    #if 1
    cer.run_thread_disable();
    cer.descriptor_computer_thread_disable();
    cer.loopcandidate_consumer_disable();
    cer.kidnaped_thread_disable();
    viz.run_thread_disable();
    #endif

    data_association_th.join(); cout << "data_association_th.join()\n";
    dm_trial_th.join(); cout << "t1_trial.join()\n";
    dm_cleanup_th.join(); cout << "dm_cleanup_th.join()\n";

    #if 1
    dot_product_th.join(); cout << "dot_product_th.join()\n";
    desc_th.join(); cout << "desc_th.join()\n";
    loopcandidate_consumer_th.join(); cout << "loopcandidate_consumer_th.join()\n";
    kidnap_th.join(); cout << "kidnap_th.join()\n";
    viz_th.join(); cout << "viz_th.join()\n";
    #endif



    //make this to 1 to save state to file upon exit
    #define __SAVE_STATE__ 0
    #if __SAVE_STATE__
    // Save State (for relocalization)
    dataManager.saveStateToDisk( "/Bulk_Data/cerebro_chkpts" );
    #endif




    ///////////////////////////////////////////////////
    // A demo of how to look inside dataManager.     //
    ///////////////////////////////////////////////////
    #if 0
    {
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
    #define __LOGGING__ 0 // make this 1 to enable logging. 0 to disable logging. rememeber to catkin_make after this change
    #if __LOGGING__
    // Note: If using roslaunch to launch this node and when LOGGING is enabled,
    // roslaunch sends a sigterm and kills this thread when ros::ok() returns false ie.
    // when you press CTRL+C. The timeout is governed by roslaunch.
    //
    // If you wish to increase this timeout, you need to edit "/opt/ros/kinetic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py"
    // then edit these two vars.
    // _TIMEOUT_SIGINT = 15.0 #seconds
    // _TIMEOUT_SIGTERM = 2.0 #seconds
    //          ^^^^ Borrowed from : https://answers.ros.org/question/11353/how-to-delay-escalation-to-sig-term-after-sending-sig-int-to-roslaunch/

        // Write json log
        // string save_dir = "/Bulk_Data/_tmp/";
        string save_dir = "/tmp/_tmp/";

        ROS_INFO( "cerebro_node logging to : %s.\nThis will take upto 2 minutes.", save_dir.c_str());
        ROS_ERROR( "cerebro_node logging to : %s.\nThis will take upto 2 minutes.", save_dir.c_str());
        ROS_WARN( "cerebro_node logging to : %s.\nThis will take upto 2 minutes.", save_dir.c_str());

        //
        // $ rm -rf ${save_dir}
        // $ mkdir ${save_dir}
        #include <cstdlib>
        string system_cmd;

        system_cmd = string("rm -rf "+save_dir).c_str();
        // cout << TermColor::YELLOW() << system_cmd << TermColor::RESET() << endl;
        // int rm_dir_err = system( system_cmd.c_str() );
        const int rm_dir_err = RawFileIO::exec_cmd( system_cmd );

        if ( rm_dir_err == -1 )
        {
            cout << TermColor::RED() << "[cerebro_node] Error removing directory!\n" << TermColor::RESET() << endl;
            exit(1);
        }

        system_cmd = string("mkdir -p "+save_dir).c_str();
        // cout << TermColor::YELLOW() << system_cmd << TermColor::RESET() << endl;
        // const int dir_err = system( system_cmd.c_str() );
        const int dir_err = RawFileIO::exec_cmd( system_cmd );
        if ( dir_err == -1 )
        {
            cout << TermColor::RED() << "[cerebro_node] Error creating directory!\n" << TermColor::RESET() << endl;
            exit(1);
        }
        // done emptying the directory.


        RawFileIO::write_string( save_dir+"/log.json", dataManager.asJson().dump(4) );
        // RawFileIO::write_string( save_dir+"/log.json", dataManager.metaDataAsJson() );
        // RawFileIO::write_string( save_dir+"/log.txt", dataManager.metaDataAsFlatFile() ); //TODO remove. Since i can read json in python as well as c++ with ease, these is no point of storing stuff as txt
        // return 0;

        #if 1
        // std::map< ros::Time, DataNode* > data_map = dataManager.getDataMapRef(); //old - can remove
        auto data_map = dataManager.getDataMapRef();
        auto img_data_mgr = dataManager.getImageManagerRef();
        for( auto it = data_map->begin() ; it!= data_map->end() ; it++ )
        {
            int seq_id = std::distance( data_map->begin() , it );

            string fname = save_dir+"/"+to_string(seq_id );
            #if 0
            //old code where we used images inside Nodes.  - mark for removal
            if( it->second->isImageAvailable() )
                RawFileIO::write_image( fname+".jpg", it->second->getImage()  );

            // additional image
            if( it->second->isImageAvailable(1) )
                RawFileIO::write_image( fname+"_1.jpg", it->second->getImage(1)  );

            #else
            if( img_data_mgr->isImageRetrivable( "left_image", it->first  ) )
            {
                cv::Mat outimg;
                img_data_mgr->getImage( "left_image", it->first, outimg );
                RawFileIO::write_image( fname+".jpg", outimg  );
            }

            if( img_data_mgr->isImageRetrivable( "right_image", it->first  ) )
            {
                cv::Mat outimg;
                img_data_mgr->getImage( "right_image", it->first, outimg );
                RawFileIO::write_image( fname+"_1.jpg", outimg  );
            }
            #endif



            // Save VINS pose
            if( it->second->isPoseAvailable() ) {
                RawFileIO::write_EigenMatrix( fname+".wTc", it->second->getPose() );
            }

            // Save Point Cloud
            #if 0
            if( it->second->isPtCldAvailable() ) {
                RawFileIO::write_EigenMatrix( fname+".wX.pointcloud", it->second->getPointCloud() );
                if( it->second->isPoseAvailable() ) {
                    RawFileIO::write_EigenMatrix( fname+".cX.pointcloud", it->second->getPose().inverse() *  it->second->getPointCloud() );
                }
                else
                    ROS_WARN( "ptclod is available but pose is not available at seq_id=%d", seq_id );
            }
            #endif


            // Save Tracked Points
            #if 0
            if( it->second->isUVAvailable() ) {
                RawFileIO::write_EigenMatrix( fname+".unvn", it->second->getUnVn() );
                RawFileIO::write_EigenMatrix( fname+".uv", it->second->getUV() );
                RawFileIO::write_EigenMatrix( fname+".id", it->second->getFeatIds() );
            }

            // Save Whole Image Descriptor
            if( it->second->isWholeImageDescriptorAvailable() ) {
                RawFileIO::write_EigenMatrix( fname+".imgdesc", it->second->getWholeImageDescriptor() );
            }
            #endif
        }
        #endif


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

        // Save kidnap info
        json json_kidnap_info = cer.kidnap_info_as_json();
        RawFileIO::write_string( save_dir+"/kidnap_info.json", json_kidnap_info.dump(4) );



        // Save matching images from ProcessedLoopCandidate
        string cmd_ = string("mkdir -p ")+save_dir+"/matching/";
        RawFileIO::exec_cmd( cmd_ );


        //******
        // If you are looking to ***save*** these matchings image pairs
        // besure to set the #define just before `Cerebro::loopcandiate_consumer_thread()`.

        json all_procloops_json_obj;
        for( int i=0 ; i<cer.processedLoops_count() ; i++ ) {

            // Retrive i^{th} item
            ProcessedLoopCandidate c = cer.processedLoops_i( i );

            // if( c.isSet_3d2d__2T1 == false ) {
                // cout << "ignore ProcessedLoopCandidate[" << i << "] because c.isSet_3d2d__2T1 == false. aka poses computed from multiple methods didnt seem consistent\n";
                // continue;
            // }

            // Write all the logged images
            for( int l=0 ; l<c.debug_images.size() ; l++ ) {
                cv::Mat __imm = c.debug_images[l] ;
                if( __imm.rows > 0 && __imm.cols > 0 ) {
                    RawFileIO::write_image( save_dir+"/matching/im_pair_"+to_string(i)+"_"+c.debug_images_titles[l]+".jpg", __imm );
                }
                else {
                    cout << "in logging, matching_im_pair of ProcessedLoopCandidate[" << i << "] is not available\n";
                }
            }


            // Json of the object
            json procloops_json_obj;
            if( c.asJson(procloops_json_obj) ) {
                procloops_json_obj["processedLoops_i"] = i;
                all_procloops_json_obj.push_back( procloops_json_obj );
            }
            else {
                cout << "cannot make json for this  ProcessedLoopCandidate[" << i << "]\n";
            }

            // Write the final poses as files
            if( c.isSet_3d2d__2T1 )
                RawFileIO::write_EigenMatrix( save_dir+"/matching/im_pair_"+to_string(i)+".3d2d__2T1", c._3d2d__2T1 );

            // Write final pose status image.
            string __final_pose_status_im_string = "";
            if( c.isSet_3d2d__2T1 ) {
                __final_pose_status_im_string = "TRUE (poses were consistent);";
                __final_pose_status_im_string += PoseManipUtils::prettyprintMatrix4d( c._3d2d__2T1 );
            } else {
                __final_pose_status_im_string = "FALSE (non consistent poses);";
            }
            cv::Mat __final_pose_status_im = cv::Mat::zeros(cv::Size(400, 20), CV_8UC3);
            MiscUtils::append_status_image( __final_pose_status_im,  __final_pose_status_im_string );
            RawFileIO::write_image( save_dir+"/matching/im_pair_"+to_string(i)+"_aaconsistency_status.jpg", __final_pose_status_im );

        }
        RawFileIO::write_string( save_dir+"/matching/ProcessedLoopCandidate.json", all_procloops_json_obj.dump(4) );

    #endif




    return 0 ;

}
