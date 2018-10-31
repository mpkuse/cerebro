/** Main for cerebro node.

        This contains the main for the cerebro. It subscribes to various topics
        from VINS-estimator and setups of threads a) vis-thread, b) redis-association-thread, c) etc

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 26th Oct, 2018
**/


#include <ros/ros.h>
#include <ros/package.h>


#include "PinholeCamera.h"
#include "DataManager.h"
#include "Cerebro.h"
#include "Visualization.h"


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
    PinholeCamera camera = PinholeCamera( config_file );
    camera.printCameraInfo(2);


    //--- DataManager ---//
    DataManager dataManager = DataManager(nh);
    dataManager.setCamera(camera);


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



    //--- Start Threads ---// TODO: start data_association_thread in dataManager class?

    // [A]
    // Data associate thread: looks at the callback buffers and sets the data in the std::map
    dataManager.data_association_thread_enable();
    std::thread t1( &DataManager::data_association_thread, &dataManager, 50 );
    // TODO Another thread in class dataManager which will deallocate images in nonkeyframes.


    // [B]
    // Cerebro threads
    // Cerebro cer;
    // cer.setDataManager( &dataManager );
    // cer.run_thread_enable();
    // std::thread t2( &Cerebro::run, &cer );

    // cer.descriptor_computer_thread_enable();
    // std::thread desc_th( &Cerebro::descriptor_computer_thread, &cer );

    // [C]
    // Visualization
    Visualization viz(nh);
    viz.setDataManager( &dataManager );
    viz.setVizPublishers( "/cerebro_node/viz/" );
    viz.run_thread_enable();
    std::thread t3( &Visualization::run, &viz, 25 ); //TODO something wrong with the logic in publish. another solution could be we keep #seq in DataNode.


    fs.release();
    ros::spin();

    dataManager.data_association_thread_disable();
    // cer.run_thread_disable();
    // cer.descriptor_computer_thread_disable();
    viz.run_thread_disable();

    t1.join();
    // t2.join();
    // desc_th.join();
    t3.join();


    #if 0
    {
        // A demo of how to look inside dataManager.
        std::map< ros::Time, DataNode* > data_map = dataManager.getDataMapRef();
        for( auto it = data_map.begin() ; it!= data_map.end() ; it++ )
        {
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


    #if 0
    {
        string save_dir = "/Bulk_Data/_tmp/";

        // Loops through all images and cv::waitKey press s will write the image and other data to dump dir
        std::map< ros::Time, DataNode* > data_map = dataManager.getDataMapRef();
        for( auto it = data_map.begin() ; it!= data_map.end() ; it++ )
        {
            if( !it->second->isKeyFrame() )
                continue;

            int seq_id = std::distance( data_map.begin() , it );

            cout << "Map-Key: " << it->first - dataManager.getPose0Stamp() << "\tseq=" << seq_id << "\t" << it->first << endl;
            cout << "Map-Value:\n";
            it->second->prettyPrint(  );

            assert( it->second->isImageAvailable() );
            cv::imshow( "win", it->second->getImage() );

            if( it->second->isUVAvailable() && dataManager.getCameraRef().isValid() && it->second->isPoseAvailable() && it->second->isPtCldAvailable() )
            {
                // Plot observed uv on image
                assert( it->second->isUVAvailable() );
                cv::Mat dst;
                const cv::Mat image = it->second->getImage();

                const MatrixXd uv = it->second->getUV();
                cout << "uv.rows=" << uv.rows() << "\tuv.cols=" << uv.cols() << endl;

                MiscUtils::plot_point_sets( image, uv, dst, cv::Scalar(0,0,255) );
                cv::imshow( "dst_uv", dst );

                // Project 3d points
                MatrixXd reprojected_pts;
                assert( dataManager.getCameraRef().isValid()  && it->second->isPoseAvailable() && it->second->isPtCldAvailable() );

                MatrixXd cX;
                cX = it->second->getPose().inverse() * it->second->getPointCloud();

                dataManager.getCameraRef().perspectiveProject3DPoints( cX, reprojected_pts );
                cv::Mat dst2;
                MiscUtils::plot_point_sets( dst, reprojected_pts, dst2, cv::Scalar(255,0,0) );
                cv::imshow( "dst_PI( wX )", dst2 );

            }

            char key = cv::waitKey(0);
            cout << "<s> to save; <q> to quit; any key to continue\n";
            if( key == 'q')
                break;
            if( key == 's' )
            {
                string fname = save_dir+"/"+to_string( it->first.toSec() );
                // Save Image
                RawFileIO::write_image( fname+".jpg", it->second->getImage()  );

                // Save VINS pose
                // RawFileIO::write_EigenMatrix( fname+".wTc", it->second->getPose() );

                // Save Point Cloud
                // RawFileIO::write_EigenMatrix( fname+".wX.pointcloud", it->second->getPointCloud() );
                RawFileIO::write_EigenMatrix( fname+".cX.pointcloud", it->second->getPose().inverse() *  it->second->getPointCloud() );

                // Save Tracked Points
                // RawFileIO::write_EigenMatrix( fname+".unvn", it->second->getUnVn() );
                // RawFileIO::write_EigenMatrix( fname+".uv", it->second->getUV() );
                // RawFileIO::write_EigenMatrix( fname+".id", it->second->getFeatIds() );
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
    #if 1
        // Write json log
        string save_dir = "/Bulk_Data/_tmp/";
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
        }


        // Save Camera Matrix and IMUCamExtrinsic
        PinholeCamera _cam = dataManager.getCameraRef();
        if( _cam.isValid() ) {
            RawFileIO::write_EigenMatrix( save_dir+"/cameraIntrinsic.K", _cam.get_eK() );
            RawFileIO::write_EigenMatrix( save_dir+"/cameraIntrinsic.D", _cam.get_eD() );
            RawFileIO::write_string( save_dir+"/cameraIntrinsic.info", _cam.getCameraInfoAsJson() );
        }
        else {
            ROS_ERROR( "[cerebro_node.cpp] cam appear to be not set...something is seem wrong\n" );
        }

    #endif

    return 0 ;

}
