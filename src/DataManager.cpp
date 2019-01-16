#include "DataManager.h"

DataManager::DataManager(ros::NodeHandle &nh )
//: out_stream(ofstream("/dev/pts/0",ios::out) )
{
    this->nh = nh;
}



DataManager::DataManager(const DataManager &obj)
//: out_stream(ofstream("/dev/pts/0",ios::out) )
{
   cout << "Copy constructor allocating ptr." << endl;
}


// void DataManager::setCamera( const PinholeCamera& camera )
// {
//   this->camera = camera;
//
//   cout << "--- Camera Params from DataManager ---\n";
//   this->camera.printCameraInfo();
//   // cout << "K\n" << this->camera.e_K << endl;
//   // cout << "D\n" << this->camera.e_D << endl;
//   cout << "--- END\n";
// }


void DataManager::setAbstractCamera( camodocal::CameraPtr abs_camera, short cam_id )
{
    assert( cam_id >= 0 && "DataManager, you requested a camera with negative id which is an error. cam_id=0 for default camera and 1,2,.. for additional cameras.\n" );
    assert(abs_camera && "[DataManager::setCamera] in datamanager you are trying to set an invalid abstract camera. You need to loadFromYAML before you can set this camera\n");
    // this->abstract_camera = abs_camera;
    this->all_abstract_cameras[cam_id] = abs_camera;

    cout << "--- Abstract CameraParams(cam_id=" << cam_id << ") from DataManager ---\n";
    // cout << this->abstract_camera->parametersToString() << endl;
    cout << this->all_abstract_cameras[cam_id]->parametersToString() << endl;
    cout << "--- END\n";
}

camodocal::CameraPtr DataManager::getAbstractCameraRef(short cam_id)
{
    assert( cam_id >= 0 && "DataManager, you requested a camera with negative id which is an error. cam_id=0 for default camera and 1,2,.. for additional cameras.\n" );
    // assert( abstract_camera && "[DataManager::getAbstractCameraRef] you are requesting a camera reference before setting.\n" );
    // return abstract_camera;
    assert( isAbstractCameraSet(cam_id) && "[DataManager::getAbstractCameraRef] you are requesting a camera reference before setting.\n");
    return this->all_abstract_cameras[cam_id];
}

bool DataManager::isAbstractCameraSet(short cam_id)
{
    assert( cam_id >= 0 && "DataManager, you requested a camera with negative id which is an error. cam_id=0 for default camera and 1,2,.. for additional cameras.\n" );
    if( this->all_abstract_cameras.count( cam_id ) > 0 ) {
        if( this->all_abstract_cameras[cam_id] )
            return true;
        else
            return false;
    }
    else {
        return false;
    }
}

vector<short> DataManager::getAbstractCameraKeys() {
    vector<short> keys;
    for( auto it=this->all_abstract_cameras.begin(); it!=all_abstract_cameras.end() ; it++ ) {
        keys.push_back( it->first );
    }
    return keys;
}


void DataManager::setCameraRelPose( Matrix4d a_T_b, std::pair<int,int> pair_a_b )
{
    // assert a and b abstract cameras exisits
    assert( isAbstractCameraSet(pair_a_b.first) && isAbstractCameraSet(pair_a_b.second) && "in [DataManager::setCameraRelPose] one of the abstract-cameras were not set, even though you asked me to set their relative pose. You need to set the cameras first before you can set the relative poses.\n" );

    cout << "---DataManager::setCameraRelPose---\n";
    cout << "setting "<< pair_a_b.first << "_T_" << pair_a_b.second << " :::> " << PoseManipUtils::prettyprintMatrix4d( a_T_b ) << endl;

    cam_relative_poses[ pair_a_b ] = a_T_b;
    cout << "---DONE---\n";

}

bool DataManager::isCameraRelPoseSet( std::pair<int,int> pair_a_b )
{
    if( this->cam_relative_poses.count( pair_a_b ) > 0 )
        return true;
    else
        return false;
}

const Matrix4d& DataManager::getCameraRelPose( std::pair<int,int> pair_a_b )
{
    assert( isCameraRelPoseSet( pair_a_b ) && "[DataManager::getCameraRelPose] make sure the rel cam pose you are requesting is available\n" );
    if( !isCameraRelPoseSet( pair_a_b) ) {
        ROS_ERROR( "[DataManager::getCameraRelPose] make sure the rel cam pose you are requesting is available\nYou requested (%d,%d) which is not available", pair_a_b.first, pair_a_b.second );
        exit(2);
    }

    return this->cam_relative_poses[ pair_a_b ];
}


vector< std::pair<int,int> > DataManager::getCameraRelPoseKeys()
{
    vector< pair<int,int> > keys;
    for( auto it=this->cam_relative_poses.begin(); it!=cam_relative_poses.end() ; it++ ) {
        keys.push_back( it->first );
    }
    return keys;
}

const Matrix4d& DataManager::getIMUCamExtrinsic()
{
    std::lock_guard<std::mutex> lk(global_vars_mutex);
    return imu_T_cam;
}

bool DataManager::isIMUCamExtrinsicAvailable()
{
    std::lock_guard<std::mutex> lk(global_vars_mutex);
    return imu_T_cam_available;
}

const ros::Time DataManager::getIMUCamExtrinsicLastUpdated()
{
    std::lock_guard<std::mutex> lk(global_vars_mutex);
    return imu_T_cam_stamp;
}


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////// call backs //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// #define __DATAMANAGER_CALLBACK_PRINT( u ) u
#define __DATAMANAGER_CALLBACK_PRINT( u )
void DataManager::camera_pose_callback( const nav_msgs::Odometry::ConstPtr msg )
{
    if( pose_0_available == false ) { //record the 1st pose
        pose_0 = msg->header.stamp;
        pose_0_available = true;
    }
    //__DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/camera_pose_callback]" << msg->header.stamp << endl; )
    __DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/camera_pose_callback]" << msg->header.stamp-pose_0 << endl; )
    // push this to queue. Another thread will associate the data
    pose_buf.push( msg );
    return;

    Matrix4d w_T_c;
    PoseManipUtils::geometry_msgs_Pose_to_eigenmat( msg->pose.pose, w_T_c );
    cout << "w_T_c:\n"<< w_T_c << endl;
    char _fname[100];
    sprintf( _fname, "/Bulk_Data/_tmp/%lf.wTc", msg->header.stamp.toSec() );
    RawFileIO::write_EigenMatrix( _fname, w_T_c );
}

void DataManager::keyframe_pose_callback( const nav_msgs::Odometry::ConstPtr msg )
{
    //__DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/camera_pose_callback]" << msg->header.stamp << endl; )
    __DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/keyframe_pose_callback]" << msg->header.stamp-pose_0 << endl; )
    // push this to queue. Another thread will associate the data
    kf_pose_buf.push( msg );
}


void DataManager::raw_image_callback( const sensor_msgs::ImageConstPtr& msg )
{
    //__DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/raw_image_callback]" << msg->header.stamp << endl; )
    __DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/raw_image_callback]" << msg->header.stamp-pose_0 << endl; )
    img_buf.push( msg );
    return;


    // TODO: Remove this code.
    try{
        char _fname[100];
        sprintf( _fname, "/Bulk_Data/_tmp/%lf.png", msg->header.stamp.toSec() );
        cv::imwrite( _fname, cv_bridge::toCvShare(msg, "bgr8" )->image  );
        // cv::imshow( "view", cv_bridge::toCvShare(msg, "bgr8" )->image );
        // cv::waitKey(30);
    }
    catch( cv_bridge::Exception& e )
    {
        ROS_ERROR( "cannot convery from %s to 'bgr8' ", msg->encoding.c_str() );
    }
}

void DataManager::raw_image_callback_1( const sensor_msgs::ImageConstPtr& msg )
{
    __DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/raw_image_callback_1]" << msg->header.stamp-pose_0 << endl; )
    img_1_buf.push( msg );
    return;
}

void DataManager::extrinsic_cam_imu_callback( const nav_msgs::Odometry::ConstPtr msg )
{
    //__DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/extrinsic_cam_imu_callback]" << msg->header.stamp << endl; )
    __DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/extrinsic_cam_imu_callback]" << msg->header.stamp-pose_0 << endl; )
    extrinsic_cam_imu_buf.push( msg );
}



// there will be 5 channels. ch[0]: un, ch[1]: vn,  ch[2]: u, ch[3]: v.  ch[4]: globalid of the feature.
 // cout << "\tpoints.size() : "<< msg->points.size(); // this will be N (say 92)
 // cout << "\tchannels.size() : "<< msg->channels.size(); //this will be N (say 92)
 // cout << "\tchannels[0].size() : "<< msg->channels[0].values.size(); //this will be 5.
 // cout << "\n";
 // An Example Keypoint msg
     // ---
     // header:
     //   seq: 40
     //   stamp:
     //     secs: 1523613562
     //     nsecs: 530859947
     //   frame_id: world
     // points:
     //   -
     //     x: -7.59081602097
     //     y: 7.11367511749
     //     z: 2.85602664948
     //   .
     //   .
     //   .
     //   -
     //     x: -2.64935922623
     //     y: 0.853760659695
     //     z: 0.796766400337
     // channels:
     //   -
     //     name: ''
     //     values: [-0.06108921766281128, 0.02294199913740158, 310.8721618652344, 260.105712890625, 2.0]
     //     .
     //     .
     //     .
     //   -
     //     name: ''
     //     values: [-0.47983112931251526, 0.8081198334693909, 218.95481872558594, 435.47357177734375, 654.0]
     //   -
     //     name: ''
     //     values: [0.07728647440671921, 1.0073764324188232, 344.2176208496094, 473.7791442871094, 660.0]
     //   -
     //     name: ''
     //     values: [-0.6801641583442688, 0.10506453365087509, 159.75746154785156, 279.6077575683594, 663.0]
void DataManager::ptcld_callback( const sensor_msgs::PointCloud::ConstPtr msg )
{
    // __DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/ptcld_callback]" << msg->header.stamp << endl; )
    __DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/ptcld_callback]" << msg->header.stamp - pose_0 << endl; )
    ptcld_buf.push( msg );
    return;

    int npts = msg->points.size();
    MatrixXd ptcld = MatrixXd::Zero(3, npts );
    for( int i=0 ; i<npts ; i++ )
    {
        ptcld(0,i) = msg->points[i].x;
        ptcld(1,i) = msg->points[i].y;
        ptcld(2,i) = msg->points[i].z;
    }
    char _fname[100];
    sprintf( _fname, "/Bulk_Data/_tmp/%lf.pointcloud", msg->header.stamp.toSec() );
    RawFileIO::write_EigenMatrix( _fname, ptcld );
}


void DataManager::tracked_feat_callback( const sensor_msgs::PointCloud::ConstPtr msg )
{
    __DATAMANAGER_CALLBACK_PRINT( cout << "[cerebro/tracked_feat_callback]" << msg->header.stamp << endl; )
    trackedfeat_buf.push( msg );
}


std::string DataManager::metaDataAsFlatFile()
{
    std::stringstream buffer;
    buffer << "#seq,rel_stamp,stamp,isKeyFrame,isImageAvailable,isImageAvailable_1,isPoseAvailable,isPtCldAvailable,isUVAvailable\n";

    {
        // puttime
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        buffer << "#generated on " << DateAndTime::current_date_and_time() << endl;
    }

    std::map< ros::Time, DataNode* > __data_map = this->getDataMapRef();
    for( auto it = __data_map.begin() ; it!= __data_map.end() ; it++ )
    {
        int seq_id = std::distance( __data_map.begin() , it );
        buffer << seq_id << ",";
        buffer << it->first -  this->getPose0Stamp() << ",";
        buffer <<  it->first << ",";
        buffer << it->second->isKeyFrame() << ",";
        buffer << it->second->isImageAvailable() << ",";
        buffer << it->second->isImageAvailable(1) << ",";
        buffer << it->second->isPoseAvailable() << ",";
        buffer << it->second->isPtCldAvailable() << ",";
        buffer << it->second->isUVAvailable();
        buffer << "\n";
    }
    return buffer.str();

}

// TODO: another function which returns the nlohmann/json.hpp object.
std::string DataManager::metaDataAsJson()
{
    std::stringstream buffer;
    buffer << "{\n\"DataNodes\": [\n";
    std::map< ros::Time, DataNode* > __data_map = this->getDataMapRef();
    IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", ",\t\n");

    for( auto it = __data_map.begin() ; it!= __data_map.end() ; it++ )
    {
        if( it != __data_map.begin() )
            buffer << ",";

        int seq_id = std::distance( __data_map.begin() , it );
        buffer << "{\n";
        buffer << "\"stamp\": " << it->first << ",\n";
        buffer << "\"stamp_relative\": " << ( it->first -  this->getPose0Stamp() ).toSec() << ",\n";
        buffer << "\"seq\": " << seq_id << ",\n";

        buffer << "\"getT\": " << it->second->getT() << ",\n";
        buffer << "\"getT_image\": " << it->second->getT_image() << ",\n";
        buffer << "\"getT_image_1\": " << it->second->getT_image(1) << ",\n";
        buffer << "\"getT_pose\": " << it->second->getT_pose() << ",\n";
        buffer << "\"getT_uv\": " << it->second->getT_uv() << ",\n";

        buffer << "\"isKeyFrame\":" << it->second->isKeyFrame() << ",\n";
        buffer << "\"isImageAvailable\":" << it->second->isImageAvailable() << ",\n";
        buffer << "\"isImageAvailable_1\":" << it->second->isImageAvailable(1) << ",\n";
        buffer << "\"isPoseAvailable\":" << it->second->isPoseAvailable() << ",\n";
        buffer << "\"isPtCldAvailable\":" << it->second->isPtCldAvailable() << ",\n";
        buffer << "\"isUnVnAvailable\":" << it->second->isUnVnAvailable() << ",\n";
        buffer << "\"isUVAvailable\":" << it->second->isUVAvailable() << ",\n";
        buffer << "\"isFeatIdsAvailable\":" << it->second->isFeatIdsAvailable() << ",\n";
        buffer << "\"isWholeImageDescriptorAvailable\":" << it->second->isWholeImageDescriptorAvailable() << ",\n";

        // image
        if(  it->second->isImageAvailable() ) {
            const cv::Mat im = it->second->getImage();
            buffer << "\"image\": {";
                buffer << "\"rows\":" << im.rows << ", \n";
                buffer << "\"cols\":" << im.cols << ",\n";
                buffer << "\"channels\":" << im.channels() << ",\n";
                buffer << "\"type\": \"" << MiscUtils::type2str( im.type() ) << "\",\n";
                buffer << "\"stamp\":" << it->second->getT_image() << "\n";
            buffer << "},\n";
        }



        // pose
        if( it->second->isPoseAvailable() ) {
            const Matrix4d wTc = it->second->getPose();
            buffer << "\"w_T_c\": {";
                buffer << "\"data\": [" << wTc.format(CSVFormat) << "],\n";
                buffer << "\"rows\":" << wTc.rows() << ", \n";
                buffer << "\"cols\":" << wTc.cols() << ", \n";
                buffer << "\"stamp\":" << it->second->getT_pose() << "\n";
            buffer << "},\n";
        }


        // ptcld, unvn, uv , id
        if( it->second->isPtCldAvailable() ) {
            const MatrixXd wX = it->second->getPointCloud();
            buffer << "\"wX\": {";
                buffer << "\"data\": [" << wX.format(CSVFormat) << "],\n";
                buffer << "\"rows\":" << wX.rows() << ", \n";
                buffer << "\"cols\":" << wX.cols() << ", \n";
                buffer << "\"stamp\":" << it->second->getT_ptcld() << "\n";
            buffer << "},\n";

        }

        if( it->second->isPoseAvailable()  && it->second->isPtCldAvailable() )
        {

            MatrixXd cX = it->second->getPose().inverse() * it->second->getPointCloud();
            buffer << "\"cX\": {";
                buffer << "\"data\": [" << cX.format(CSVFormat) << "],\n";
                buffer << "\"rows\":" << cX.rows() << ", \n";
                buffer << "\"cols\":" << cX.cols() << ", \n";
                buffer << "\"stamp\":" << it->second->getT_ptcld() << "\n";
            buffer << "},\n";

        }

        if( it->second->isUnVnAvailable() ) {
            const MatrixXd unvn = it->second->getUnVn();
            buffer << "\"unvn\": {";
                buffer << "\"data\": [" << unvn.format(CSVFormat) << "],\n";
                buffer << "\"rows\":" << unvn.rows() << ", \n";
                buffer << "\"cols\":" << unvn.cols() << ", \n";
                buffer << "\"stamp\":" << it->second->getT_unvn() << "\n";
            buffer << "},\n";
        }

        if( it->second->isUVAvailable() ) {
            const MatrixXd uv = it->second->getUV();
            buffer << "\"uv\": {";
                buffer << "\"data\": [" << uv.format(CSVFormat) << "],\n";
                buffer << "\"rows\":" << uv.rows() << ", \n";
                buffer << "\"cols\":" << uv.cols() << ", \n";
                buffer << "\"stamp\":" << it->second->getT_uv() << "\n";
            buffer << "},\n";
        }

        if( it->second->isFeatIdsAvailable() ) {
            const VectorXi fids = it->second->getFeatIds();
            buffer << "\"fids\": {";
                buffer << "\"data\": [" << fids.format(CSVFormat) << "],\n";
                buffer << "\"rows\":" << fids.rows() << ", \n";
                buffer << "\"cols\":" << fids.cols() << ", \n";
                buffer << "\"stamp\":" << it->second->getT_uv() << "\n";
            buffer << "},\n";
        }

        // buffer << "\"getT\": " << "1\n";
        buffer << "\"getT\":" << it->second->getT() << "\n";


        buffer << "}\n";
    }
    buffer << "\n]\n";

    // global data
    buffer << ", \"global\": ";
    buffer << "{";
    buffer << "\"isPose0Available\": " << this->isPose0Available() << ", \n";
    buffer << "\"Pose0Stamp\": " << this->getPose0Stamp() << ", \n";
    // // imu_T_cam
    if( this->isIMUCamExtrinsicAvailable() ) {
        Matrix4d __imu_T_cam = this->getIMUCamExtrinsic();
        buffer << "\"imu_T_cam\": {";
            buffer << "\"data\": [" << __imu_T_cam.format(CSVFormat) << "],\n";
            buffer << "\"rows\":" << __imu_T_cam.rows() << ", \n";
            buffer << "\"cols\":" << __imu_T_cam.cols() << ", \n";
            buffer << "\"stamp\":" << this->getIMUCamExtrinsicLastUpdated() << "\n";
        buffer << "},\n";
    }


    if( this->isAbstractCameraSet() ) {
        buffer << "\"abstract_camera\": ";
        // buffer << "\"" << this->abstract_camera->parametersToString() << "\"";
        buffer << "\"OK. see yaml file\"";
        buffer << ",\n";
    }

    // TODO: Removal. PinholeCamera no more in use
    // const PinholeCamera _cam = this->getCameraRef();
    // const Matrix3d eK = _cam.get_eK();
    // const Vector4d eD = _cam.get_eD();
    //
    // if( _cam.isValid() )
    // {
    //     buffer << "\"eK\": {";
    //         buffer << "\"data\": [" << eK.format(CSVFormat) << "],\n";
    //         buffer << "\"rows\":" << eK.rows() << ", \n";
    //         buffer << "\"cols\":" << eK.cols() << "\n";
    //     buffer << "},\n";
    //
    //     buffer << "\"eD\": {";
    //         buffer << "\"data\": [" << eD.format(CSVFormat) << "],\n";
    //         buffer << "\"rows\":" << eD.rows() << ", \n";
    //         buffer << "\"cols\":" << eD.cols() << "\n";
    //     buffer << "},\n";
    // }

    buffer << "\"isIMUCamExtrinsicAvailable\": " << this->isIMUCamExtrinsicAvailable() << "\n";


    buffer << "}";



    buffer << " }";
    return buffer.str();

}


string DataManager::print_queue_size( int verbose=1 )
{
    std::stringstream buffer;

    if( verbose == 1)
    {
        buffer << "img_buf=" << img_buf.size() << "\t";
        buffer << "img_1_buf=" << img_1_buf.size() << "\t";
        buffer << "pose_buf=" << pose_buf.size() << "\t";
        buffer << "kf_pose_buf=" << kf_pose_buf.size() << "\t";
        buffer << "ptcld_buf=" << ptcld_buf.size() << "\t";
        buffer << "trackedfeat_buf=" << trackedfeat_buf.size() << "\t";
        buffer << "extrinsic_cam_imu_buf=" << extrinsic_cam_imu_buf.size() << "\t";
        buffer << endl;
    }

    if( verbose == 2 )
    {
        buffer << "img_buf=" << img_buf.size() << " (";
        if( img_buf.size()  > 0 ) {
            buffer << std::fixed << std::setprecision(4) << img_buf.front()->header.stamp-pose_0 << "-->";
            buffer << std::fixed << std::setprecision(4) << img_buf.back()->header.stamp-pose_0 << ";";
        }
        buffer << ")\t";

        buffer << "img_1_buf=" << img_1_buf.size() << " (";
        if( img_1_buf.size()  > 0 ) {
            buffer << std::fixed << std::setprecision(4) << img_1_buf.front()->header.stamp-pose_0 << "-->";
            buffer << std::fixed << std::setprecision(4) << img_1_buf.back()->header.stamp-pose_0 << ";";
        }
        buffer << ")\t";

        buffer << "pose_buf=" << pose_buf.size() << " (";
        if( pose_buf.size()  > 0 ) {
            buffer << std::fixed << std::setprecision(4) << pose_buf.front()->header.stamp-pose_0 << "-->";
            buffer << std::fixed << std::setprecision(4) << pose_buf.back()->header.stamp-pose_0 << ";";
        }
        buffer << ")\t";

        buffer << "kf_pose_buf=" << kf_pose_buf.size() << " (";
        if( kf_pose_buf.size()  > 0 ) {
            buffer << std::fixed << std::setprecision(4) << kf_pose_buf.front()->header.stamp-pose_0 << "-->";
            buffer << std::fixed << std::setprecision(4) << kf_pose_buf.back()->header.stamp-pose_0 << ";";
        }
        buffer << ")\t";

        buffer << "ptcld_buf=" << ptcld_buf.size() << " (";
        if( ptcld_buf.size()  > 0 ) {
            buffer << std::fixed << std::setprecision(4) << ptcld_buf.front()->header.stamp-pose_0 << "-->";
            buffer << std::fixed << std::setprecision(4) << ptcld_buf.back()->header.stamp-pose_0 << ";";
        }
        buffer << ")\t";

        buffer << "trackedfeat_buf=" << trackedfeat_buf.size() << " (";
        if( trackedfeat_buf.size()  > 0 ) {
            buffer << std::fixed << std::setprecision(4) << trackedfeat_buf.front()->header.stamp-pose_0 << "-->";
            buffer << std::fixed << std::setprecision(4) << trackedfeat_buf.back()->header.stamp-pose_0 << ";";
        }
        buffer << ")\t";

        buffer << "extrinsic_cam_imu_buf=" << extrinsic_cam_imu_buf.size() << " (";
        if( extrinsic_cam_imu_buf.size()  > 0 ) {
            buffer << std::fixed << std::setprecision(4) << extrinsic_cam_imu_buf.front()->header.stamp-pose_0 << "-->";
            buffer << std::fixed << std::setprecision(4) << extrinsic_cam_imu_buf.back()->header.stamp-pose_0 << ";";
        }
        buffer << ")\t";
        buffer << "\n";
    }

    if( verbose == 3 )
    {
        buffer << "img_buf.len=" << img_buf.size() << "\t";
        buffer << "pose_buf.len=" << pose_buf.size() << "\t";
        buffer << "kf_pose_buf.len=" << kf_pose_buf.size() << "\t";
        buffer << "ptcld_buf.len=" << ptcld_buf.size() << "\t";
        buffer << "trackedfeat_buf.len=" << trackedfeat_buf.size() << "\t";
        buffer << "extrinsic_cam_imu_buf.len=" << extrinsic_cam_imu_buf.size() << "\t";
        buffer << endl;

        if( img_buf.size() > 0 )
            buffer << "img_buf.back.t=" << img_buf.back()->header.stamp-pose_0 << "\t";
        if( pose_buf.size() > 0 )
            buffer << "pose_buf.back.t=" << pose_buf.back()->header.stamp-pose_0 << "\t";
        if( kf_pose_buf.size() > 0 )
            buffer << "kf_pose_buf.back.t=" << kf_pose_buf.back()->header.stamp-pose_0 << "\t";
        if( ptcld_buf.size() > 0 )
            buffer << "ptcld_buf.back.t=" << ptcld_buf.back()->header.stamp-pose_0 << "\t";
        if( trackedfeat_buf.size() > 0 )
            buffer << "trackedfeat_buf.back.t=" << trackedfeat_buf.back()->header.stamp-pose_0 << "\t";
        if( extrinsic_cam_imu_buf.size() > 0 )
            buffer << "extrinsic_cam_imu_buf.back.t=" << extrinsic_cam_imu_buf.back()->header.stamp-pose_0 << "\t";
        buffer << endl;

        if( img_buf.size() > 0 )
            buffer << "img_buf.front.t=" << img_buf.front()->header.stamp-pose_0 << "\t";
        if( pose_buf.size() > 0 )
            buffer << "pose_buf.front.t=" << pose_buf.front()->header.stamp-pose_0 << "\t";
        if( kf_pose_buf.size() > 0 )
            buffer << "kf_pose_buf.front.t=" << kf_pose_buf.front()->header.stamp-pose_0 << "\t";
        if( ptcld_buf.size() > 0 )
            buffer << "ptcld_buf.front.t=" << ptcld_buf.front()->header.stamp-pose_0 << "\t";
        if( trackedfeat_buf.size() > 0 )
            buffer << "trackedfeat_buf.front.t=" << trackedfeat_buf.front()->header.stamp-pose_0 << "\t";
        if( extrinsic_cam_imu_buf.size() > 0 )
            buffer << "extrinsic_cam_imu_buf.front.t=" << extrinsic_cam_imu_buf.front()->header.stamp-pose_0 << "\t";
        buffer << endl;
    }

    return buffer.str();
}


//////////////////////////// Callback ends /////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Thread mains /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// #define _12335_print( str ) cout << "[data_association_thread]" << str;
#define _12335_print( str ) ;
void DataManager::data_association_thread( int max_loop_rate_in_hz )
{
    assert( max_loop_rate_in_hz > 0 && max_loop_rate_in_hz < 200 && "[DataManager::data_association_thread] I am expecting the loop rate to be between 1-50" );
    _12335_print( "Start thread");
    assert( b_data_association_thread && "You have not enabled thread execution. Call function DataManager::data_association_thread_enable() before you spawn this thread\n");
    float requested_loop_time_ms = 1000. / max_loop_rate_in_hz;

    while( b_data_association_thread )
    {
        auto loop_start_at = std::chrono::high_resolution_clock::now();

        //------------------------ Process here--------------------------------
        // std::this_thread::sleep_for( std::chrono::milliseconds(2000) );
        _12335_print( "---\n" );
        _12335_print( print_queue_size(2 ) ); //< sizes of buffer queues
        _12335_print( "\t\tSize_of_data_map = "+ to_string( data_map.size() ) + "\n" );


        // deqeue all raw images and make DataNodes of each of them, s
        while( img_buf.size() > 0 ) {
            sensor_msgs::ImageConstPtr img_msg = img_buf.pop();
            // cout << "Added a DataNode in data_map with poped() rawimage t=" << img_msg->header.stamp - pose_0 << endl;
            DataNode * n = new DataNode( img_msg->header.stamp );
            n->setImageFromMsg( img_msg );

            data_map.insert( std::make_pair(img_msg->header.stamp, n) );
        }

        // dequeue additional raw images
        while( img_1_buf.size() > 0 ) {
            sensor_msgs::ImageConstPtr img_1_msg = img_1_buf.pop();
            ros::Time t = img_1_msg->header.stamp;

            if( data_map.count(t) > 0 ) {
                data_map.at( t )->setImageFromMsg( img_1_msg, 1 );
            }
            else {
                assert( false && "[DataManager::data_association_thread] attempting to set additional image into datanode. However that datanode is not found in the map. This cannot be happening\n");
            }
        }


        // dequeue all poses and set them to data_map
        while( pose_buf.size() > 0 ) {
             nav_msgs::Odometry::ConstPtr pose_msg = pose_buf.pop();
             ros::Time t = pose_msg->header.stamp;
            //  cout << "Attempt adding poped() pose in data_map with t=" << pose_msg->header.stamp -pose_0 << endl;

             // find the DataNode with this timestamp
             if( data_map.count( t ) > 0 ) {
                 // a Node seem to exist with this t.
                 data_map.at( t )->setPoseFromMsg( pose_msg );
             }
             else {
                 assert( false && "[DataManager::data_association_thread] data_map does not seem to contain the t of pose_msg. This cannot be happening\n");
             }
         }


        // dequeue all point clouds (these are at keyframes)
        while( ptcld_buf.size() > 0 ) {
            sensor_msgs::PointCloudConstPtr ptcld_msg = ptcld_buf.pop();
            ros::Time t = ptcld_msg->header.stamp;
            // cout << "Attempt adding poped() pointcloud in data_map at t=" << t - pose_0 << endl;

            // find the DataNode with this timestamp
            if( data_map.count( t ) > 0 ) {
                // a Node seem to exist with this t.
                data_map.at( t )->setPointCloudFromMsg( ptcld_msg );
                data_map.at( t )->setUnVnFromMsg( ptcld_msg );
                data_map.at( t )->setUVFromMsg( ptcld_msg );
                data_map.at( t )->setTrackedFeatIdsFromMsg( ptcld_msg );
                data_map.at( t )->setAsKeyFrame();
            }
            else {
                assert( false && "[DataManager::data_association_thread] data_map does not seem to contain the t of ptcld_msg. This cannot be happening\n");
            }
        }


        // Deal with cam_imu_extrinsic. Store the last in global variable of this class.
        bool flag = false;
        nav_msgs::OdometryConstPtr __msg;
        while( extrinsic_cam_imu_buf.size() > 0 ) {
            // dump all
            __msg = extrinsic_cam_imu_buf.pop();
            flag = true;
        }
        if( flag ) {
            std::lock_guard<std::mutex> lk(global_vars_mutex);
            PoseManipUtils::geometry_msgs_Pose_to_eigenmat( __msg->pose.pose, imu_T_cam );
            imu_T_cam_available = true;
            imu_T_cam_stamp = __msg->header.stamp;
        }





        //--------------------------- Done processing--------------------------
        auto loop_end_at = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ellapsed = loop_end_at-loop_start_at;

        // sleep_for( requested_loop_time - ellapsed )
        int sleep_for = int(requested_loop_time_ms - (float) ellapsed.count()) ;
        _12335_print( "Loop iteration done in "<< ellapsed.count() << " ms; sleep_for=" << sleep_for << " ms" << endl; )

        if( sleep_for > 0 )
            std::this_thread::sleep_for( std::chrono::milliseconds( sleep_for )  );
        else {
            _12335_print( "Queueing in thread `data_association_thread`" );
            ROS_WARN( "Queueing in thread `data_association_thread`" );
        }

    }

    _12335_print( "Finish thread");

}
