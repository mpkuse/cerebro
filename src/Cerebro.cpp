#include "Cerebro.h"


Cerebro::Cerebro( ros::NodeHandle& nh )
{
    b_run_thread = false;
    b_descriptor_computer_thread = false;
    this->nh = nh;

    connected_to_descriptor_server = false;
    descriptor_size_available = false;
}

void Cerebro::setDataManager( DataManager* dataManager )
{
    this->dataManager = dataManager;
    m_dataManager_available = true;
}

void Cerebro::setPublishers( const string base_topic_name )
{
    string pub_loopedge_topic = base_topic_name+"/loopedge";
    ROS_INFO( "[Cerebro::setPublishers] Publish <cerebro::LoopEdge> %s", pub_loopedge_topic.c_str() );
    pub_loopedge = nh.advertise<cerebro::LoopEdge>( pub_loopedge_topic, 1000 );

    m_pub_available = true;
}


// #define __Cerebro__run__( msg ) msg ;
#define __Cerebro__run__( msg ) ;
void Cerebro::run()
{
    descrip_N__dot__descrip_0_N();
}


///
/// This implements a simple loopclosure detection scheme based on dot-product of descriptor-vectors.
///
/// TODO: In the future more intelligent schemes can be experimented with. Besure to run those in new threads and disable this thread.
/// wholeImageComputedList is a list for which descriptors are computed. Similarly other threads can compute
/// scene-object labels, text etc etc in addition to currently computed whole-image-descriptor
void Cerebro::descrip_N__dot__descrip_0_N()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    assert( b_run_thread && "you need to call run_thread_enable() before run() can start executing\n" );

    ros::Rate rate(10);

    // wait until connected_to_descriptor_server=true and descriptor_size_available=true
    int wait_itr = 0;
    while( true ) {
        if( this->connected_to_descriptor_server && this->descriptor_size_available)
            break;
        __Cerebro__run__(cout << wait_itr << " [Cerebro::run]waiting for `descriptor_size_available` to be true\n";)
        rate.sleep();
        wait_itr++;
        if( wait_itr > 157 ) {
            __Cerebro__run__( cout << TermColor::RED() << "[Cerebro::run] `this->connected_to_descriptor_server && this->descriptor_size_available` has not become true dispite waiting for about 15sec. So quiting the run thread.\n" << TermColor::RESET(); )
            return;
        }
    }
    __Cerebro__run__( cout << TermColor::GREEN() <<"[Cerebro::run] descriptor_size=" << this->descriptor_size << "  connected_to_descriptor_server && descriptor_size_available" << TermColor::RESET() << endl; )
    assert( this->descriptor_size> 0 );

    int LOCALITY_THRESH = 8;
    float DOT_PROD_THRESH = 0.87;

    int l=0, last_l=0;
    int last_processed=0;
    MatrixXd M = MatrixXd::Zero( this->descriptor_size, 9000 ); // TODO: Need dynamic allocation here.
    cout << "[Cerebro::run] M.rows = " << M.rows() << "  M.cols=" << M.cols()  << endl;
    while( b_run_thread )
    {
        // rate.sleep();
        // continue;
        auto data_map = dataManager->getDataMapRef(); // this needs to be get every iteration else i dont get the ubdated values which are constantly being updated by other threads.
        {
            std::lock_guard<std::mutex> lk(m_wholeImageComputedList);
            l = wholeImageComputedList.size();
        }

        if( l - last_l < 3 ) {
            // cout << "nothing new\n";
            rate.sleep();
            continue;
        }

        __Cerebro__run__( cout << TermColor::RED() << "---" << TermColor::RESET() << endl; )
        __Cerebro__run__( cout << "l=" << l << endl; )

        VectorXd v, vm, vmm;
        {
            std::lock_guard<std::mutex> lk(m_wholeImageComputedList);
            assert(  data_map.count( wholeImageComputedList[l-1] ) > 0  &&
                     data_map.count( wholeImageComputedList[l-2] ) > 0  &&
                     data_map.count( wholeImageComputedList[l-3] ) > 0 &&
                     "either of l, l-1, l-2 is not available in the datamap"
                 );
            v   = data_map[ wholeImageComputedList[l-1] ]->getWholeImageDescriptor();
            vm  = data_map[ wholeImageComputedList[l-2] ]->getWholeImageDescriptor();
            vmm = data_map[ wholeImageComputedList[l-3] ]->getWholeImageDescriptor();
        }



        // This is very inefficient. Better to have a matrix-vector product and not getWholeImageDescriptor() all the time.
        assert( M.rows() == v.size() );
        assert( l < M.cols() );
        M.col( l-1 ) = v;
        M.col( l-2 ) = vm;
        M.col( l-3 ) = vmm;

        int k = l - 50; // given a stamp, l, get another stamp k. better make this to 200.

        //usable size of M is 8192xl, let k (k<l) be the length until which dot is needed by time.
        if( k > 5 ) {
            ElapsedTime timer;
            timer.tic();
            VectorXd u   = v.transpose() * M.leftCols( k );
            VectorXd um  = vm.transpose() * M.leftCols( k );
            VectorXd umm = vmm.transpose() * M.leftCols( k );
            __Cerebro__run__( cout << "<v , M[0 to "<< k << "]\t";)
            __Cerebro__run__( cout << "Done in (ms): " << timer.toc_milli() << endl; )

            double u_max = u.maxCoeff();
            double um_max = um.maxCoeff();
            double umm_max = umm.maxCoeff();
            int u_argmax=-1, um_argmax=-1, umm_argmax=-1;
            for( int ii=0 ; ii<u.size() ; ii++ ) {
                if( u(ii) == u_max ) u_argmax = ii;
                if( um(ii) == um_max ) um_argmax = ii;
                if( umm(ii) == umm_max ) umm_argmax = ii;
            }
            assert( u_argmax > 0 && um_argmax > 0 && umm_argmax > 0 );

            if( abs(u_argmax - um_argmax) < LOCALITY_THRESH && abs(u_argmax-umm_argmax) < 8 && u_max > DOT_PROD_THRESH  )
            {
                std::lock_guard<std::mutex> lk(m_wholeImageComputedList);


                __Cerebro__run__(
                cout << TermColor::RED() << "Loop FOUND!!" <<  u_argmax << "\t" << um_argmax << "\t" << umm_argmax << TermColor::RESET() << endl;
                cout << TermColor::RED() << "loop FOUND!! "
                                         <<  wholeImageComputedList[l-1]
                                         << "<" << u_max << ">"
                                         << wholeImageComputedList[u_argmax]
                                         << TermColor::RESET() << endl;
                                )

                  //TODO :
                  // publish the image pair based on a config_file_flag
                  // read flags for publish image, threshold(0.92), locality threshold (8) from file.

                {
                std::lock_guard<std::mutex> lk_foundloops(m_foundLoops);
                foundLoops.push_back( std::make_tuple( wholeImageComputedList[l-1], wholeImageComputedList[u_argmax], u_max ) );
                }
            }

        }
        else {
            __Cerebro__run__(
            cout << "[Cerebro::descrip_N__dot__descrip_0_N] do nothing. not seen enough yet.\n";
            )
        }


        last_l = l;
        rate.sleep();
    }


}

// #define __Cerebro__descriptor_computer_thread( msg ) msg
#define __Cerebro__descriptor_computer_thread( msg ) ;
void Cerebro::descriptor_computer_thread()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    assert( b_descriptor_computer_thread && "You need to call descriptor_computer_thread_enable() before spawning the thread\n" );


    // Service Call
    // Sample Code : https://github.com/mpkuse/cerebro/blob/master/src/unittest/unittest_rosservice_client.cpp
    connected_to_descriptor_server = false;
    descriptor_size_available = false;
    cout << "Attempt connecting to ros-service for 10sec (will give up after that)\n";
    ros::ServiceClient client = nh.serviceClient<cerebro::WholeImageDescriptorCompute>( "/whole_image_descriptor_compute" );
    client.waitForExistence( ros::Duration(10, 0) ); //wait maximum 10 sec
    if( !client.exists() ) {
        ROS_ERROR( "Connection to server NOT successful. Quiting the thread." );
        return;
    }
    else std::cout << TermColor::GREEN() <<  "Connection to ros-service ``" << client.getService() << "`` established" << TermColor::RESET() << endl;
    connected_to_descriptor_server = true;


    // Send a zeros image to the server just to know the descriptor size
    int nrows=-1, ncols=-1, desc_size=-1;
    {
        auto _abs_cam = dataManager->getAbstractCameraRef();
        assert( _abs_cam && "[Cerebro::descriptor_computer_thread] request from cerebro to access camera from dataManager is invalid. This means that camera was not yet set in dataManager\n");
        nrows = _abs_cam->imageHeight();
        ncols = _abs_cam->imageWidth();
        cv::Mat zero_image = cv::Mat::zeros( nrows, ncols, CV_8UC3 );
        // create zero image sensor_msgs::Image
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", zero_image).toImageMsg();
        image_msg->header.stamp = ros::Time::now();
        cerebro::WholeImageDescriptorCompute srv; //service message
        srv.request.ima = *image_msg;
        srv.request.a = 986;
        // make request to server
        if( client.call( srv ) ) {
            __Cerebro__descriptor_computer_thread(std::cout <<  "Received response from server\t";)
            __Cerebro__descriptor_computer_thread(std::cout << "desc.size=" << srv.response.desc.size() << std::endl;)
            assert( srv.response.desc.size() > 0 && "The received descriptor appear to be of zero length. This is a fatal error.\n" );
            desc_size = int( srv.response.desc.size() );
        }
    }
    assert( nrows > 0 && ncols > 0 && desc_size > 0 );
    cout << "[Cerebro::descriptor_computer_thread] nrows=" << nrows << "  ncols=" << ncols << "  desc_size=" << desc_size << endl;
    this->descriptor_size = int(desc_size);
    descriptor_size_available = true;





    ros::Rate rate(20);
    while( b_descriptor_computer_thread )
    {
        auto data_map = dataManager->getDataMapRef();
        if( data_map.begin() == data_map.end() ) {
            __Cerebro__descriptor_computer_thread( cout << "nothing to compute descriptor\n" );
            std::this_thread::sleep_for( std::chrono::milliseconds( 1000 )  );
            continue;
        }
        ros::Time lb = data_map.rbegin()->first - ros::Duration(10, 0); // look at recent 10sec.
        auto S = data_map.lower_bound( lb );
        auto E = data_map.end();
        for( auto it = S; it != E ; it++ ) {
            //descriptor does not exisit at this stamp, so compute it.
            // Here I compute the whole image descriptor only at keyframes, you may try something like, if the pose is available compute it.
            if( it->second->isWholeImageDescriptorAvailable() == false && it->second->isKeyFrame()  )
            {
                if( it->second->getNumberOfSuccessfullyTrackedFeatures() < 20 )
                {
                    __Cerebro__descriptor_computer_thread( cout << "[Cerebro::descriptor_computer_thread] skip computing whole-image-descriptor for this image because it appears to be a kidnapped image frame.\n" ; )
                    continue;
                }

                __Cerebro__descriptor_computer_thread(ElapsedTime _time);
                __Cerebro__descriptor_computer_thread(_time.tic());

                // use it->second->getImage() to compute descriptor. call the service
                const cv::Mat& image_curr = it->second->getImage();
                sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_curr).toImageMsg();
                image_msg->header.stamp = ros::Time( it->first );
                cerebro::WholeImageDescriptorCompute srv; //service message
                srv.request.ima = *image_msg;
                srv.request.a = 986;
                if( client.call( srv ) ) {
                    __Cerebro__descriptor_computer_thread(std::cout <<  "Received response from server\t";)
                    __Cerebro__descriptor_computer_thread(std::cout << "desc.size=" << srv.response.desc.size() << std::endl;)
                    assert( srv.response.desc.size() > 0 && "The received descriptor appear to be of zero length. This is a fatal error.\n" );

                    VectorXd vec( srv.response.desc.size() ); // allocated a vector
                    for( int j=0 ; j<srv.response.desc.size() ; j++ ) {
                        vec(j) = srv.response.desc[j];
                    }
                    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    it->second->setWholeImageDescriptor( vec );

                    {
                        std::lock_guard<std::mutex> lk(m_wholeImageComputedList);
                        this->wholeImageComputedList.push_back( it->first ); // note down where it was computed.
                    }
                    __Cerebro__descriptor_computer_thread(cout << "Computed descriptor at t=" << it->first - dataManager->getPose0Stamp() << "\t" << it->first << endl;)
                    __Cerebro__descriptor_computer_thread(std::cout << "Computed descriptor in (millisec) = " << _time.toc_milli()  << endl;)
                }
                else {
                    ROS_ERROR( "Failed to call ros service" );
                }
            }
        }

        rate.sleep();
    }


}



const int Cerebro::foundLoops_count() const
{
    std::lock_guard<std::mutex> lk(m_foundLoops);
    return foundLoops.size();
}

const std::tuple<ros::Time, ros::Time, double> Cerebro::foundLoops_i( int i) const
{
    std::lock_guard<std::mutex> lk(m_foundLoops);
    assert( i >= 0 && i<foundLoops.size() && "[Cerebro::foundLoops_i] you requested a loop on in range\n");
    return foundLoops[i];
}


json Cerebro::foundLoops_as_JSON()
{
    std::lock_guard<std::mutex> lk(m_foundLoops);

    json jsonout_obj;
    std::map< ros::Time, DataNode* > data_map = dataManager->getDataMapRef();

    int n_foundloops = foundLoops.size();
    for( int i=0 ; i<n_foundloops ; i++ ) {
        auto u = foundLoops[ i ];
        ros::Time t_curr = std::get<0>(u);
        ros::Time t_prev = std::get<1>(u);
        double score = std::get<2>(u);


        assert( data_map.count( t_curr ) > 0 && data_map.count( t_prev ) > 0  && "One or both of the timestamps in foundloops where not in the data_map. This cannot be happening...fatal...\n" );
        int idx_1 = std::distance( data_map.begin(), data_map.find( t_curr )  );
        int idx_2 = std::distance( data_map.begin(), data_map.find( t_prev )  );
        // cout << "loop#" << i << " of " << n_foundloops << ": ";
        // cout << t_curr << "(" << score << ")" << t_prev << endl;
        // cout << idx_1 << "<--->" << idx_2 << endl;

        json _cur_json_obj;
        _cur_json_obj["time_sec_a"] = t_curr.sec;
        _cur_json_obj["time_nsec_a"] = t_curr.nsec;
        _cur_json_obj["time_sec_b"] = t_prev.sec;
        _cur_json_obj["time_nsec_b"] = t_prev.nsec;
        _cur_json_obj["time_double_a"] = t_curr.toSec();
        _cur_json_obj["time_double_b"] = t_prev.toSec();
        _cur_json_obj["global_a"] = idx_1;
        _cur_json_obj["global_b"] = idx_2;
        _cur_json_obj["score"] = score;
        jsonout_obj.push_back( _cur_json_obj );
    }

    return jsonout_obj;

}




// #define __Cerebro__loopcandi_consumer__(msg) msg;
#define __Cerebro__loopcandi_consumer__(msg)  ;
// ^This will also imshow image-pairs with gms-matches marked.

#define __Cerebro__loopcandi_consumer__IMP( msg ) msg;
// #define __Cerebro__loopcandi_consumer__IMP( msg ) ;
// ^Important Text only printing


// #define __Cerebro__loopcandi_consumer__IMSHOW 0 // will not produce the images (ofcourse will not show as well)
#define __Cerebro__loopcandi_consumer__IMSHOW 1 // produce the images and log them, will not imshow
// #define __Cerebro__loopcandi_consumer__IMSHOW 2 // produce the images and imshow them, don't log

// Just uncomment it to disable consistency check.
// #define __Cerebro__loopcandi_consumer__no_pose_consistency_check
void Cerebro::loopcandiate_consumer_thread()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    assert( b_loopcandidate_consumer && "you need to call loopcandidate_consumer_enable() before loopcandiate_consumer_thread() can start executing\n" );


    // init StereoGeometry
    bool stereogeom_status = init_stereogeom();
    if( !stereogeom_status ) {
        assert( false && "[Cerebro::loopcandiate_consumer_thread()] cannot init_stereogeom\n");
        return;
    }
    // stereogeom->set_K( 375.0, 375.0, 376.0, 240.0 ); // set this to something sensible, best is to use left camera's K


    // init pt-feature-matcher


    ros::Rate rate(1);
    int prev_count = 0;
    int new_count = 0;
    ElapsedTime timer;

    while( b_loopcandidate_consumer )
    {
        new_count = foundLoops_count();

        if( new_count == prev_count ) {
            rate.sleep();
            continue;
        }

        __Cerebro__loopcandi_consumer__IMP(
        cout << TermColor::iGREEN() ;
        cout << "I see "<< new_count - prev_count << " new candidates. from i=["<< prev_count << "," << new_count-1 << "]\n";
        // cout << "Cerebro::loopcandiate_consumer_thread(); #loops=" << new_count << TermColor::RESET() << endl;
        cout << TermColor::RESET() ;
        )

        for( int j= prev_count ; j<=new_count-1 ; j++ )
        {
            ProcessedLoopCandidate proc_candi;

            timer.tic() ;
            #ifdef __Cerebro__loopcandi_consumer__no_pose_consistency_check
            bool proc___status = process_loop_candidate_imagepair( j, proc_candi );
            #else
            bool proc___status = process_loop_candidate_imagepair_consistent_pose_compute( j, proc_candi );
            #endif
            __Cerebro__loopcandi_consumer__IMP(
                cout << "\t" << timer.toc_milli() << "(ms) !! process_loop_candidate_imagepair()\n";
            )

            // the data is in the `proc_candi` which is put into a vector which is a class variable.
            if( proc___status )
            {
                std::lock_guard<std::mutex> lk(m_processedLoops);

                // publish proc_candi
                cerebro::LoopEdge loopedge_msg;
                #ifdef __Cerebro__loopcandi_consumer__no_pose_consistency_check
                bool __makemsg_status = proc_candi.makeLoopEdgeMsg( loopedge_msg );
                #else
                bool __makemsg_status = proc_candi.makeLoopEdgeMsgWithConsistencyCheck( loopedge_msg );
                #endif


                __Cerebro__loopcandi_consumer__IMP(
                cout << TermColor::iBLUE() <<  "__makemsg_status: " << __makemsg_status << "  publish loopedge_msg" << TermColor::RESET() << endl;
                )

                // publish only if msg was successfully made ==> all the candidate relative poses are consistent.
                if( __makemsg_status )
                {
                    __Cerebro__loopcandi_consumer__IMP( cout << TermColor::GREEN() << "publish loopedge_msg" << TermColor::RESET() << endl; )
                    pub_loopedge.publish( loopedge_msg );
                } else {
                    __Cerebro__loopcandi_consumer__IMP( cout << TermColor::RED() << "not publishing because __makemsg_status was false" << TermColor::RESET() << endl; )
                }

                // irrespective of where the poses are consistent or not add it to the list.
                processedloopcandi_list.push_back(  proc_candi );
                __Cerebro__loopcandi_consumer__IMP(cout  << "\tAdded to `processedloopcandi_list`"  << endl;)
            }
            else {
                __Cerebro__loopcandi_consumer__IMP( cout << "\tNot added to `processedloopcandi_list`, status was false\n" << endl; )
            }
        }

        prev_count = new_count;
        rate.sleep();

    }

    cout << "disable called, quitting loopcandiate_consumer_thread\n";

}


const int Cerebro::processedLoops_count() const
{
    std::lock_guard<std::mutex> lk(m_processedLoops);
    return processedloopcandi_list.size();

}

const ProcessedLoopCandidate& Cerebro::processedLoops_i( int i ) const
{
    std::lock_guard<std::mutex> lk(m_processedLoops);

    assert( i>=0 && i< processedloopcandi_list.size() && "[processedLoops_i] input i is not in the correct range" );
    if( i>=0 && i< processedloopcandi_list.size() ) {
        return processedloopcandi_list[ i ];
    }
    else {
        cout << TermColor::RED() << "[Cerebro::processedLoops_i] error you requested for processedloopcandidate idx=" << i << " but the length of the vector was " << processedloopcandi_list.size() << endl;
        exit(1);
    }
}



bool Cerebro::init_stereogeom()
{
    auto left_camera = dataManager->getAbstractCameraRef(0);
    if( !dataManager->isAbstractCameraSet(1) )
    {
        ROS_ERROR( "Stereo-camera right doesn't appear to be set. to use this thread, you need to have StereoGeometry. Edit the config file to set the 2nd camera and its baseline\n");
        return false;
    }
    auto right_camera = dataManager->getAbstractCameraRef(1);
    __Cerebro__loopcandi_consumer__(
    cout << TermColor::iGREEN() ;
    cout << "[Cerebro::loopcandiate_consumer_thread] left_camera\n" << left_camera->parametersToString() << endl;
    cout << "[Cerebro::loopcandiate_consumer_thread] right_camera\n" << right_camera->parametersToString() << endl;
    cout << TermColor::RESET();
    )

    Matrix4d right_T_left;
    if( !dataManager->isCameraRelPoseSet( std::make_pair(1,0)) )
    {
        ROS_ERROR( "stereo camera appears to be set but the baseline (stereo-extrinsic) is not specified in the config_file. In config file you need to set `extrinsic_1_T_0`");
        return false;
    }
    right_T_left = dataManager->getCameraRelPose( std::make_pair(1,0) );
    __Cerebro__loopcandi_consumer__(
    cout << TermColor::iGREEN() ;
    cout << "[Cerebro::loopcandiate_consumer_thread] right_T_left: " << PoseManipUtils::prettyprintMatrix4d( right_T_left ) << endl;
    cout << TermColor::RESET();
    )
    stereogeom = std::make_shared<StereoGeometry>( left_camera,right_camera,     right_T_left  );
    return true;
}


bool Cerebro::retrive_stereo_pair( DataNode* node, cv::Mat& left_image, cv::Mat& right_image, bool bgr2gray )
{
    // raw stereo-images in gray
    if( !(node->isImageAvailable()) || !(node->isImageAvailable(1)) ) {
        cout << TermColor::RED() << "[Cerebro::retrive_stereo_pair] Either of the node images (stereo-pair was not available). This is probably not fatal, this loopcandidate will be skipped." << TermColor::RESET() << endl;
        return false;
    }
    const cv::Mat& bgr_left_image = node->getImage();
    const cv::Mat& bgr_right_image = node->getImage(1);

    if( bgr2gray ) {

        if( bgr_left_image.channels() > 1 )
            cv::cvtColor( bgr_left_image, left_image, CV_BGR2GRAY );
        else
            left_image = bgr_left_image;

        if( bgr_right_image.channels() > 1 )
            cv::cvtColor( bgr_right_image, right_image, CV_BGR2GRAY );
        else
            right_image = bgr_right_image;
    }
    else {
        left_image = bgr_left_image;
        right_image = bgr_right_image;
    }

    #if 0
    cout << "bgr_left_image: " << MiscUtils::cvmat_info( bgr_left_image ) << "\t";
    cout << "bgr_right_image: " << MiscUtils::cvmat_info( bgr_right_image ) << "\t";
    cout << "left_image: " << MiscUtils::cvmat_info( left_image ) << "\t";
    cout << "right_image: " << MiscUtils::cvmat_info( right_image ) << "\t";
    cout << endl;
    #endif
    return true;

}



bool Cerebro::make_3d_2d_collection__using__pfmatches_and_disparity( const MatrixXd& uv, const cv::Mat& _3dImage_uv,     const MatrixXd& uv_d,
                            std::vector<Eigen::Vector2d>& feature_position_uv, std::vector<Eigen::Vector2d>& feature_position_uv_d,
                            std::vector<Eigen::Vector3d>& world_point )
{
    assert( (uv.cols() == uv_d.cols() ) && "[Cerebro::make_3d_2d_collection__using__pfmatches_and_disparity] pf-matches need to be of same length. You provided of different lengths\n" );
    assert( _3dImage_uv.type() == CV_32FC3 );

    if( uv.cols() != uv_d.cols() ) {
        cout << TermColor::RED() << "[Cerebro::make_3d_2d_collection__using__pfmatches_and_disparity] pf-matches need to be of same length. You provided of different lengths\n" << TermColor::RESET();
        return false;
    }

    if( _3dImage_uv.type() != CV_32FC3 && _3dImage_uv.rows <= 0 && _3dImage_uv.cols <= 0  ) {
        cout << TermColor::RED() << "[Cerebro::make_3d_2d_collection__using__pfmatches_and_disparity] _3dImage is expected to be of size CV_32FC3\n" << TermColor::RESET();
        return false;
    }



    int c = 0;
    MatrixXd ud_normalized = stereogeom->get_K().inverse() * uv_d;
    MatrixXd u_normalized = stereogeom->get_K().inverse() * uv;
    feature_position_uv.clear();
    feature_position_uv_d.clear();
    world_point.clear();

    for( int k=0 ; k<uv.cols() ; k++ )
    {
        cv::Vec3f _3dpt = _3dImage_uv.at<cv::Vec3f>( (int)uv(1,k), (int)uv(0,k) );
        if( _3dpt[2] < 0.1 || _3dpt[2] > 25.  )
            continue;

        c++;
        #if 0
        cout << TermColor::RED() << "---" << k << "---" << TermColor::RESET() << endl;
        cout << "ud=" << ud.col(k).transpose() ;
        cout << " <--> ";
        cout << "u=" << u.col(k).transpose() ;
        cout << "  3dpt of u=";
        cout <<  TermColor::GREEN() << _3dpt[0] << " " << _3dpt[1] << " " << _3dpt[2] << " " << TermColor::RESET();
        cout << endl;
        #endif

        feature_position_uv.push_back( Vector2d( u_normalized(0,k), u_normalized(1,k) ) );
        feature_position_uv_d.push_back( Vector2d( ud_normalized(0,k), ud_normalized(1,k) ) );
        world_point.push_back( Vector3d( _3dpt[0], _3dpt[1], _3dpt[2] ) );
    }
    __Cerebro__loopcandi_consumer__(
        cout << "of the total " << uv.cols() << " point feature correspondences " << c << " had valid depths\n";
    )
    return true;

    if( c < 30 ) {
        cout << TermColor::RED() << "[ Cerebro::make_3d_2d_collection__using__pfmatches_and_disparity]too few valid 3d points between frames" <<  TermColor::RESET() << endl;
        return false;
    }

}

bool Cerebro::process_loop_candidate_imagepair_consistent_pose_compute( int ii, ProcessedLoopCandidate& proc_candi )
{
    auto u = foundLoops_i( ii );
    ros::Time t_curr = std::get<0>(u);
    ros::Time t_prev = std::get<1>(u);
    double score = std::get<2>(u);

    auto data_map = dataManager->getDataMapRef();
    assert( data_map.count( t_curr ) > 0 && data_map.count( t_prev ) > 0  && "One or both of the timestamps in foundloops where not in the data_map. This cannot be happening...fatal...\n" );
    int idx_1 = std::distance( data_map.begin(), data_map.find( t_curr )  );
    int idx_2 = std::distance( data_map.begin(), data_map.find( t_prev )  );

    DataNode * node_1 = data_map.find( t_curr )->second;
    DataNode * node_2 = data_map.find( t_prev )->second;




    __Cerebro__loopcandi_consumer__IMP(
    cout << TermColor::BLUE() << "{"<<ii <<  "} process: "<< idx_1 << "<--->" << idx_2 << TermColor::RESET() << endl;
    )

    // if( node_1->getNumberOfSuccessfullyTrackedFeatures() < 20 || node_2->getNumberOfSuccessfullyTrackedFeatures() < 20 ) {
        // __Cerebro__loopcandi_consumer__IMP( "[Cerebro::process_loop_candidate_imagepair_consistent_pose_compute] skip processing this pair because it is a kidnaped node\n" );
        // return false;
    // }

    cv::Mat a_imleft_raw, a_imright_raw;
    bool ret_status_a = retrive_stereo_pair( node_1, a_imleft_raw, a_imright_raw );
    if( !ret_status_a )
        return false;

    cv::Mat b_imleft_raw, b_imright_raw;
    bool ret_status_b = retrive_stereo_pair( node_2, b_imleft_raw, b_imright_raw );
    if( !ret_status_b )
        return false;

    //------------------------------------------------
    //------ 3d points from frame_a
    //------------------------------------------------
    cv::Mat a_imleft_srectified, a_imright_srectified;
    cv::Mat a_3dImage;
    MatrixXd a_3dpts;
    cv::Mat a_disp_viz;
    stereogeom->get_srectifiedim_and_3dpoints_and_3dmap_and_disparity_from_raw_images(
        a_imleft_raw, a_imright_raw,
        a_imleft_srectified,a_imright_srectified,  a_3dpts, a_3dImage, a_disp_viz );


    //-----------------------------------------------------
    //--------------- 3d points from frame_b
    //-----------------------------------------------------
    cv::Mat b_imleft_srectified, b_imright_srectified;
    cv::Mat b_3dImage;
    MatrixXd b_3dpts;
    cv::Mat b_disp_viz;
    stereogeom->get_srectifiedim_and_3dpoints_and_3dmap_and_disparity_from_raw_images(
        b_imleft_raw, b_imright_raw,
        b_imleft_srectified, b_imright_srectified,  b_3dpts, b_3dImage, b_disp_viz );


    //---------------------------------------------------------------------
    //------------ point matches between a_left, b_left
    //---------------------------------------------------------------------
    MatrixXd uv, uv_d; // u is from frame_a; ud is from frame_b
    ElapsedTime timer;
    timer.tic();
    StaticPointFeatureMatching::gms_point_feature_matches(a_imleft_srectified, b_imleft_srectified, uv, uv_d );
    string msg_pf_matches = to_string( timer.toc_milli() )+" (ms) elapsed time for point_feature_matches computation and resulted in " + std::to_string(uv.cols()) + " number of point correspondences" ;
    __Cerebro__loopcandi_consumer__( cout << msg_pf_matches << endl; )
    if( uv.cols() < 150 ) {
        __Cerebro__loopcandi_consumer__IMP(
        cout << TermColor::RED() << "[Cerebro::process_loop_candidate_imagepair_consistent_pose_compute]too few gms matches between node#" << idx_1 << " and node#" << idx_2;
        cout << " contains pf_matches=" << uv.cols() << ", thresh=150, so I am rejecting this loopcandidate." << TermColor::RESET() << endl;
        )
        return false;
    }




    //----------------------------------------------------------------------
    //-------------- PNP and P3P
    //----------------------------------------------------------------------
    Matrix4d odom_b_T_a = node_2->getPose().inverse() * node_1->getPose();
    proc_candi = ProcessedLoopCandidate( ii, node_1, node_2 );
    proc_candi.idx_from_datamanager_1 = idx_1;
    proc_candi.idx_from_datamanager_2 = idx_2;
    proc_candi.pf_matches = uv.cols();

    //----- Option-A:
    __Cerebro__loopcandi_consumer__IMP( cout << TermColor::BLUE() << "Option-A" << TermColor::RESET() << endl; )
    std::vector<Eigen::Vector2d> feature_position_uv;
    std::vector<Eigen::Vector2d> feature_position_uv_d;
    std::vector<Eigen::Vector3d> world_point_uv;
    StaticPointFeatureMatching::make_3d_2d_collection__using__pfmatches_and_disparity(
        stereogeom, uv, a_3dImage, uv_d,
                                feature_position_uv, feature_position_uv_d, world_point_uv);
    Matrix4d op1__b_T_a; string pnp__msg;
    float pnp_goodness = StaticTheiaPoseCompute::PNP( world_point_uv, feature_position_uv_d, op1__b_T_a, pnp__msg  );
    __Cerebro__loopcandi_consumer__IMP(
    cout << TermColor::YELLOW() << "pnp_goodness=" << pnp_goodness << " op1__b_T_a = " << PoseManipUtils::prettyprintMatrix4d( op1__b_T_a ) << TermColor::RESET() << endl;
    )
    __Cerebro__loopcandi_consumer__(
    cout << pnp__msg << endl;
    )

    // reprojection debug image for op-1
    // plot( PI( op1__b_T_a * world_point_uv ) ) on imB
    #if (__Cerebro__loopcandi_consumer__IMSHOW == 1) || (__Cerebro__loopcandi_consumer__IMSHOW == 2)
    {
    MatrixXd PI_world_point_uv, PI_world_point_uv_odom;
    GeometryUtils::idealProjection( stereogeom->get_K(), op1__b_T_a, world_point_uv, PI_world_point_uv );
    GeometryUtils::idealProjection( stereogeom->get_K(), odom_b_T_a, world_point_uv, PI_world_point_uv_odom );
    cv::Mat pi_dst_img;
    MiscUtils::plot_point_sets( b_imleft_srectified, PI_world_point_uv, pi_dst_img, cv::Scalar(0,255,0), false  );
    MiscUtils::plot_point_sets( pi_dst_img, PI_world_point_uv_odom, cv::Scalar(255,0,0), false  );
    MiscUtils::plot_point_sets( pi_dst_img, uv, cv::Scalar(0,0,255), false  );
    MiscUtils::plot_point_sets( pi_dst_img, uv_d, cv::Scalar(255,0,255), false  );
    cv::resize(pi_dst_img,pi_dst_img, cv::Size(), 0.6, 0.6 );
    MiscUtils::append_status_image( pi_dst_img, string( "^this is image b=")+to_string(idx_2)+";plot( PI( op1__b_T_a * world_point_uv ) ) on imB in green;plot( PI( odom__b_T_a * world_point_uv ) ) on imB in blue;plot( uv ) on imB in red;plot( uv_d) on imB in pink");


    #if __Cerebro__loopcandi_consumer__IMSHOW == 1
    proc_candi.debug_images.push_back( pi_dst_img );
    proc_candi.debug_images_titles.push_back( "reprojection_debug_image_1" );
    #endif

    #if __Cerebro__loopcandi_consumer__IMSHOW == 2
    cv::imshow( "plot( PI( op1__b_T_a * world_point_uv ) ) on imB", pi_dst_img );
    #endif
    }

    #endif


    //----- Option-B:
    __Cerebro__loopcandi_consumer__IMP( cout << TermColor::BLUE() << "Option-B" << TermColor::RESET() << endl; )
    std::vector<Eigen::Vector3d> world_point_uv_d;
    StaticPointFeatureMatching::make_3d_2d_collection__using__pfmatches_and_disparity(
        stereogeom, uv_d, b_3dImage, uv,
                                feature_position_uv_d, feature_position_uv, world_point_uv_d);
    Matrix4d op2__a_T_b, op2__b_T_a; string pnp__msg_option_B;
    float pnp_goodness_optioN_B = StaticTheiaPoseCompute::PNP( world_point_uv_d, feature_position_uv, op2__a_T_b, pnp__msg_option_B  );
    op2__b_T_a = op2__a_T_b.inverse();
    __Cerebro__loopcandi_consumer__IMP(
    cout << TermColor::YELLOW() << pnp_goodness_optioN_B << " op2__a_T_b = " << PoseManipUtils::prettyprintMatrix4d( op2__a_T_b ) << TermColor::RESET() << endl;
    cout << TermColor::YELLOW() << pnp_goodness_optioN_B << " op2__b_T_a = " << PoseManipUtils::prettyprintMatrix4d( op2__b_T_a ) << TermColor::RESET() << endl;
    )
    __Cerebro__loopcandi_consumer__(
    cout << pnp__msg_option_B << endl;
    )


    // reprojection debug image for op-2
    // plot( PI( op2__a_T_b * world_point_uv_d ) ) on imA
    #if (__Cerebro__loopcandi_consumer__IMSHOW == 1) || (__Cerebro__loopcandi_consumer__IMSHOW == 2)
    {
    MatrixXd PI_world_point_uvd, PI_world_point_uvd_odom;
    GeometryUtils::idealProjection( stereogeom->get_K(), op2__a_T_b, world_point_uv_d, PI_world_point_uvd );
    GeometryUtils::idealProjection( stereogeom->get_K(), odom_b_T_a.inverse(), world_point_uv_d, PI_world_point_uvd_odom );
    cv::Mat pi_dst_img;
    MiscUtils::plot_point_sets( a_imleft_srectified, PI_world_point_uvd, pi_dst_img, cv::Scalar(0,255,0), false  );
    MiscUtils::plot_point_sets( pi_dst_img, PI_world_point_uvd_odom, cv::Scalar(255,0,0), false  );
    MiscUtils::plot_point_sets( pi_dst_img, uv, cv::Scalar(0,0,255), false  );
    MiscUtils::plot_point_sets( pi_dst_img, uv_d, cv::Scalar(255,0,255), false  );
    cv::resize(pi_dst_img,pi_dst_img, cv::Size(), 0.6, 0.6 );
    MiscUtils::append_status_image( pi_dst_img, string( "^this is image a=")+to_string(idx_1)+";plot( PI( op2__a_T_b * world_point_uv_d ) ) on imA in green;plot( PI( odom__b_T_a * world_point_uv ) ) on imA in blue;plot( uv ) on imA in red;plot( uv_d) on imA in pink");


    #if __Cerebro__loopcandi_consumer__IMSHOW == 1
    proc_candi.debug_images.push_back( pi_dst_img );
    proc_candi.debug_images_titles.push_back( "reprojection_debug_image_2" );
    #endif

    #if __Cerebro__loopcandi_consumer__IMSHOW == 2
    cv::imshow( "plot( PI( op2__a_T_b * world_point_uv_d ) ) on imA", pi_dst_img );
    #endif
    }

    #endif

    //----- Option-C
    __Cerebro__loopcandi_consumer__IMP( cout << TermColor::BLUE() << "Option-C" << TermColor::RESET() << endl; )
    vector< Vector3d> uv_X;
    vector< Vector3d> uvd_Y;
    StaticPointFeatureMatching::make_3d_3d_collection__using__pfmatches_and_disparity( uv, a_3dImage, uv_d, b_3dImage,
        uv_X, uvd_Y );
    Matrix4d icp_b_T_a; string p3p__msg;
    float p3p_goodness = StaticTheiaPoseCompute::P3P_ICP( uv_X, uvd_Y, icp_b_T_a, p3p__msg );
    __Cerebro__loopcandi_consumer__IMP(
    cout << TermColor::YELLOW() << p3p_goodness << " icp_b_T_a = " << PoseManipUtils::prettyprintMatrix4d( icp_b_T_a ) << TermColor::RESET() << endl;
    )
    __Cerebro__loopcandi_consumer__(
    cout << p3p__msg << endl;
    )


    // reprojection debug image for op-3
    // plot( PI( icp_b_T_a * world_point_uv ) ) on imB
    #if (__Cerebro__loopcandi_consumer__IMSHOW == 1) || (__Cerebro__loopcandi_consumer__IMSHOW == 2)
    {
    MatrixXd PI_world_point_uv, PI_world_point_uv_odom;
    GeometryUtils::idealProjection( stereogeom->get_K(), icp_b_T_a, world_point_uv, PI_world_point_uv );
    GeometryUtils::idealProjection( stereogeom->get_K(), odom_b_T_a, world_point_uv, PI_world_point_uv_odom );
    cv::Mat pi_dst_img;
    MiscUtils::plot_point_sets( b_imleft_srectified, PI_world_point_uv, pi_dst_img, cv::Scalar(0,255,0), false  );
    MiscUtils::plot_point_sets( pi_dst_img, PI_world_point_uv_odom, cv::Scalar(255,0,0), false  );
    MiscUtils::plot_point_sets( pi_dst_img, uv, cv::Scalar(0,0,255), false  );
    MiscUtils::plot_point_sets( pi_dst_img, uv_d, cv::Scalar(255,0,255), false  );
    cv::resize(pi_dst_img,pi_dst_img, cv::Size(), 0.5, 0.5 );
    MiscUtils::append_status_image( pi_dst_img, string( "^this is image b=")+to_string(idx_2)+";plot( PI( icp_b_T_a * world_point_uv ) ) on imB in green;plot( PI( odom__b_T_a * world_point_uv ) ) on imB in blue;plot( uv ) on imB in red;plot( uv_d) on imB in pink");


    #if __Cerebro__loopcandi_consumer__IMSHOW == 1
    proc_candi.debug_images.push_back( pi_dst_img );
    proc_candi.debug_images_titles.push_back( "reprojection_debug_image_3" );
    #endif

    #if __Cerebro__loopcandi_consumer__IMSHOW == 2
    cv::imshow( "plot( PI( icp_b_T_a * world_point_uv ) ) on imB", pi_dst_img );
    #endif
    }

    #endif

    // if( isnan( op1__b_T_a  ) ) {
    if( op1__b_T_a != op1__b_T_a  || op2__b_T_a != op2__b_T_a || icp_b_T_a != icp_b_T_a ) {
        __Cerebro__loopcandi_consumer__IMP( cout << "=======++++ one of the 3 ways had inf or nan in them. This is bad, so skip this loop candidate.\n"; );
        return false;
    }

    __Cerebro__loopcandi_consumer__IMP(
    cout << TermColor::YELLOW() << "odom_b_T_a = " << PoseManipUtils::prettyprintMatrix4d( odom_b_T_a ) << TermColor::RESET() << endl;
    cout << "|op1 - op2|" << PoseManipUtils::prettyprintMatrix4d( op1__b_T_a.inverse() * op2__b_T_a ) << endl;
    cout << "|op1 - icp|" << PoseManipUtils::prettyprintMatrix4d( op1__b_T_a.inverse() * icp_b_T_a ) << endl;
    cout << "|op2 - icp|" << PoseManipUtils::prettyprintMatrix4d( op2__b_T_a.inverse() * icp_b_T_a ) << endl;
    )





    //-----------------------------------------------------------------
    // Fill the output
    //-----------------------------------------------------------------

    // final pose
    // proc_candi._3d2d__2T1 = op1__b_T_a;
    // proc_candi.isSet_3d2d__2T1 = true;
    // proc_candi._3d2d__2T1__ransac_confidence = pnp_goodness;


    // Fill up all the poses which were computed
    // option-A
    proc_candi.opX_b_T_a.push_back( op1__b_T_a );
    proc_candi.opX_goodness.push_back( pnp_goodness );
    proc_candi.opX_b_T_a_name.push_back( "op1__b_T_a" );
    proc_candi.opX_b_T_a_debugmsg.push_back( pnp__msg );
    // Option-B
    proc_candi.opX_b_T_a.push_back( op2__b_T_a );
    proc_candi.opX_goodness.push_back( pnp_goodness_optioN_B );
    proc_candi.opX_b_T_a_name.push_back( "op2__b_T_a" );
    proc_candi.opX_b_T_a_debugmsg.push_back( pnp__msg_option_B );
    // Option-C
    proc_candi.opX_b_T_a.push_back( icp_b_T_a );
    proc_candi.opX_goodness.push_back( p3p_goodness );
    proc_candi.opX_b_T_a_name.push_back( "icp_b_T_a" );
    proc_candi.opX_b_T_a_debugmsg.push_back( p3p__msg );






    //-----------------------------------------------------------------
    // Build Viz Images
    //-----------------------------------------------------------------
    #if (__Cerebro__loopcandi_consumer__IMSHOW == 1) || (__Cerebro__loopcandi_consumer__IMSHOW == 2)

        ///------A : pf matching
    cv::Mat dst_feat_matches;
    MiscUtils::plot_point_pair( a_imleft_srectified, uv, idx_1,
                     b_imleft_srectified, uv_d, idx_2,
                        dst_feat_matches, 3, msg_pf_matches+";#pf-matches: "+to_string( uv.cols() )  );
    cv::resize(dst_feat_matches, dst_feat_matches, cv::Size(), 0.5, 0.5 );


        ///----------B : Disparities
    cv::Mat dst_disp;
    MiscUtils::side_by_side( a_disp_viz, b_disp_viz, dst_disp );
    MiscUtils::append_status_image( dst_disp, "a="+ to_string(idx_1)+"     b="+to_string(idx_2), .8 );
    cv::resize(dst_disp, dst_disp, cv::Size(), 0.5, 0.5 );
    // cv::imshow( "dst_disp", dst_disp );

    MiscUtils::append_status_image( dst_disp, "[b_T_a <-- PNP( 3d(a), uv(b))];"+pnp__msg+";[a_T_b <-- PNP( 3d(b), uv(a))];"+pnp__msg_option_B+";[icp_b_T_a <-- ICP(3d(a), 3d(b))];"+p3p__msg );

    MiscUtils::append_status_image( dst_disp, "odom_b_T_a:"+PoseManipUtils::prettyprintMatrix4d( odom_b_T_a ) );

        ///---------C: Reprojections
        //TODO

    #if __Cerebro__loopcandi_consumer__IMSHOW == 1
    proc_candi.debug_images.push_back( dst_feat_matches );
    proc_candi.debug_images_titles.push_back( "apf_matches" );
    proc_candi.debug_images.push_back( dst_disp );
    proc_candi.debug_images_titles.push_back( "dsparity_and_poses" );
    #endif

    #if __Cerebro__loopcandi_consumer__IMSHOW == 2
    cv::imshow( "dst_feat_matches", dst_feat_matches );
    cv::imshow( "dst_disp", dst_disp );
    cv::waitKey(30);
    #endif

    #endif


    return true;

}

bool Cerebro::process_loop_candidate_imagepair( int ii, ProcessedLoopCandidate& proc_candi )
{
    auto u = foundLoops_i( ii );
    ros::Time t_curr = std::get<0>(u);
    ros::Time t_prev = std::get<1>(u);
    double score = std::get<2>(u);

    auto data_map = dataManager->getDataMapRef();
    assert( data_map.count( t_curr ) > 0 && data_map.count( t_prev ) > 0  && "One or both of the timestamps in foundloops where not in the data_map. This cannot be happening...fatal...\n" );
    int idx_1 = std::distance( data_map.begin(), data_map.find( t_curr )  );
    int idx_2 = std::distance( data_map.begin(), data_map.find( t_prev )  );

    DataNode * node_1 = data_map.find( t_curr )->second;
    DataNode * node_2 = data_map.find( t_prev )->second;

    // cv::Mat im_1 = node_1->getImage();
    // cv::Mat im_2 = node_2->getImage();

    __Cerebro__loopcandi_consumer__IMP(
    cout << TermColor::BLUE() << "{"<<ii <<  "} process: "<< idx_1 << "<--->" << idx_2 << TermColor::RESET() << endl;
    )
    // cv::imshow( "im_1", im_1);
    // cv::imshow( "im_2", im_2);



    //----------------------------------------
    //---------Disparity of `idx_1`
    //----------------------------------------
    cv::Mat grey_im_1_left, grey_im_1_right;
    bool ret_status = retrive_stereo_pair( node_1, grey_im_1_left, grey_im_1_right );
    if( !ret_status )
        return false;


    // will get 3d points, stereo-rectified image, and disparity false colormap
    MatrixXd _3dpts__im1; //4xN. 3d points of im1
    cv::Mat _3dImage__im1;
    cv::Mat imleft_srectified, imright_srectified;
    cv::Mat disparity_for_visualization;
    ElapsedTime timer;
    timer.tic();
    stereogeom->get_srectifiedim_and_3dpoints_and_3dmap_and_disparity_from_raw_images( grey_im_1_left, grey_im_1_right,
         imleft_srectified,imright_srectified,
         _3dpts__im1, _3dImage__im1, disparity_for_visualization );
    __Cerebro__loopcandi_consumer__(
    cout << timer.toc_milli() << " (ms)!!  get_srectifiedim_and_3dpoints_and_disparity_from_raw_images\n";
    )





    //---------------- END Disparity of `idx_1`



    //--------------------------------------------------------------
    //------------ srectified of idx_2 image pair needed
    //--------------------------------------------------------------
    cv::Mat grey_im_2_left, grey_im_2_right;
    bool ret_status_2 = retrive_stereo_pair( node_2, grey_im_2_left, grey_im_2_right );
    if( !ret_status_2 )
        return false;

    cv::Mat im2_left_srectified, im2_right_srectified;
    timer.tic();
    // TODO: If feasible can also compute depth of im2, so that the pose computed can be verified for consistency.
    // TODO: accept the pose if pnp( 3d of idx_1, 2d of idx_2 ) === pnp( 2d of idx_1, 3d of idx_2 )
    stereogeom->do_stereo_rectification_of_raw_images( grey_im_2_left, grey_im_2_right,
                            im2_left_srectified, im2_right_srectified );
    __Cerebro__loopcandi_consumer__(
    cout << timer.toc_milli() << " (ms)!! do_stereo_rectification_of_raw_images\n";
    )
    // cv::imshow( "im2_left_srectified", im2_left_srectified );

    //------------- END srectified of idx_2 image pair needed



    //-----------------------------------------------------------------------
    //----------point_feature_matches for `idx_1` <--> `idx_2`
    // gms matcher
    //-----------------------------------------------------------------------
    MatrixXd uv, uv_d; // u is from frame_a; ud is from frame_b
    timer.tic();
    StaticPointFeatureMatching::gms_point_feature_matches( imleft_srectified, im2_left_srectified, uv, uv_d );
    auto elapsed_time_gms = timer.toc_milli();
    __Cerebro__loopcandi_consumer__(
    cout << elapsed_time_gms << " (ms)!! gms_point_feature_matches\n";
    )


    //--------------------- END Matcher ----------------------------------



    //---------------- make collection of 3d 2d points
    // 3d of `idx_1` <---> 2d of `idx_2`


    std::vector<Eigen::Vector2d> feature_position_uv;
    std::vector<Eigen::Vector2d> feature_position_uv_d;
    std::vector<Eigen::Vector3d> world_point;
    make_3d_2d_collection__using__pfmatches_and_disparity( uv, _3dImage__im1,     uv_d,
                                feature_position_uv, feature_position_uv_d, world_point );
    int n_valid_depths = world_point.size();




    // TODO 2d of idx_1 <---> 3d of idx_2





    // See if the matching can be rejected due to insufficient # of matches
    if( uv.cols() < 150 ) {
        __Cerebro__loopcandi_consumer__IMP(
        cout << TermColor::YELLOW() << "of the total "+std::to_string(uv.cols())+" point feature correspondences " +std::to_string(n_valid_depths)+ " had valid depths. " << " too few pf-matches from gms. Skip this. TH=80" << TermColor::RESET() << endl;
        )
        return false;
    }

    if( n_valid_depths < 150 ) {
        __Cerebro__loopcandi_consumer__IMP(
        cout << TermColor::YELLOW() << "of the total "+std::to_string(uv.cols())+" point feature correspondences " +std::to_string(n_valid_depths)+ " had valid depths. " << "too few pf-matches depths from gms. Skip this. TH=80" << TermColor::RESET() << endl;
        )
        return false;
    }

    // TODO assess u, ud, world_point for histogram spreads. a more spread in measurements will give more precise more. Also will be helpful to weedout bad poses

    //------------------------------------------
    //-------------- theia::pnp
    //------------------------------------------
    Matrix4d b_T_a; //< RESULT
    string pnp__msg = string(""); //< msg about pnp
    #if 1
    //--- DlsPnp
    std::vector<Eigen::Quaterniond> solution_rotations;
    std::vector<Eigen::Vector3d> solution_translations;
    timer.tic();
    theia::DlsPnp( feature_position_uv_d, world_point, &solution_rotations, &solution_translations  );
    auto elapsed_dls_pnp = timer.toc_milli() ;
    __Cerebro__loopcandi_consumer__(
    cout << elapsed_dls_pnp << " : (ms) : theia::DlsPnp done in\n";
    cout << "solutions count = " << solution_rotations.size() << " " << solution_translations.size() << endl;
    )

    if( solution_rotations.size() == 0 ) {
        __Cerebro__loopcandi_consumer__IMP(
        cout << TermColor::RED() << " theia::DlsPnp returns no solution" << TermColor::RESET() << endl;
        )
        return false;
    }

    if( solution_rotations.size() > 1 ) {
        __Cerebro__loopcandi_consumer__IMP(
        cout << TermColor::RED() << " theia::DlsPnp returns multiple solution" << TermColor::RESET() << endl;
        )
        return false;
    }

    // retrive solution
    b_T_a = Matrix4d::Identity();
    b_T_a.topLeftCorner(3,3) = solution_rotations[0].toRotationMatrix();
    b_T_a.col(3).topRows(3) = solution_translations[0];

    __Cerebro__loopcandi_consumer__IMP(
    // cout << "solution_T " << b_T_a << endl;
    cout << TermColor::GREEN() << "DlsPnp (b_T_a): " << PoseManipUtils::prettyprintMatrix4d( b_T_a ) << TermColor::RESET() << endl;
    // out_b_T_a = b_T_a;
    pnp__msg +=  "DlsPnp (b_T_a): " + PoseManipUtils::prettyprintMatrix4d( b_T_a ) + ";";
    pnp__msg += "  elapsed_dls_pnp="+to_string(elapsed_dls_pnp)+";";
    )
    #endif


    #if 1
    //--- DlsPnpWithRansac
    __Cerebro__loopcandi_consumer__( timer.tic() );
    // prep data
    vector<CorrespondencePair_3d2d> data_r;
    for( int i=0 ; i<world_point.size() ; i++ )
    {
        CorrespondencePair_3d2d _data;
        _data.a_X = world_point[i];
        _data.uv_d = feature_position_uv_d[i];
        data_r.push_back( _data );
    }

    // Specify RANSAC parameters.

    DlsPnpWithRansac dlspnp_estimator;
    RelativePose best_rel_pose;

    // Set the ransac parameters.
    theia::RansacParameters params;
    params.error_thresh = 0.02;
    params.min_inlier_ratio = 0.7;
    params.max_iterations = 50;
    params.min_iterations = 5;
    params.use_mle = true;

    // Create Ransac object, specifying the number of points to sample to
    // generate a model estimation.
    theia::Ransac<DlsPnpWithRansac> ransac_estimator(params, dlspnp_estimator);
    // Initialize must always be called!
    ransac_estimator.Initialize();

    theia::RansacSummary summary;
    ransac_estimator.Estimate(data_r, &best_rel_pose, &summary);

    auto elapsed_dls_pnp_ransac=timer.toc_milli();
    __Cerebro__loopcandi_consumer__IMP(
    cout << elapsed_dls_pnp_ransac << "(ms)!! DlsPnpWithRansac ElapsedTime includes data prep\n";
    cout << "Ransac:";
    // for( int i=0; i<summary.inliers.size() ; i++ )
        // cout << "\t" << i<<":"<< summary.inliers[i];
    cout << "\tnum_iterations=" << summary.num_iterations;
    cout << "\tconfidence=" << summary.confidence;
    cout << endl;
    cout << TermColor::GREEN() << "best solution (ransac) : "<< PoseManipUtils::prettyprintMatrix4d( best_rel_pose.b_T_a ) << TermColor::RESET() << endl;
    )
    pnp__msg +=  "DlsPnpWithRansac (best_rel_pose.b_T_a): " + PoseManipUtils::prettyprintMatrix4d( best_rel_pose.b_T_a ) + ";";
    pnp__msg += string("    num_iterations=")+to_string(summary.num_iterations)+"  confidence="+to_string(summary.confidence);
    pnp__msg += string( "   elapsed_dls_pnp_ransac (ms)=")+to_string(elapsed_dls_pnp_ransac)+";";


    b_T_a = best_rel_pose.b_T_a;
    #endif




    __Cerebro__loopcandi_consumer__(
    // cout << "solution_T " << b_T_a << endl;
    cout << "Final (b_T_a): " << PoseManipUtils::prettyprintMatrix4d( b_T_a ) << endl;
    // out_b_T_a = b_T_a;
    )


    // Fill the output
    proc_candi = ProcessedLoopCandidate( ii, node_1, node_2 );
    proc_candi.idx_from_datamanager_1 = idx_1;
    proc_candi.idx_from_datamanager_2 = idx_2;
    proc_candi.pf_matches = uv.cols();
    proc_candi._3d2d_n_pfvalid_depth = n_valid_depths;
    proc_candi._3d2d__2T1 = b_T_a;
    proc_candi.isSet_3d2d__2T1 = true;
    proc_candi._3d2d__2T1__ransac_confidence = summary.confidence;


    //################ All Plotting/Viz/Debug after this ##########################//


    //-----------------------------------------
    // plot point-feature matches.
    //------------------------------------------
    #if (__Cerebro__loopcandi_consumer__IMSHOW == 1) || (__Cerebro__loopcandi_consumer__IMSHOW == 2)
    cv::Mat dst_feat_matches;
    string msg = string("gms_matcher:")+"of the total "+std::to_string(uv.cols())+" point feature correspondences " +std::to_string(n_valid_depths)+ " had valid depths;"+"computed in (ms)"+to_string(elapsed_time_gms);
    msg += ";_3d2d_2T1: "+  PoseManipUtils::prettyprintMatrix4d( b_T_a ) ;
    MiscUtils::plot_point_pair( imleft_srectified, uv, idx_1,
                     im2_left_srectified, uv_d, idx_2,
                        dst_feat_matches, 3, msg  );
    cv::resize(dst_feat_matches, dst_feat_matches, cv::Size(), 0.5, 0.5 );

    #if __Cerebro__loopcandi_consumer__IMSHOW == 2
    cv::imshow( "dst_feat_matches", dst_feat_matches );
    #endif



    __Cerebro__loopcandi_consumer__(
    string msg2 = string("gms_matcher:")+"of the total "+std::to_string(uv.cols())+" point feature correspondences " +std::to_string(n_valid_depths)+ " had valid depths;"+"computed in (ms)"+to_string(elapsed_time_gms);
    msg2 += ";_3d2d_2T1: "+  PoseManipUtils::prettyprintMatrix4d( b_T_a ) ;
    cout << msg2 << endl;
    cout << "adding `cv::Mat dst_feat_matches` to `proc_candi.matching_im_pair`\n";
    )
    // proc_candi.matching_im_pair = dst_feat_matches;
    proc_candi.debug_images.push_back( dst_feat_matches );
    proc_candi.debug_images_titles.push_back( "matching_im_pair");

    #endif



    // verification image of the pose
    #if (__Cerebro__loopcandi_consumer__IMSHOW == 1) || (__Cerebro__loopcandi_consumer__IMSHOW == 2)
    //---------------------------------------------
    //--- Use the theia's estimated pose do PI( b_T_a * 3dpts ) and plot these
    //--- points on B. Also plot detected points of B
    //---------------------------------------------
    MatrixXd _3dpts_of_A_projectedonB = MatrixXd::Zero(3, world_point.size() );
    MatrixXd _detectedpts_of_B = MatrixXd::Zero(3, world_point.size() );
    MatrixXd _detectedpts_of_A = MatrixXd::Zero(3, world_point.size() );
    int n_good = 0;
    for( int i=0 ; i<world_point.size() ; i++ ) {
        Vector4d a_X_i;
        a_X_i << Vector3d(world_point[i]), 1.0;
        Vector4d b_X_i = b_T_a * a_X_i;
        Vector3d _X_i = b_X_i.topRows(3);
        _X_i /= _X_i(2); // Z-division for projection
        _3dpts_of_A_projectedonB.col(i) = stereogeom->get_K() * _X_i; //< scaling with camera-intrinsic matrix


        Vector3d _tmp;
        _tmp << feature_position_uv_d[i], 1.0 ;
        _detectedpts_of_B.col(i) = stereogeom->get_K() * _tmp;

        _tmp << feature_position_uv[i], 1.0 ;
        _detectedpts_of_A.col(i) = stereogeom->get_K() * _tmp;


        Vector3d delta = _3dpts_of_A_projectedonB.col(i) - _detectedpts_of_B.col(i);
        if( abs(delta(0)) < 2. && abs(delta(1)) < 2. ) {
            // cout << "  delta=" << delta.transpose();
            n_good++;
        }
        else {
            // cout << TermColor::RED() << "delta=" << delta.transpose() << TermColor::RESET();
        }
        // cout << endl;
    }
    // cout << "n_good=" <<n_good << endl;


    //------------------------------
    //--- Use relative pose of obometry to do PI( odom_b_T_a * 3dpts ) and plot these on image-b
    //--- load odometry poses
    //-------------------------------
    #if 1

    MatrixXd _3dpts_of_A_projectedonB_with_odom_rel_pose = MatrixXd::Zero(3, world_point.size() );
    if( node_1->isPoseAvailable() && node_2->isPoseAvailable() ) {

        Matrix4d odom_wTA = node_1->getPose();
        // cout << "wTa(odom)" << PoseManipUtils::prettyprintMatrix4d( wTa ) << endl;
        Matrix4d odom_wTB = node_2->getPose();
        // cout << "wTb(odom)" << PoseManipUtils::prettyprintMatrix4d( wTb ) << endl;
        Matrix4d odom_b_T_a = odom_wTB.inverse() * odom_wTA;


        __Cerebro__loopcandi_consumer__(
            cout << "odom_b_T_a" << PoseManipUtils::prettyprintMatrix4d( odom_b_T_a ) << endl;
        )
        pnp__msg += "odom_b_T_a "+PoseManipUtils::prettyprintMatrix4d( odom_b_T_a ) +";";

        auto a_timestamp = node_1->getT();
        auto b_timestamp = node_2->getT();
        pnp__msg += "a.timestamp: " + to_string(a_timestamp.toSec());
        pnp__msg += "    b.timestamp: " + to_string(b_timestamp.toSec()) + ";";

        // PI( odom_b_T_a * a_X )
        for( int i=0 ; i<world_point.size() ; i++ ) {
            Vector4d a_X_i;
            a_X_i << Vector3d(world_point[i]), 1.0;
            Vector4d b_X_i = odom_b_T_a * a_X_i;
            Vector3d _X_i = b_X_i.topRows(3);
            _X_i /= _X_i(2); // Z-division for projection
            _3dpts_of_A_projectedonB_with_odom_rel_pose.col(i) = stereogeom->get_K() * _X_i; //< scaling with camera-intrinsic matrix
        }

    }
    else {
        cout << TermColor::RED() << "[Cerebro::process_loop_candidate_imagepair] In plotting part needs poses from DataManager which is not available." << TermColor::RESET() << endl;
    }

    #endif




    // plot( B, ud )
    cv::Mat __dst;
    MiscUtils::plot_point_sets( im2_left_srectified, _detectedpts_of_B, __dst, cv::Scalar( 0,0,255), false, "_detectedpts_of_B (red) npts="+to_string(_detectedpts_of_B.cols()) );
    // cv::imshow( "plot( B, ud )", __dst );

    // plot( B, _3dpts_of_A_projectedonB )
    MiscUtils::plot_point_sets( __dst, _3dpts_of_A_projectedonB, cv::Scalar( 0,255,255), false, ";_3dpts_of_A_projectedonB, PI( b_T_a * X),(yellow)" );
    // MiscUtils::plot_point_sets( b_imleft_srectified, _3dpts_of_A_projectedonB, __dst, cv::Scalar( 0,255,255), false, "_3dpts_of_A_projectedonB" );

    MiscUtils::plot_point_sets( __dst, _detectedpts_of_A, cv::Scalar( 255,255,255), false, ";;_detectedpts_of_A(white)" );

    MiscUtils::plot_point_sets( __dst, _3dpts_of_A_projectedonB_with_odom_rel_pose, cv::Scalar( 255,0,0), false, ";;;_3dpts_of_A_projectedonB_with_odom_rel_pose, PI( odom_b_T_a * X),(blue)" );


    // status image for __dst.
    string status_msg = "im2_left_srectified;";
    status_msg += "_detectedpts_of_B (red);";
    status_msg += "_3dpts_of_A_projectedonB, PI( b_T_a * X),(yellow);";
    status_msg += "_detectedpts_of_A(white);";
    status_msg += "_3dpts_of_A_projectedonB_with_odom_rel_pose, PI( odom_b_T_a * X),(blue);";
    MiscUtils::append_status_image( __dst, status_msg+";"+pnp__msg );

    // proc_candi.pnp_verification_image = __dst;
    proc_candi.debug_images.push_back( __dst );
    proc_candi.debug_images_titles.push_back( "pnp_verification_image" );

    #if __Cerebro__loopcandi_consumer__IMSHOW == 2
    cv::imshow( "im2_left_srectified", __dst );
    #endif

    #endif



    // disparity and other images
    #if (__Cerebro__loopcandi_consumer__IMSHOW == 1) || (__Cerebro__loopcandi_consumer__IMSHOW == 2)
    // proc_candi.node_1_disparity_viz = disparity_for_visualization; // this is disparity of im1. see code way above
    proc_candi.debug_images.push_back( disparity_for_visualization );
    proc_candi.debug_images_titles.push_back( "node_1_disparity_viz" );

    #if __Cerebro__loopcandi_consumer__IMSHOW == 2
    // cv::imshow( "imleft_srectified", imleft_srectified );
    // cv::imshow( "imright_srectified", imright_srectified );
    cv::imshow( "im1_disparity_for_visualization", disparity_for_visualization );
    #endif
    #endif



    // Done
    #if __Cerebro__loopcandi_consumer__IMSHOW == 2
    cv::waitKey(50);
    #endif

    return true;





}

//////////// END pose computation //////////////////////////////

///////////////////////////////////////////////////////////
//////////////////////// KIDNAP ///////////////////////////
///////////////////////////////////////////////////////////

// Kidnap identification thread. This thread monitors dataManager->getDataMapRef().size
// for every new node added if there are zero tracked features means that, I have
// been kidnaped. It however declares kidnap only after 2 sec of kidnaped
// #define __Cerebro__kidnaped_thread__( msg ) msg;
#define __Cerebro__kidnaped_thread__( msg ) ;
void Cerebro::kidnaped_thread( int loop_rate_hz )
{
    if( loop_rate_hz <= 0 || loop_rate_hz >= 30 ) {
        cout << TermColor::RED() << "[Cerebro::kidnaped_thead] Invalid loop_rate_hz. Expected to be between [1,30]\n" << TermColor::RESET() << endl;
        return;
    }

    cout << TermColor::GREEN() << "Start  Cerebro::kipnaped_thead\n" << TermColor::RESET() << endl;


    if( dataManager->isKidnapIndicatorPubSet() == false ) {
        cout << TermColor::RED() << "FATAL ERROR in [Cerebro::kidnaped_thread] kidnap indicator ros::Publishers were not set\n";
        exit(2);
    }



    ros::Rate loop_rate( loop_rate_hz );
    int prev_count = 0, new_count = 0;
    ros::Time last_known_keyframe;

    // TODO: Move both of these to class variables and make them atomic. is_kidnapped_start is valid only when is_kidnapped==true
    bool is_kidnapped = false;
    bool is_kidnapped_more_than_n_sec = false;
    ros::Time is_kidnapped_start;


    // Related to handling to playing multiple bags one-after-another.
    bool first_data_received = false;
    bool waiting_for_next_bag_to_start = false;

    while( b_kidnaped_thread_enable ) {
        // Book Keeping
        prev_count = new_count;
        loop_rate.sleep();



        auto data_map = dataManager->getDataMapRef(); //< map< ros::Time, DataNode*>
        new_count = data_map.size();

        if( new_count <= prev_count ) {
            __Cerebro__kidnaped_thread__(cout << "[Cerebro::kidnaped_thread]Nothing new\n";)

            #if 0
            // Handling for multiple bags. The idea is, if no new messages then publish false and stop the vins_estimator.
            if( first_data_received ) {
                cout << "[Cerebro::kidnaped_thread] looks like a bag has ended and we are waiting for the next bag to start\n";

                if( waiting_for_next_bag_to_start == false ) {
                    cout << TermColor::RED() << "[Cerebro::kidnaped_thread] PUBLISH FALSE (t= " << data_map.rbegin()->first << " to indicate the vins_estimator to stop.\n" << TermColor::RESET();
                    // publish False (bool msg)
                    std_msgs::Bool bool_msg; bool_msg.data = false;
                    rcvd_flag_pub.publish( bool_msg );

                    // publish header message
                    std_msgs::Header header_msg;
                    header_msg.stamp = data_map.rbegin()->first;
                    header_msg.frame_id = "kidnapped";
                    kidnap_indicator_header_pub.publish( header_msg );

                }
                waiting_for_next_bag_to_start = true;
            }
            #endif
            continue;
        }

        ros::Time lb = data_map.rbegin()->first - ros::Duration(5, 0); // look at recent 5sec.
        // auto S = data_map.begin();
        auto S = data_map.lower_bound( lb );
        auto E = data_map.end();
        __Cerebro__kidnaped_thread__( cout << "S=" << S->first << "  E=" << E->first <<  endl; )

        for( auto it = S ; it != E ; it++ )
        {
            #if 0
            // Handling for multiple bags. The idea is, if no new messages then publish false and stop the vins_estimator.
            // If new messages are seen after stopiing vins_estimator, this is an indicator of new bag playing.
            if(first_data_received && waiting_for_next_bag_to_start ) {
                // this means a new bag has started. So, publish true
                cout << TermColor::GREEN() << "Looks to me that a new bag has started. PUBLISH TRUE with t=" << S->first << TermColor::RESET() << endl;

                // publish true to vins_estimator to indicate that it may resume the estimation with a new co-ordinate system.
                // Publish True
                __Cerebro__kidnaped_thread__( cout << "PUBLISH TRUE\n"; )
                std_msgs::Bool bool_msg; bool_msg.data = true;
                rcvd_flag_pub.publish( bool_msg );
                // rcvd_flag_pub.publish( true );

                // publish header msg
                std_msgs::Header header_msg;
                header_msg.stamp =  S->first ;
                header_msg.frame_id = "unkidnapped";
                kidnap_indicator_header_pub.publish( header_msg );
            }
            first_data_received = true;
            waiting_for_next_bag_to_start = false;
            #endif

            #if 0
            if( it->second->isImageAvailable() ) {
                cout << TermColor::GREEN();
            } else { cout << TermColor::BLUE() ; }

            if( it->second->isPoseAvailable() && it->second->getNumberOfSuccessfullyTrackedFeatures() < 0 ) {
                cout << "A";
            }
            if( !it->second->isPoseAvailable() && it->second->getNumberOfSuccessfullyTrackedFeatures() < 0 ) {
                cout << "B";
            }
            if( it->second->isPoseAvailable() && ! (it->second->getNumberOfSuccessfullyTrackedFeatures() < 0 ) ) {
                cout << "C";
            }
            if( !it->second->isPoseAvailable() && ! (it->second->getNumberOfSuccessfullyTrackedFeatures() < 0 )  ){
                cout << "D";
            }
            cout << TermColor::RESET() ;
            #endif


            int n_feats = it->second->getNumberOfSuccessfullyTrackedFeatures();
            if( it->first <= last_known_keyframe ||  n_feats < 0 )
                continue;

            last_known_keyframe = it->first;

            const int THRESH_N_FEATS = 24;
            const ros::Duration WAIT_BEFORE_DECLARING_AS_KIDNAP = ros::Duration(3.0);
            if( is_kidnapped==false && n_feats < THRESH_N_FEATS  ) {
                is_kidnapped = true;
                is_kidnapped_start = it->first;

                __Cerebro__kidnaped_thread__(
                cout << TermColor::RED() << "I am kidnapped t=" << it->first << TermColor::RESET() << endl;
                cout << "I think so because the number of tracked features (from feature tracker) have fallen to only "
                     << n_feats << ", the threshold was " << THRESH_N_FEATS
                     << ". However, I will wait for " << WAIT_BEFORE_DECLARING_AS_KIDNAP << " sec to declare kidnapped to vins_estimator." << endl;
                )
            }

            __Cerebro__kidnaped_thread__(
            if( is_kidnapped ) {
                    cout << "is_kidnapped=true t=" << it->first << "  n_feats=" << n_feats << endl;
            }
            )

            if( is_kidnapped && !is_kidnapped_more_than_n_sec && (it->first - is_kidnapped_start) > WAIT_BEFORE_DECLARING_AS_KIDNAP )
            {
                __Cerebro__kidnaped_thread__(
                cout << "Kidnapped for more than " << WAIT_BEFORE_DECLARING_AS_KIDNAP << "sec. I am quite confident that I have been kidnapped\n";
                cout << "PUBLISH FALSE\n";
                )
                is_kidnapped_more_than_n_sec = true;

                dataManager->PUBLISH__FALSE( is_kidnapped_start );

            }

            if( is_kidnapped && n_feats > THRESH_N_FEATS ) {
                __Cerebro__kidnaped_thread__(
                cout << TermColor::GREEN() << "Looks like i have been unkidnapped t=" << it->first << TermColor::RESET() << endl;
                )

                if( is_kidnapped_more_than_n_sec )
                {
                    // publish true to vins_estimator to indicate that it may resume the estimation with a new co-ordinate system.
                    dataManager->PUBLISH__TRUE( it->first );

                }
                is_kidnapped = false;
                is_kidnapped_more_than_n_sec = false;

            }
        }
        // cout << endl;


    }


    cout << TermColor::RED() << "Cerebro::kipnaped_thead Ends\n" << TermColor::RESET() << endl;
}


bool Cerebro::kidnap_info( int i, ros::Time& start_, ros::Time& end_ )
{
    std::lock_guard<std::mutex> lk(mutex_kidnap);
    if( i>=0 && i<start_of_kidnap.size() ) {
        start_ = start_of_kidnap[i];
        end_ = end_of_kidnap[i];
        return true;
    }
    else {
        start_ = ros::Time();
        end_ = ros::Time();
        return false;
    }
}

json Cerebro::kidnap_info_as_json()
{
    std::lock_guard<std::mutex> lk(mutex_kidnap);
    json json_obj;
    json_obj["meta_info"]["state_is_kidnapped"] = (bool) state_is_kidnapped;
    json_obj["meta_info"]["len_start_of_kidnap"] = start_of_kidnap.size();
    json_obj["meta_info"]["len_end_of_kidnap"] = end_of_kidnap.size();
    for( int i=0 ; i<start_of_kidnap.size() ; i++ )
    {
        json tmp;
        tmp["start_of_kidnap_sec"] = start_of_kidnap[i].sec;
        tmp["start_of_kidnap_nsec"] = start_of_kidnap[i].nsec;
        tmp["end_of_kidnap_sec"] = end_of_kidnap[i].sec;
        tmp["end_of_kidnap_nsec"] = end_of_kidnap[i].nsec;
        json_obj["info"].push_back( tmp );
    }
    return json_obj;
}

int Cerebro::n_kidnaps() //< will with return length of `start_of_kidnap` current state has to be inferred by call to `is_kidnapped()`
{
    std::lock_guard<std::mutex> lk(mutex_kidnap);
    return start_of_kidnap.size();
}

bool Cerebro::is_kidnapped(){ state_is_kidnapped; }


#define __CEREBRO_kidnap_callbacks(msg) msg;
// #define __CEREBRO_kidnap_callbacks(msg) ;

void Cerebro::kidnap_bool_callback( const std_msgs::BoolConstPtr& rcvd_bool )
{
    return; // ignore this.
    __CEREBRO_kidnap_callbacks( cout << TermColor::iCYAN() << "[Cerebro::kidnap_bool_callback] msg=" << (bool)rcvd_bool->data << TermColor::RESET() << endl; )
}

void Cerebro::kidnap_header_callback( const std_msgs::HeaderConstPtr& rcvd_header )
{
    __CEREBRO_kidnap_callbacks(
    cout << TermColor::iCYAN() << "[Cerebro::kidnap_header_callback]" ;
    cout << rcvd_header->stamp << ":" << rcvd_header->frame_id ;
    cout << TermColor::RESET() << endl;
    )

    if( rcvd_header->frame_id == "kidnapped" ) {
        std::lock_guard<std::mutex> lk(mutex_kidnap);
        this->state_is_kidnapped = true;
        start_of_kidnap.push_back( rcvd_header->stamp );
        __CEREBRO_kidnap_callbacks(
        cout << TermColor::iCYAN() << "start_of_kidnap.push_back("<< rcvd_header->stamp << ")" << TermColor::RESET() << endl;
        )
        return;
    }

    if( rcvd_header->frame_id == "unkidnapped" ) {
        std::lock_guard<std::mutex> lk(mutex_kidnap);
        this->state_is_kidnapped = false;
        end_of_kidnap.push_back( rcvd_header->stamp );
        __CEREBRO_kidnap_callbacks(
        cout << TermColor::iCYAN() << "end_of_kidnap.push_back("<< rcvd_header->stamp << ")" << TermColor::RESET() << endl;
        )
        return;
    }

    assert( false && "in Cerebro::kidnap_header_callback. rcvd_header->frame_id is something other than `kidnapped` or `unkidnapped`.");

}
