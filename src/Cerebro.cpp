#include "Cerebro.h"

//-------------------------------------------------------------//
//--- Setup Cerebro
//-------------------------------------------------------------//
Cerebro::Cerebro( ros::NodeHandle& nh )
{
    b_run_thread = false;
    b_descriptor_computer_thread = false;
    this->nh = nh;

    connected_to_descriptor_server = false;
    descriptor_size_available = false;
}

void Cerebro::readParamsFromFile( const cv::FileStorage& fs )
{
    cout << TermColor::GREEN() << "\n[Cerebro::readParamsFromFile]\n";

    if( !fs.isOpened() ) {
        cout << "[Cerebro::readParamsFromFile] The input FileStorage handle is not opened. I am expecting to receive an open handle to read from\n";
        exit(1);
    }


    //-------------------------------------//
    //---- Hypothesis Generation Params ---//
    //-------------------------------------//
    // bool SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK;
    // string SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR;
    fs["SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK"] >> this->SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK;
    fs["SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR"] >> this->SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR;

    if( this->SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK == true && RawFileIO::is_path_a_directory( this->SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR )==false )
    {
        // make sure the save directory exists
        cout << TermColor::RED() << "[Cerebro::readParamsFromFile] ERROR. You asked me to save the loop hypothesis \
            representative images to \
            SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR=" << SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR << \
            ", len=" << SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR.length() << " this however is not a valid directory.\
            If you want to save the loop hypothesis data make sure the directory exists\n" << TermColor::RESET();
        exit(1);

    }

    cout << "\tSAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK=" << this->SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK << endl;
    cout << "\tSAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR" << this->SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR << endl;




    //-------------------------------------//
    //----   Pose Computation Params    ---//
    //-------------------------------------//
    fs["SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES"] >> this->SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES;
    fs["SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX"] >> this->SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX;
    if( this->SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES == true && RawFileIO::is_path_a_directory(this->SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX) == false )
    {
        cout << TermColor::RED() << "[Cerebro::readParamsFromFile] ERROR. you asked me to save reprojection images to disk in \
            SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX=" << SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX << ", this directory however doesnot exist\
            To use this feature the directory need to exist and be writable\n";
            exit(1);
    }

    cout << "\tSAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES=" << this->SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES << endl;
    cout << "\tSAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX=" << this->SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX << endl;

    cout << TermColor::GREEN() << "\n[Cerebro::readParamsFromFile] ENDS\n";
    // exit(4);
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

//-------------------------------------------------------------//
//--- END Setup Cerebro
//-------------------------------------------------------------//


//------------------------------------------------------------------//
// per Keyframe Descriptor Computation
//     Can have more threads to compute other aspects for keyframes
//     like texts, objects visible etc. TODO
//------------------------------------------------------------------//

#define __Cerebro__descriptor_computer_thread( msg ) ;
// #define __Cerebro__descriptor_computer_thread( msg ) msg

#define __Cerebro__descriptor_computer_thread__imp( msg ) ;
// #define __Cerebro__descriptor_computer_thread__imp( msg ) msg;
void Cerebro::descriptor_computer_thread()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    assert( b_descriptor_computer_thread && "You need to call descriptor_computer_thread_enable() before spawning the thread\n" );

    //---------
    // Send a zeros image to the server just to know the descriptor size
    int nrows=-1, ncols=-1, desc_size=-1;
    int nChannels = 1; //< ***This needs to be set correctly depending on the model_type. If you need error messages from server about channels size, you need to change this. ****
    //----------

    // Service Call
    // Sample Code : https://github.com/mpkuse/cerebro/blob/master/src/unittest/unittest_rosservice_client.cpp
    connected_to_descriptor_server = false;
    descriptor_size_available = false;
    int n_sec_wait_for_connection = 71;
    cout << TermColor::iYELLOW() << "[Cerebro::descriptor_computer_thread]Attempt connecting to ros-service for " << n_sec_wait_for_connection << " sec (will give up after that)\n" << TermColor::RESET();
    ros::ServiceClient client = nh.serviceClient<cerebro::WholeImageDescriptorCompute>( "/whole_image_descriptor_compute" );
    client.waitForExistence( ros::Duration(n_sec_wait_for_connection, 0) ); //wait maximum 10 sec
    if( !client.exists() ) {
        ROS_ERROR( "[Cerebro::descriptor_computer_thread]Connection to server NOT successful. I tried waiting for %d sec, still couldnt establish connection. Quiting the thread.", n_sec_wait_for_connection );
        return;
    }
    else std::cout << TermColor::GREEN() <<  "Connection to ros-service ``" << client.getService() << "`` established" << TermColor::RESET() << endl;
    connected_to_descriptor_server = true;


    ElapsedTime _est_desc_compute_time_ms;
    {
        auto _abs_cam = dataManager->getAbstractCameraRef();
        assert( _abs_cam && "[Cerebro::descriptor_computer_thread] request from cerebro to access camera from dataManager is invalid. This means that camera was not yet set in dataManager\n");
        nrows = _abs_cam->imageHeight();
        ncols = _abs_cam->imageWidth();
        cv::Mat zero_image;
        if( nChannels == 3 )
            zero_image = cv::Mat::zeros( nrows, ncols, CV_8UC3 );
        else if( nChannels == 1 )
            zero_image = cv::Mat::zeros( nrows, ncols, CV_8UC1 );
        else {
            assert( false && "Invalid number of channels specified in Cerebro::descriptor_computer_thread() ");
            cout << TermColor::RED() << "[ERROR] Ax Invalid number of channels specified in Cerebro::descriptor_computer_thread() " << endl << TermColor::RESET();
            exit(3);
        }

        // create zero image sensor_msgs::Image
        sensor_msgs::ImagePtr image_msg;
        if( nChannels == 3 )
            image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", zero_image).toImageMsg();
        else if( nChannels == 1 )
            image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", zero_image).toImageMsg();
        else {
            assert( false && "Invalid number of channels specified in Cerebro::descriptor_computer_thread() ");
            cout << TermColor::RED() << "[ERROR] Bx Invalid number of channels specified in Cerebro::descriptor_computer_thread() " << endl << TermColor::RESET();
            exit(3);
        }
        image_msg->header.stamp = ros::Time::now();
        cerebro::WholeImageDescriptorCompute srv; //service message
        srv.request.ima = *image_msg;
        srv.request.a = 986;
        // make request to server

        _est_desc_compute_time_ms.tic();
        if( client.call( srv ) ) {
            __Cerebro__descriptor_computer_thread(std::cout <<  "Received response from server\t";)
            __Cerebro__descriptor_computer_thread(std::cout << "desc.size=" << srv.response.desc.size() << std::endl;)
            assert( srv.response.desc.size() > 0 && "The received descriptor appear to be of zero length. This is a fatal error.\n" );
            desc_size = int( srv.response.desc.size() );
        }
    }
    assert( nrows > 0 && ncols > 0 && desc_size > 0 );
    int estimated_descriptor_compute_time_ms = _est_desc_compute_time_ms.toc_milli();
    cout << "[Cerebro::descriptor_computer_thread] nrows=" << nrows << "  ncols=" << ncols << " nChannels=" << nChannels << " ===>  desc_size=" << desc_size << "\t estimated_descriptor_compute_time_ms=" << estimated_descriptor_compute_time_ms << endl;
    this->descriptor_size = int(desc_size);
    descriptor_size_available = true;



    ros::Rate rate(20);
    auto data_map = dataManager->getDataMapRef();
    auto img_data_mgr = dataManager->getImageManagerRef();

    #if 1
    //*****************
    // If loadStateFromDisk, ie. if data_map already has a lot of items means that the
    // state was preloaded. in this case just setup this thread appropriately.
    //**************************
    if( data_map->begin() == data_map->end() )
    {
        // This means the data_map is empty, which inturn means fresh run
        cout << TermColor::iBLUE() << "[Cerebro::descriptor_computer_thread] Looks like data_map is empty which means this is a fresh run" << TermColor::RESET() << endl;;
    }
    else {
        // This means, state was preloaded from file
        cout << TermColor::iGREEN() << "[Cerebro::descriptor_computer_thread] Looks like data_map is NOT empty which means state was loaded from disk" << TermColor::RESET() << endl;;

        // loop over data_map and make a note of all the timestamps where descriptor is available.
        int n_whlimgdescavai = 0;
        // img_data_mgr->print_status();
        for( auto itd=data_map->begin() ; itd!=data_map->end() ; itd++ )
        {
            if( itd->second->isWholeImageDescriptorAvailable() ) {
                n_whlimgdescavai++;
                wholeImageComputedList_pushback( itd->first );

                // Also make sure these images are rechable.
                bool kxxx = img_data_mgr->isImageRetrivable( "left_image", itd->first );
                if( kxxx == false ) // this is bad
                {
                    cout << "[Cerebro::descriptor_computer_thread] THIS IS BAD. REPORT IT TO AUTHORS WITH CODE jewbcjsmn\nEXIT.....\n";
                    exit(1);
                }
            }
        }
        cout << "[Cerebro::descriptor_computer_thread]i find "<< n_whlimgdescavai << " datanodes where image descrptor is available and images are retrivable through the ImageDataManager\n";
    }
    #endif


    ros::Time last_proc_timestamp =ros::Time();
    int n_computed=0;
    ElapsedTime _time;
    int incoming_diff_ms = 0;
    while( b_descriptor_computer_thread )
    {
        if( data_map->begin() == data_map->end() ) {
            __Cerebro__descriptor_computer_thread( cout << "nothing to compute descriptor\n" );
            std::this_thread::sleep_for( std::chrono::milliseconds( 1000 )  );
            continue;
        }
        ros::Time lb = data_map->rbegin()->first - ros::Duration(10, 0); // look at recent 10sec.
        auto S = data_map->lower_bound( lb );
        auto E = data_map->end();
        __Cerebro__descriptor_computer_thread(cout << "[Cerebro::descriptor_computer_thread]] S=" << S->first << "  E=" << (data_map->rbegin())->first << endl;)
        for( auto it = S; it != E ; it++ ) {

            // cout << "[Cerebro::descriptor_computer_thread]try : " << it->first << "\t";
            // cout << "isWholeImageDescriptorAvailable=" << it->second->isWholeImageDescriptorAvailable() << "\t";
            // cout << "isKeyFrame=" << it->second->isKeyFrame() << "\t";
            // cout << endl;

            //descriptor does not exisit at this stamp, so compute it.
            // Here I compute the whole image descriptor only at keyframes, you may try something like, if the pose is available compute it.
            if( it->second->isWholeImageDescriptorAvailable() == false && it->second->isKeyFrame() && it->first >  last_proc_timestamp )
            {
                  __Cerebro__descriptor_computer_thread(cout << TermColor::MAGENTA() << "[Cerebro::descriptor_computer_thread]--- process t=" << it->first << "\trel_t=" <<  it->first - dataManager->getPose0Stamp() << TermColor::RESET() << " n_computed=" << n_computed << endl;)
                  n_computed++;
                  incoming_diff_ms = (it->first - last_proc_timestamp).toSec() * 1000. ;
                  float skip_frac = 1.0 -  incoming_diff_ms/float(estimated_descriptor_compute_time_ms) ;
                  __Cerebro__descriptor_computer_thread(  cout << "incoming_diff_ms = " << incoming_diff_ms << "\testimated_descriptor_compute_time_ms=" << estimated_descriptor_compute_time_ms  << " so, skip_frac="<< skip_frac <<  endl; )
                  last_proc_timestamp = it->first;

                  //   if( n_computed%2 == 0 ) // simple skip
                  if( n_computed>4 && ( rand()/ float(RAND_MAX) ) < skip_frac ) // dynamic skip
                  {
                      __Cerebro__descriptor_computer_thread( cout << "\t\t...SKIP...\n"; )
                      continue;
                  }


                    if( it->second->getNumberOfSuccessfullyTrackedFeatures() < 20 )
                    {
                        __Cerebro__descriptor_computer_thread( cout << "[Cerebro::descriptor_computer_thread] skip computing whole-image-descriptor for this image because it appears to be a kidnapped image frame.\n" ; )
                        continue;
                    }

                    _time.tic();

                    // use it->second->getImage() to compute descriptor. call the service
                    cv::Mat image_curr;
                    #if 0 // set this to 1 to get image from data_map (deprecated way), 0 to get image from image_manager.
                    // old code in which images were stored in nodes
                    image_curr = it->second->getImage();
                    #else
                    // using image data manager
                    { //dont remove these braces, it is used for scoping and automatic deallocation
                        cv::Mat tmp_img;
                        bool getimstatus = img_data_mgr->getImage( "left_image", it->first, tmp_img );
                        if( getimstatus == false ) {
                            __Cerebro__descriptor_computer_thread( cout << "since I cannot getImage, goto hurr_here\n"; );
                            goto huur_here;
                        }

                        if( nChannels == 3 && tmp_img.channels() == 1 )
                            cv::cvtColor( tmp_img, image_curr, CV_GRAY2BGR );
                        else if( nChannels==1 && tmp_img.channels() == 3)
                            cv::cvtColor( tmp_img, image_curr, CV_BGR2GRAY );
                        else
                            image_curr = tmp_img;
                    }

                    __Cerebro__descriptor_computer_thread(
                    cout << "image from mgr info: " << MiscUtils::cvmat_info(image_curr) << endl;
                    cout << "nChannels=" << nChannels << endl;
                    )
                    #endif

                    sensor_msgs::ImagePtr image_msg;
                    if( nChannels == 3 ) {
                        image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_curr).toImageMsg();
                    }
                    else if( nChannels == 1 ) {
                        // cout << "cv_bridge mono8 encoding\n";
                        image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_curr).toImageMsg();
                    }
                    else {
                        assert( false && "Invalid number of channels specified in Cerebro::descriptor_computer_thread() ");
                        cout << TermColor::RED() << "[ERROR] Cx Invalid number of channels specified in Cerebro::descriptor_computer_thread() " << endl << TermColor::RESET();
                        exit(3);
                    }
                    image_msg->header.stamp = ros::Time( it->first );



                    cerebro::WholeImageDescriptorCompute srv; //service message
                    srv.request.ima = *image_msg;
                    srv.request.a = 986;
                    if( client.call( srv ) ) {
                        // __Cerebro__descriptor_computer_thread(std::cout <<  "Received response from server\t";)
                        // __Cerebro__descriptor_computer_thread(std::cout << "desc.size=" << srv.response.desc.size() << std::endl;)
                        assert( srv.response.desc.size() > 0 && "The received descriptor appear to be of zero length. This is a fatal error.\n" );

                        VectorXd vec( srv.response.desc.size() ); // allocated a vector
                        for( int j=0 ; j<(int)srv.response.desc.size() ; j++ ) {
                            vec(j) = srv.response.desc[j];
                        }
                        // std::this_thread::sleep_for(std::chrono::milliseconds(100));

                        it->second->setWholeImageDescriptor( vec );
                        wholeImageComputedList_pushback( it->first );

                        // __Cerebro__descriptor_computer_thread(cout << "Computed descriptor at t=" << it->first - dataManager->getPose0Stamp() << "\t" << it->first << endl;)
                        // __Cerebro__descriptor_computer_thread(std::cout << "Computed descriptor in (millisec) = " << _time.toc_milli()  << endl;)
                        // __Cerebro__descriptor_computer_thread__imp(cout << TermColor::CYAN() << "Computed descriptor at t=" << it->first - dataManager->getPose0Stamp() << "\t" << it->first << TermColor::RESET() << endl);

                         estimated_descriptor_compute_time_ms = _time.toc_milli();
                        __Cerebro__descriptor_computer_thread__imp(
                        cout << "Computed descriptor at t=" << it->first << " of dim=" << srv.response.desc.size()  << " in millisec=" << estimated_descriptor_compute_time_ms << endl;
                        )


                    }
                    else {
                        ROS_ERROR( "Failed to call ros service" );
                    }

            }
            huur_here:
            int ddddd=0;  //this is here for the goto statement.
        }

        rate.sleep();
    }


    cout << "[Cerebro::descriptor_computer_thread] Finished\n";

}



// these both functions are newly added. There are still some raw access
// limited only to the function ``descrip_N__dot__descrip_0_N()``.
const int Cerebro::wholeImageComputedList_size() const {
    std::lock_guard<std::mutex> lk(m_wholeImageComputedList);
    return wholeImageComputedList.size();
}

const ros::Time Cerebro::wholeImageComputedList_at(int k) const
{
    std::lock_guard<std::mutex> lk(m_wholeImageComputedList);
    assert( k>=0 && k<wholeImageComputedList.size() && "[Cerebro::wholeImageComputedList_at]");
    return wholeImageComputedList.at( k );
}

void Cerebro::wholeImageComputedList_pushback( const ros::Time __tx )
{
    std::lock_guard<std::mutex> lk(m_wholeImageComputedList);
    this->wholeImageComputedList.push_back( __tx ); // note down where it was computed.

}

//------------------------------------------------------------------//
// END per Keyframe Descriptor Computation
//------------------------------------------------------------------//




//------------------------------------------------------------------//
//---------------- Populate Loop Candidates --------------------//
//------------------------------------------------------------------//



// #define __Cerebro__run__( msg ) msg ;
#define __Cerebro__run__( msg ) ;

// #define __Cerebro__run__debug( msg ) msg ;
#define __Cerebro__run__debug( msg ) ;

/// TODO: In the future more intelligent schemes can be experimented with. Besure to run those in new threads and disable this thread.
/// wholeImageComputedList is a list for which descriptors are computed. Similarly other threads can compute
/// scene-object labels, text etc etc in addition to currently computed whole-image-descriptor
void Cerebro::run()
{
    // descrip_N__dot__descrip_0_N();
    // faiss__naive_loopcandidate_generator();
    // faiss_clique_loopcandidate_generator();
    faiss_multihypothesis_tracking();

}


#ifdef HAVE_FAISS
///-----------------------------------------------------------------
/// Nearest neighbors search using facebook research's faiss library's IndexFlatIP
///         Roughly follows : https://github.com/mpkuse/vins_mono_debug_pkg/blob/master/src_place_recog/faiss_try1.py
/// This function is similar in functionality to `descrip_N__dot__descrip_0_N`
///        but using faiss
void Cerebro::faiss__naive_loopcandidate_generator()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    assert( b_run_thread && "you need to call run_thread_enable() before run() can start executing\n" );

    //-----------------//
    //---- Settings ---//
    //-----------------//
    const int start_adding_descriptors_to_index_after = 150;
    const int LOCALITY_THRESH = 12;
    const float DOT_PROD_THRESH = 0.9;

    //------ END -----//

    // wait until connected_to_descriptor_server=true and descriptor_size_available=true
    if( wait_until__connectedToDescServer_and_descSizeAvailable( 71 ) == false ) {
        cout << TermColor::RED() << "[Cerebro::faiss__naive_loopcandidate_generator ERROR] wait_until__connectedToDescServer_and_descSizeAvailable returned false implying a timeout.\n" << TermColor::RESET();
        return;
    }
    __Cerebro__run__( cout << TermColor::GREEN() <<"[Cerebro::faiss__naive_loopcandidate_generator] descriptor_size=" << this->descriptor_size << "  connected_to_descriptor_server && descriptor_size_available" << TermColor::RESET() << endl; )
    assert( this->descriptor_size> 0 );


    cout << TermColor::GREEN() << "init a faiss::IndexFlatIP("<< this->descriptor_size << ")" << TermColor::RESET() << endl;
    faiss::IndexFlatIP index(this->descriptor_size);


    ros::Rate rate(10);
    int l=0, last_l=0;
    int l_last_added_to_index = 0;

    auto data_map = dataManager->getDataMapRef();
    while( b_run_thread )
    {
        l=wholeImageComputedList_size();

        if( l - last_l < 3 ) {
            __Cerebro__run__debug( cout << "[Cerebro::faiss__naive_loopcandidate_generator]nothing new\n"; );
            rate.sleep();
            continue;
        }

        __Cerebro__run__( cout << TermColor::RED() << "---" << TermColor::RESET() << endl; )
        __Cerebro__run__( cout << "l=" << l << endl; )
        __Cerebro__run__debug( cout << "[Cerebro::faiss__naive_loopcandidate_generator] data_map.size() = " << data_map->size() << "\tper_image_descriptor_size=" << this->descriptor_size << endl; )


        // add
        __Cerebro__run__(cout << TermColor::YELLOW();)
        if( l> start_adding_descriptors_to_index_after ) {
            for( int j=l_last_added_to_index ; j<l-start_adding_descriptors_to_index_after ; j++ ) {
                __Cerebro__run__(
                cout << "index.add( " << j << ")" ;
                cout << "\t aka  index.add(" << wholeImageComputedList_at(j) << ")";
                cout << endl;)
                VectorXd X = data_map->at( wholeImageComputedList_at( j ) )->getWholeImageDescriptor();
                VectorXf X_float = X.cast<float>();
                float * X_raw = X_float.data();
                #if 0 //see if the type casting was correct.
                for( int g=0;g<5;g++ ) {
                    // cout << "(" << X(g) << "," << X_float(g) << ") ";
                    cout << "(" << X(g) << "," << X_raw[g] << ") ";
                }
                cout << endl;
                #endif
                index.add( 1, X_raw ); //-TODO
            }
            l_last_added_to_index = l-start_adding_descriptors_to_index_after;
        } else {
            __Cerebro__run__(cout << "not seen enough. so,, add nothing. Will start adding after "<< start_adding_descriptors_to_index_after<< " descriptors\n";)
        }
        __Cerebro__run__(cout << TermColor::RESET();)

        // search
        vector<float> tmp_;
        vector<int> tmp_i;
        for( int l_i=last_l ; l_i<l; l_i++ ) {
            __Cerebro__run__(
            cout << TermColor::MAGENTA();
            cout << "index.search(" << l_i << ")\t";
            cout << "index.search(" << wholeImageComputedList_at(l_i) << ")\t";
            cout << "index.ntotal=" << index.ntotal << "\t";
            // cout << endl;
            )

            if( index.ntotal < 5 )
                continue;

            VectorXd X = data_map->at( wholeImageComputedList_at( l_i ) )->getWholeImageDescriptor();
            VectorXf X_float = X.cast<float>();
            float * X_raw = X_float.data();
            float distances[5];
            faiss::Index::idx_t labels[5];
            ElapsedTime time_to_search = ElapsedTime();
            index.search( 1, X_raw, 5, distances, labels ); //TODO
            __Cerebro__run__(cout << "search done in (ms): " << time_to_search.toc_milli() << endl;)
            for( int g=0 ; g<5; g++ ) {
                __Cerebro__run__(
                // cout << g << " labels=" << labels[g] << " distances=" << distances[g] << endl;
                // cout << l_i << "<--("<< distances[g] << ")-->" << labels[g] << "\t";
                printf( "%4d<--(%4.2f)-->%d\t", l_i, distances[g], labels[g] );
                )
            }
            __Cerebro__run__(cout << endl;)
            __Cerebro__run__(cout << TermColor::RESET();)
            tmp_.push_back(  distances[0] );
            tmp_i.push_back( labels[0] );
        }

        int _n = tmp_.size();
        if( _n ==3 && tmp_[_n-1] > DOT_PROD_THRESH && abs(tmp_i[0] - tmp_i[1]) < LOCALITY_THRESH && abs(tmp_i[0] - tmp_i[2]) < LOCALITY_THRESH  )
        {
            __Cerebro__run__(
            cout << TermColor::RED() << "loop found" << l-1 << "<--->" << tmp_i[2] << TermColor::RESET();
            )

            {
            std::lock_guard<std::mutex> lk_foundloops(m_foundLoops);
            foundLoops.push_back( std::make_tuple( wholeImageComputedList_at(l-1), wholeImageComputedList_at(tmp_i[2]), tmp_[2] ) );
            }
        }

        last_l = l;
        rate.sleep();
    }
    cout << "[Cerebro::faiss__naive_loopcandidate_generator()] Finished\n";
}


// #define __faiss_clique_loopcandidate_generator__imp(msg) msg;
#define __faiss_clique_loopcandidate_generator__imp(msg) ;

// #define __faiss_clique_loopcandidate_generator__debug(msg) msg;
#define __faiss_clique_loopcandidate_generator__debug(msg) ;

// #define __faiss_clique_loopcandidate_generator__addtoindex(msg) msg;
#define __faiss_clique_loopcandidate_generator__addtoindex(msg) ;

#define __faiss_clique_loopcandidate_generator__search(msg) msg;
// #define __faiss_clique_loopcandidate_generator__search(msg) ;
void Cerebro::faiss_clique_loopcandidate_generator()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    assert( b_run_thread && "you need to call run_thread_enable() before run() can start executing\n" );
    //-----------------//
    //---- Settings ---//
    //-----------------//
    const int start_adding_descriptors_to_index_after = 150;
    const int K_NEAREST_NEIGHBOURS=5;
    const double DOT_PROD_THRESH = 0.85; //0.85
    const int LOCALITY = 7;//7
    const int reset_accumulation_every_n_frames = 4; //4

    // wait until connected_to_descriptor_server=true and descriptor_size_available=true
    if( wait_until__connectedToDescServer_and_descSizeAvailable( 71 ) == false ) {
        cout << TermColor::RED() << "[Cerebro::faiss__naive_loopcandidate_generator ERROR] wait_until__connectedToDescServer_and_descSizeAvailable returned false implying a timeout.\n" << TermColor::RESET();
        return;
    }
    __faiss_clique_loopcandidate_generator__imp( cout << TermColor::GREEN() <<"[Cerebro::faiss__naive_loopcandidate_generator] descriptor_size=" << this->descriptor_size << "  connected_to_descriptor_server && descriptor_size_available" << TermColor::RESET() << endl; )
    assert( this->descriptor_size> 0 );

    // init faiss index
    __faiss_clique_loopcandidate_generator__imp(
    cout << TermColor::GREEN() << "init a faiss::IndexFlatIP("<< this->descriptor_size << ")" << TermColor::RESET() << endl;
    )
    faiss::IndexFlatIP index(this->descriptor_size);

    ros::Rate rate(10);
    auto data_map = dataManager->getDataMapRef();

    int l=0, last_l=0, l_last_added_to_index=0;
    std::map< int, std::map< int, float> > data_strc;


    float * distances = new float[K_NEAREST_NEIGHBOURS];
    faiss::Index::idx_t * labels = new faiss::Index::idx_t[K_NEAREST_NEIGHBOURS];
    map<faiss::Index::idx_t, int> retained; //< Accumulation map. reset every say 3 indices. tune this as per amount of computation available

    while( b_run_thread ) {
        l = wholeImageComputedList_size();
        if( l<= last_l ) {
            __faiss_clique_loopcandidate_generator__debug( cout << "[Cerebro::faiss_clique_loopcandidate_generator] nothing new\n";)
            rate.sleep();
            continue;
        }

        __faiss_clique_loopcandidate_generator__imp(
        cout << TermColor::RED() << "--- " << TermColor::RESET() << l << "\t";
        cout << "new [" << last_l << " to " << l << ")\n";
        )


        //--------- add
        __faiss_clique_loopcandidate_generator__addtoindex(cout << TermColor::YELLOW();)
        if( l>start_adding_descriptors_to_index_after) {
            for( int j=l_last_added_to_index ; j<l-start_adding_descriptors_to_index_after ; j++ )
            {
                __faiss_clique_loopcandidate_generator__addtoindex(
                cout << "index.add( " << j << ")" ;
                cout << "\t aka  index.add(" << wholeImageComputedList_at(j) << ")\n";
                )

                VectorXd X = data_map->at( wholeImageComputedList_at( j ) )->getWholeImageDescriptor();
                VectorXf X_float = X.cast<float>();
                float * X_raw = X_float.data();
                #if 0 //see if the type casting was correct.
                for( int g=0;g<5;g++ ) {
                    // cout << "(" << X(g) << "," << X_float(g) << ") ";
                    cout << "(" << X(g) << "," << X_raw[g] << ") ";
                }
                cout << endl;
                #endif
                index.add( 1, X_raw );
            }
            l_last_added_to_index = l-start_adding_descriptors_to_index_after;
        } else {
            __faiss_clique_loopcandidate_generator__addtoindex(
            cout << "not seen enough. so,, add nothing. Will start adding after "<< start_adding_descriptors_to_index_after<< " descriptors\n";
            )
        }
        __faiss_clique_loopcandidate_generator__addtoindex(cout << TermColor::RESET();)



        //----------- search
        __faiss_clique_loopcandidate_generator__search(cout << TermColor::MAGENTA();)
        for( int l_i = last_l ; l_i < l ; l_i++ )
        {
            __faiss_clique_loopcandidate_generator__search(
            cout << "\t\tindex.search(" << l_i << ")\t";
            cout << "index.search(" << wholeImageComputedList_at(l_i) << ")\t";
            cout << "index.ntotal=" << index.ntotal << "\n";
            )
            if( index.ntotal < K_NEAREST_NEIGHBOURS )
                break;

            // l_i's descriptor
            VectorXd X = data_map->at( wholeImageComputedList_at( l_i ) )->getWholeImageDescriptor();
            VectorXf X_float = X.cast<float>();
            float * X_raw = X_float.data();

            // ElapsedTime time_to_search = ElapsedTime();
            index.search( 1, X_raw, K_NEAREST_NEIGHBOURS, distances, labels );
            // cout << "search done in (ms): " << time_to_search.toc_milli() << endl;

            __faiss_clique_loopcandidate_generator__search(
            // Printing
            for( int g=0 ; g<K_NEAREST_NEIGHBOURS; g++ ) {
                if( distances[g] > DOT_PROD_THRESH )
                    cout << TermColor::GREEN();
                else cout << TermColor::MAGENTA();
                printf( "%4d<--(%4.2f)-->%d\t", l_i, distances[g],(int)labels[g] );
                cout << TermColor::RESET();
            }
            cout << endl;
            )


            // Go thru each nearest neighbours and add to list separated ones.                                              vv same as 168, ignore
            // for example, if the neighbours: `802<--(0.92)-->606	 802<--(0.89)-->168	 802<--(0.89)-->172	 802<--(0.89)-->169	 802<--(0.88)-->607`
            //                                                 ^^                   ^^                  ^^same as 168, so ignore                ^^ same as 606, ignore

            // cout << "loop thru " << K_NEAREST_NEIGHBOURS << "\n";
            for( int g=0 ; g < K_NEAREST_NEIGHBOURS ; g++ )
            {
                if( distances[g] < DOT_PROD_THRESH ) //since distances are arranged in decending order, if you start seeing below my threshold its time to not process further.
                    break;

                // cout << "g="<<g << ", label="<< labels[g] << ", dis="<< distances[g] ;
                // cout << "\tloop thru retained. retained.size()=" << retained.size() << endl;
                faiss::Index::idx_t duplicate = -1;
                for( auto ity=retained.begin() ; ity!=retained.end() ; ity++ )
                {
                    // cout << "\tity->first=" << ity->first << endl;
                    if( (ity->first - labels[g]) < LOCALITY )
                    {
                        // cout << "\tcond satisfied\n";
                        duplicate = ity->first;
                        break;
                    }
                }
                if( duplicate != -1 ) {
                    // cout << "\tthis was not uniq so increment the key=" << duplicate << " by 1\n";
                    retained[ duplicate ]++;
                } else {
                    // cout << "\tadd new key at " << labels[g] << endl;
                    retained[ labels[g] ] = 1;
                }
            }


            if( retained.size() > 0 && l_i%reset_accumulation_every_n_frames == 0)
            {
                __faiss_clique_loopcandidate_generator__search(
                cout << TermColor::iYELLOW() << "l_i=" << l_i << " retained (ie. added to `foundLoops`) cout all: ";
                for( auto ity=retained.begin() ; ity!=retained.end() ; ity++ )
                {
                    cout << ity->first << ":" << ity->second << ", ";
                }
                cout << TermColor::RESET() << endl;
                )

                //%%%%
                //%%%% NOTE: If you put all the feasible candidates into the foundLoops it causes
                //%%%%       The pose computation to queue-up, since the number of candidates are too many.
                //%%%%       Usually addition of more than 2 candidates at a time spells trouble.
                //%%%%       I use simple heuristics when there are more than 3 items in the retained.


                if( retained.size() == 1 )
                {
                    __faiss_clique_loopcandidate_generator__search(
                    cout << "only 1 item in retained, pushback " << wholeImageComputedList_at(l-1) << "<--->" <<
                                                        wholeImageComputedList_at(retained.begin()->first) << endl;
                                                    )
                    std::lock_guard<std::mutex> lk_foundloops(m_foundLoops);
                    // __faiss_clique_loopcandidate_generator__search( cout << "pushback: " << wholeImageComputedList_at(l-1) << "<--->" << wholeImageComputedList_at(retained.begin()->first) << endl; )
                    foundLoops.push_back( std::make_tuple( wholeImageComputedList_at(l-1),
                                                        wholeImageComputedList_at(retained.begin()->first ),
                                                        0.9 ) );
                }

                if( retained.size() > 1 )
                {
                    int percent = 100. / retained.size(); //will retain this many
                    __faiss_clique_loopcandidate_generator__search( cout << "Retain %=" << percent << endl; )
                    for( auto ity=retained.begin() ; ity!=retained.end() ; ity++ )
                    {
                        if( rand()%100 < percent ) {
                            std::lock_guard<std::mutex> lk_foundloops(m_foundLoops);
                            __faiss_clique_loopcandidate_generator__search( cout << "pushback: " << wholeImageComputedList_at(l-1) << "<--->" << wholeImageComputedList_at(ity->first) << endl; )
                            foundLoops.push_back( std::make_tuple( wholeImageComputedList_at(l-1),
                                                            wholeImageComputedList_at(ity->first ),
                                                            0.9 ) );
                        }
                    }
                }


                retained.clear();
            }


        }


        last_l = l;
        rate.sleep();
    }


    delete [] distances;
    delete [] labels;
    cout << "[Cerebro::faiss_clique_loopcandidate_generator] Finished Thread\n";


}



// #define ___faiss_multihypothesis_tracking___loophead(msg) msg;
#define ___faiss_multihypothesis_tracking___loophead(msg) ;

// #define ___faiss_multihypothesis_tracking___add(msg) msg;
#define ___faiss_multihypothesis_tracking___add(msg) ;

// #define ___faiss_multihypothesis_tracking___search(msg) msg;
#define ___faiss_multihypothesis_tracking___search(msg) ;

void Cerebro::faiss_multihypothesis_tracking()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    assert( b_run_thread && "you need to call run_thread_enable() before run() can start executing\n" );
    //-----------------//
    //---- Settings ---//
    //-----------------//
    const int start_adding_descriptors_to_index_after = 150;
    const int K_NEAREST_NEIGHBOURS=5;

    #if 1
    const bool SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK = this->SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK; //set this to false to not write to disk
    const string SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR = this->SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR;
    #else
    const bool SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK = true; //set this to false to not write to disk
    const string SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR = "/app/tmp/cerebro/";
    #endif


    //-----------------//
    //----   Wait  ----//
    //-----------------//
    // wait until connected_to_descriptor_server=true and descriptor_size_available=true
    if( wait_until__connectedToDescServer_and_descSizeAvailable( 71 ) == false ) {
        cout << TermColor::RED() << "[Cerebro::faiss__naive_loopcandidate_generator ERROR] wait_until__connectedToDescServer_and_descSizeAvailable returned false implying a timeout.\n" << TermColor::RESET();
        return;
    }
    cout << TermColor::GREEN() <<"[Cerebro::faiss_multihypothesis_tracking] descriptor_size=" << this->descriptor_size << "  connected_to_descriptor_server && descriptor_size_available" << TermColor::RESET() << endl;
    assert( this->descriptor_size> 0 );

    //---------------------//
    //--- init faiss index //
    //---------------------//
    cout << TermColor::GREEN() << "[[Cerebro::faiss_multihypothesis_tracking]] init a faiss::IndexFlatIP("<< this->descriptor_size << ")" << TermColor::RESET() << endl;
    faiss::IndexFlatIP index(this->descriptor_size); // Eventually replace with some simpler (and portable) nearest-neighbour library


    // init (misc)
    ros::Rate rate(10);
    auto data_map = dataManager->getDataMapRef();

    hyp_manager = std::make_shared<HypothesisManager>();

    #if 0
    // Start monitoring thread
    hyp_manager->monitoring_thread_enable();
    // hyp_manager->monitoring_thread_disable();
    std::thread hyp_monitoring_th( &HypothesisManager::monitoring_thread, hyp_manager );
    #endif



    int l=0, last_l=0, l_last_added_to_index=0;


    float * distances = new float[K_NEAREST_NEIGHBOURS];
    faiss::Index::idx_t * labels = new faiss::Index::idx_t[K_NEAREST_NEIGHBOURS];

    //---------------------//
    // While
    //      index.add( i-150 )
    //      index.search( i )
    //---------------------//
    while( b_run_thread ) {
        l = wholeImageComputedList_size();
        if( l<= last_l ) {
            ___faiss_multihypothesis_tracking___loophead(
            cout << "[Cerebro::faiss_multihypothesis_tracking] nothing new\n"; )
            rate.sleep();
            continue;
        }

        // looks like new descriptors are available.
        ___faiss_multihypothesis_tracking___loophead(
        cout << TermColor::RED() << "--- " << TermColor::RESET() << l << "\t";
        cout << "new [" << last_l << " to " << l << ")\n";
        )



        //--------- add
        ___faiss_multihypothesis_tracking___add(cout << TermColor::YELLOW();)
        if( l>start_adding_descriptors_to_index_after) {
            for( int j=l_last_added_to_index ; j<l-start_adding_descriptors_to_index_after ; j++ )
            {
                ___faiss_multihypothesis_tracking___add(
                cout << "index.add( " << j << ")" ;
                cout << "\t aka  index.add(" << wholeImageComputedList_at(j) << ")\n";)


                VectorXd X = data_map->at( wholeImageComputedList_at( j ) )->getWholeImageDescriptor();
                VectorXf X_float = X.cast<float>();
                float * X_raw = X_float.data();
                #if 0 //see if the type casting was correct.
                for( int g=0;g<5;g++ ) {
                    // cout << "(" << X(g) << "," << X_float(g) << ") ";
                    cout << "(" << X(g) << "," << X_raw[g] << ") ";
                }
                cout << endl;
                #endif
                index.add( 1, X_raw );
            }
            l_last_added_to_index = l-start_adding_descriptors_to_index_after;
        } else {
            ___faiss_multihypothesis_tracking___add(
                cout << "not seen enough. so,, add nothing to the index. Will start adding after "<< start_adding_descriptors_to_index_after<< " descriptors\n";
            )

        }
        ___faiss_multihypothesis_tracking___add(cout << TermColor::RESET();)



        //----------- search

        for( int l_i = last_l ; l_i < l ; l_i++ )
        {
            ___faiss_multihypothesis_tracking___search(
            cout << TermColor::MAGENTA() << "\t\t";
            cout << "index.search(" << l_i << ")\t";
            cout << "index.search(" << wholeImageComputedList_at(l_i) << ")\t";
            cout << "index.ntotal=" << index.ntotal << "\n" << TermColor::RESET();
            )
            if( index.ntotal < K_NEAREST_NEIGHBOURS )
                break;

            // l_i's descriptor
            VectorXd X = data_map->at( wholeImageComputedList_at( l_i ) )->getWholeImageDescriptor();
            VectorXf X_float = X.cast<float>();
            float * X_raw = X_float.data();

            // ElapsedTime time_to_search = ElapsedTime();
            index.search( 1, X_raw, K_NEAREST_NEIGHBOURS, distances, labels );
            // cout << "search done in (ms): " << time_to_search.toc_milli() << endl;

            // Loop over all the nearest neighbours
            ___faiss_multihypothesis_tracking___search( cout << "\t\t"; )
            for( int g=0 ; g<K_NEAREST_NEIGHBOURS; g++ ) {


                ___faiss_multihypothesis_tracking___search(
                if( distances[g] > 0.85 )
                    cout << TermColor::GREEN();
                else cout << TermColor::MAGENTA();
                #if 1 // style-A
                // printf( "g=%2d: %4d<~(%4.2f)~>%4d; ", g, l_i, distances[g], labels[g] );
                #endif

                if( g==0 )
                    printf( "%4d <--- ", l_i );

                printf( "(%4d, %4.2f) ", labels[g], distances[g] );
                cout << TermColor::RESET();

                )

                // add the edge
                int is_new_hypothesis_added = hyp_manager->add_node( l_i, labels[g], distances[g], g );

                #if 1
                if( is_new_hypothesis_added > 0 && SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK ) {
                    ___faiss_multihypothesis_tracking___search(
                    cout << "number of new hypothesis = " << is_new_hypothesis_added << endl; )
                    // save for debugging,
                    for( int s=0 ; s<is_new_hypothesis_added ; s++ ) {
                        save_loop_hypothesis_representative_image_pair_to_disk( SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR, hyp_manager->n_hypothesis()-1-s );
                    }
                }
                #endif

            }
            ___faiss_multihypothesis_tracking___search( cout << endl ; )

        }


        last_l = l;
        rate.sleep();
    }

    #if 1
    // debug look inside hyp_manager, when ESC was pressed....this causes threading issues, better for do image related stuff here, printing is still ok
    cout << TermColor::RED() << "[Cerebro::faiss_multihypothesis_tracking] =====Print debug data for the hyp_manager just before I quit the thread\n" << TermColor::RESET();
    // hyp_manager->print_hyp_q_all();


    // make image pairs
    int seq_a_start, seq_a_end, seq_b_start, seq_b_end; //these index in `wholeImageComputedList`
    int idx_a_start, idx_a_end, idx_b_start, idx_b_end;
    ros::Time seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T;
    int n_hyp = hyp_manager->n_hypothesis();
    auto img_data_mgr = dataManager->getImageManagerRef();
    // auto data_map = dataManager->getDataMapRef();

    json all_loop_hyp;
    cout << "~~~~ Printing Info on All Loop Hypothesis, n_hyp=" << n_hyp << "~~~~~" << endl;

    for( int i=0 ; i<n_hyp; i++ )
    {
        // this code as exactly the same effect at the #else part.
        this->loop_hypothesis_i_idx( i,  seq_a_start, seq_a_end, seq_b_start, seq_b_end  );
        this->loop_hypothesis_i_T( i,   seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T );
        this->loop_hypothesis_i_datamap_idx( i, idx_a_start, idx_a_end, idx_b_start, idx_b_end  );



        if( hyp_manager->is_computed_pose_available(i) == true )
            cout << TermColor::GREEN();


        // print timestamps for ith hypothesis
        cout << "#" << i << " ";

        cout << "data_map_idx=(" << idx_a_start << "," << idx_a_end << ")";
        cout << "<----->";
        cout << "(" << idx_b_start << "," << idx_b_end << ")";

        if( hyp_manager->is_computed_pose_available(i) )
        {
            cout << "\n\ta_T_b = " << PoseManipUtils::prettyprintMatrix4d( hyp_manager->get_computed_pose(i) ) << " ";
        }

        // cout << "(" << seq_a_start_T << "," << seq_a_end_T << ")";
        // cout << "<----->";
        // cout << "(" << seq_b_start_T << "," << seq_b_end_T << ")";
        cout << "\n\t<comments>";
        cout << hyp_manager->get_debug_string( i, "\n\t" ) << endl;
        cout << "\t</comments>\n";

        cout << TermColor::RESET();
        cout << endl;


        #if 1
        json obj;
        obj["hypothesis_i"] = i;
        obj["idx_a_start"] = idx_a_start;
        obj["idx_a_end"] = idx_a_end;
        obj["idx_b_start"] = idx_b_start;
        obj["idx_b_end"] = idx_b_end;

        obj["idx_a_start_T"] = seq_a_start_T.toNSec();
        obj["idx_a_end_T"] = seq_a_end_T.toNSec();
        obj["idx_b_start_T"] = seq_b_start_T.toNSec();
        obj["idx_b_end_T"] = seq_b_end_T.toNSec();

        obj["is_relative_pose_available"] = hyp_manager->is_computed_pose_available(i);
        if( hyp_manager->is_computed_pose_available(i) )
        {
            Matrix4d opt_a0_T_b0 = hyp_manager->get_computed_pose(i);
            obj["opt_a0_T_b0"] = RawFileIO::write_eigen_matrix_tojson( opt_a0_T_b0 );
            obj["opt_a0_T_b0_prettyprint"] = PoseManipUtils::prettyprintMatrix4d( opt_a0_T_b0 );
        }

        obj["debug_str"] = hyp_manager->get_debug_string( i, "\n\t" );
        all_loop_hyp.push_back( obj );
        #endif


    }

    cout << "~~~~ ~~~~~" << endl;

    if( SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK ) {
        string fname_loop_hyp_json = SAVE_REPRESENTATIVE_IMAGE_PAIR_TO_DISK_SAVE_DIR + "/loop_hypothesis.json";
        cout << TermColor::YELLOW() << "Write JSON: " << fname_loop_hyp_json << endl;
        std::ofstream out_loop_hyp_file(fname_loop_hyp_json);
        out_loop_hyp_file << std::setw(4) << all_loop_hyp << std::endl;
    }

    #endif


    delete [] distances;
    delete [] labels;
    // delete hyp_manager;
    cout << "[Cerebro::faiss_multihypothesis_tracking] Finished Thread\n";


}




#endif //HAVE_FAISS

///-----------------------------------------------------------------

///-----------------------------------------------------------------
/// This implements a simple loopclosure detection scheme based on dot-product of descriptor-vectors.
///

// 1 will plot the result of dot product as image. 0 will not plot to image
// #define __Cerebro__descrip_N__dot__descrip_0_N__implotting 0
#define __Cerebro__descrip_N__dot__descrip_0_N__implotting 1


void Cerebro::descrip_N__dot__descrip_0_N()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    assert( b_run_thread && "you need to call run_thread_enable() before run() can start executing\n" );

    cout << TermColor::GREEN() << "[Cerebro::descrip_N__dot__descrip_0_N] Start thread" << TermColor::RESET() << endl;
    //---
    //--- Main settings for this function
    //---
    int LOCALITY_THRESH = 12;
    float DOT_PROD_THRESH = 0.85;
    const int start_adding_descriptors_to_index_after = 50;

    ros::Rate rate(10);

    #if 0
    // wait until connected_to_descriptor_server=true and descriptor_size_available=true
    int wait_itr = 0;
    while( true ) {
        if( this->connected_to_descriptor_server && this->descriptor_size_available)
            break;
        __Cerebro__run__(cout << wait_itr << " [Cerebro::descrip_N__dot__descrip_0_N]waiting for `descriptor_size_available` to be true\n";)
        rate.sleep();
        wait_itr++;
        if( wait_itr > 157 ) {
            __Cerebro__run__( cout << TermColor::RED() << "[Cerebro::descrip_N__dot__descrip_0_N] `this->connected_to_descriptor_server && this->descriptor_size_available` has not become true dispite waiting for about 15sec. So quiting the run thread.\n" << TermColor::RESET(); )
            return;
        }
    }
    #endif

    if( wait_until__connectedToDescServer_and_descSizeAvailable( 71 ) == false ) {
        cout << TermColor::RED() << "[ Cerebro::descrip_N__dot__descrip_0_N ERROR] wait_until__connectedToDescServer_and_descSizeAvailable returned false implying a timeout.\n" << TermColor::RESET();
        return;
    }

    __Cerebro__run__( cout << TermColor::GREEN() <<"[Cerebro::descrip_N__dot__descrip_0_N] descriptor_size=" << this->descriptor_size << "  connected_to_descriptor_server && descriptor_size_available" << TermColor::RESET() << endl; )
    assert( this->descriptor_size> 0 );



    int l=0, last_l=0;
    int last_processed=0;
    MatrixXd M = MatrixXd::Zero( this->descriptor_size, 29000 ); // TODO: Need dynamic allocation here.
    cout << "[Cerebro::descrip_N__dot__descrip_0_N] M.rows = " << M.rows() << "  M.cols=" << M.cols()  << endl;
    cout << "[Cerebro::descrip_N__dot__descrip_0_N] TODO: Need dynamic allocation here.\n";

    #if __Cerebro__descrip_N__dot__descrip_0_N__implotting > 0
    // Plotting image
    Plot2Mat handle(320,240, cv::Scalar(80,80,80) );
    // Plot2Mat handle;
    handle.setYminmax( -0.1, 1.1);
    #endif
    while( b_run_thread )
    {

        auto data_map = dataManager->getDataMapRef(); // this needs to be get every iteration else i dont get the ubdated values which are constantly being updated by other threads.
        l = wholeImageComputedList_size();

        if( l - last_l < 3 ) {
            __Cerebro__run__debug( cout << "[Cerebro::descrip_N__dot__descrip_0_N]nothing new\n"; );
            rate.sleep();
            continue;
        }

        __Cerebro__run__( cout << TermColor::RED() << "---" << TermColor::RESET() << endl; )
        __Cerebro__run__( cout << "l=" << l << endl; )
        __Cerebro__run__debug( cout << "[Cerebro::descrip_N__dot__descrip_0_N] data_map.size() = " << data_map->size() << "\tper_image_descriptor_size="<< this->descriptor_size << endl; )


        ///------------ Extract v, v-1, v-2. The latest 3 descriptors.
        VectorXd v, vm, vmm;
        ros::Time i_v, i_vm, i_vmm;
        {
            // std::lock_guard<std::mutex> lk(m_wholeImageComputedList);
            assert(  data_map->count( wholeImageComputedList_at(l-1) ) > 0  &&
                     data_map->count( wholeImageComputedList_at(l-2) ) > 0  &&
                     data_map->count( wholeImageComputedList_at(l-3) ) > 0 &&
                     "either of l, l-1, l-2 is not available in the datamap"
                 );




            v   = data_map->at( wholeImageComputedList_at(l-1) )->getWholeImageDescriptor();
            vm  = data_map->at( wholeImageComputedList_at(l-2) )->getWholeImageDescriptor();
            vmm = data_map->at( wholeImageComputedList_at(l-3) )->getWholeImageDescriptor();
            i_v = wholeImageComputedList_at(l-1);
            i_vm = wholeImageComputedList_at(l-2);
            i_vmm = wholeImageComputedList_at(l-3);
        }



        // This is very inefficient. Better to have a matrix-vector product and not getWholeImageDescriptor() all the time.
        assert( M.rows() == v.size() );
        assert( l < M.cols() );


        ////-------------- Fill descriptors [last_l, l) into M.
        {
            // std::lock_guard<std::mutex> lk(m_wholeImageComputedList); //no need for this as we switched to _at functions which are threadsafe
            for( int _s=last_l ; _s<l ; _s++ ) {
                M.col(_s) = data_map->at( wholeImageComputedList_at(_s) )->getWholeImageDescriptor();

                __Cerebro__run__debug(
                cout << "M.col(" << _s << ") = data_map[ " << wholeImageComputedList_at(_s) << " ]. \t";
                cout << " isWholeImageDescriptorAvailable = " << data_map->at( wholeImageComputedList_at(_s) )->isWholeImageDescriptorAvailable() << endl;
                )
            }
        }


        //////////////////////////////////////
        ////----------- DOT PRODUCT----------
        /////////////////////////////////////
        int k = l - start_adding_descriptors_to_index_after; // given a stamp, l, get another stamp k. better make this to 200.

        //usable size of M is 8192xl, let k (k<l) be the length until which dot is needed by time.
        if( k > 5 ) {
            ElapsedTime timer;
            timer.tic();
            // dot
            VectorXd u   = v.transpose() * M.leftCols( k );
            VectorXd um  = vm.transpose() * M.leftCols( k );
            VectorXd umm = vmm.transpose() * M.leftCols( k );
            __Cerebro__run__( cout << "<v=" << i_v  <<" , M[0 to "<< k << "]>  ";)
            __Cerebro__run__( cout << "<vm=" << i_vm  <<" , M[0 to "<< k << "]>  ";)
            __Cerebro__run__( cout << "<vmm=" << i_vmm  <<" , M[0 to "<< k << "]>     ";)
            __Cerebro__run__( cout << "Done in (ms): " << timer.toc_milli() << endl; )

            // max coefficient and the index of maxcoeff.
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


            #if __Cerebro__descrip_N__dot__descrip_0_N__implotting > 0
            // plot
            handle.resetCanvas();
            handle.plot( u );

            #endif


            // Criteria for a recognized place
            if( abs(u_argmax - um_argmax) < LOCALITY_THRESH && abs(u_argmax-umm_argmax) < LOCALITY_THRESH && u_max > DOT_PROD_THRESH  )
            {
                // std::lock_guard<std::mutex> lk(m_wholeImageComputedList); //no need of lock here as we switched to _at functions for whlist


                __Cerebro__run__(
                cout << TermColor::RED() << "Loop FOUND!! a_idx=" << l-1 << "<-----> (" <<   u_argmax << "," << um_argmax << "," << umm_argmax << ")"<< TermColor::RESET() << endl;
                cout << TermColor::RED() << "loop FOUND!! "
                                         <<  "t_l=" << wholeImageComputedList_at(l-1)
                                         << "  <DOT=" << u_max << ">  "
                                         << "t_argmax=" << wholeImageComputedList_at(u_argmax)
                                         << TermColor::RESET() << endl;
                                )

                  #if __Cerebro__descrip_N__dot__descrip_0_N__implotting > 0
                  handle.mark( u_argmax );
                  #endif

                  //TODO :
                  // publish the image pair based on a config_file_flag
                  // read flags for publish image, threshold(0.92), locality threshold (8) from file.

                {
                std::lock_guard<std::mutex> lk_foundloops(m_foundLoops);
                foundLoops.push_back( std::make_tuple( wholeImageComputedList_at(l-1), wholeImageComputedList_at(u_argmax), u_max ) );
                }
            }


            #if __Cerebro__descrip_N__dot__descrip_0_N__implotting > 0
            cv::imshow("canvas", handle.getCanvasConstPtr() );
            cv::waitKey(20);
            #endif

        }
        else {
            __Cerebro__run__(
            cout << "[Cerebro::descrip_N__dot__descrip_0_N] do nothing. not seen enough yet.\n";
            )
        }


        last_l = l;
        rate.sleep();
    }

    cout << TermColor::RED() << "[Cerebro::descrip_N__dot__descrip_0_N] Finished thread" << TermColor::RESET() << endl;
}



//------------------------------------------------------------------//
//---------------- END Populate Loop Candidates --------------------//
//------------------------------------------------------------------//


//------------------------------------------------------------------//
//----------  Functions to Query about Loop Candidates  ------------//
//------------------------------------------------------------------//
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
    auto data_map = dataManager->getDataMapRef();

    int n_foundloops = foundLoops.size();
    for( int i=0 ; i<n_foundloops ; i++ ) {
        auto u = foundLoops[ i ];
        ros::Time t_curr = std::get<0>(u);
        ros::Time t_prev = std::get<1>(u);
        double score = std::get<2>(u);


        assert( data_map->count( t_curr ) > 0 && data_map->count( t_prev ) > 0  && "One or both of the timestamps in foundloops where not in the data_map. This cannot be happening...fatal...\n" );
        int idx_1 = std::distance( data_map->begin(), data_map->find( t_curr )  );
        int idx_2 = std::distance( data_map->begin(), data_map->find( t_prev )  );
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


//------------------------------------------------------------------//
//----------  Functions to Query about Hypothesis Manager ----------//
//------------------------------------------------------------------//

const int Cerebro::loop_hypothesis_count() const
{
    return hyp_manager->n_hypothesis();
}

void Cerebro::loop_hypothesis_i_idx( int i, int& seq_a_start, int& seq_a_end, int& seq_b_start, int& seq_b_end ) const
{
    bool status = hyp_manager->hypothesis_i( i, seq_a_start, seq_a_end, seq_b_start, seq_b_end );
    assert( status );
}

void Cerebro::loop_hypothesis_i_T(   int i, ros::Time& seq_a_start_T, ros::Time& seq_a_end_T, ros::Time& seq_b_start_T, ros::Time& seq_b_end_T  ) const
{
    int seq_a_start, seq_a_end, seq_b_start, seq_b_end;
    bool status = hyp_manager->hypothesis_i( i, seq_a_start, seq_a_end, seq_b_start, seq_b_end );
    assert( status );
    if( status == false ) {
        cout << "[Cerebro::loop_hypothesis_i_T] hyp_manager->hypothesis_i failed...";
        exit(2);
    }
    seq_a_start_T = wholeImageComputedList_at( seq_a_start );
    seq_a_end_T   = wholeImageComputedList_at( seq_a_end );
    seq_b_start_T = wholeImageComputedList_at( seq_b_start );
    seq_b_end_T   = wholeImageComputedList_at( seq_b_end );
}

void Cerebro::loop_hypothesis_i_im(  int i, cv::Mat& seq_a_start_im , cv::Mat& seq_a_end_im , cv::Mat& seq_b_start_im , cv::Mat& seq_b_end_im ) const
{
    cout << "[Cerebro::loop_hypothesis_i_im] NOT IMPLEMTED....\n";
    exit(4);
}



void Cerebro::loop_hypothesis_i_datamap_idx( int i, int& datamap_seq_a_start, int& datamap_seq_a_end, int& datamap_seq_b_start, int& datamap_seq_b_end ) const
{
    ros::Time seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T;
    loop_hypothesis_i_T( i, seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T );

    auto data_map = dataManager->getDataMapRef();

    datamap_seq_a_start = std::distance( data_map->begin(), data_map->find( seq_a_start_T )  );
    datamap_seq_a_end   = std::distance( data_map->begin(), data_map->find( seq_a_end_T )  );
    datamap_seq_b_start = std::distance( data_map->begin(), data_map->find( seq_b_start_T )  );
    datamap_seq_b_end   = std::distance( data_map->begin(), data_map->find( seq_b_end_T )  );
    // buffer << "index in data_map #" << i << " ";
    // buffer << "(" << idx_a_start << "," << idx_a_end << ")";
    // buffer << "<----->";
    // buffer << "(" << idx_b_start << "," << idx_b_end << ");";
}


json Cerebro::loop_hypothesis_as_json() const
{
    cout << "[Cerebro::loop_hypothesis_as_json] NOT IMPLEMENTED....\n";
    exit(4);
}


void Cerebro::loop_hypothesis_i__set_computed_pose( int i,  Matrix4d a_T_b, string info_str )
{
    hyp_manager->set_computed_pose( i, a_T_b, info_str );
}

void Cerebro::loop_hypothesis_i__append_debug_string( int i,  string info_str )
{
    hyp_manager->append_debug_string( i, info_str );
}

//------------------------------------------------------------------//
//----------------- Geometry for Loop Hypothesis -------------------//
//------------------------------------------------------------------//

// #define __Cerebro__loop_hypothesis_consumer_thread( msg ) msg;
#define __Cerebro__loop_hypothesis_consumer_thread( msg ) ;
void Cerebro::loop_hypothesis_consumer_thread()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // wait for 50ms ,

    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    cout << TermColor::GREEN() <<   "[Cerebro::loop_hypothesis_consumer_thread] Start thread" << TermColor::RESET() << endl;
    // assert( b_loopcandidate_consumer && "you need to call loopcandidate_consumer_enable() before loopcandiate_consumer_thread() can start executing\n" );



    ros::Rate rate(1);
    ros::Rate rate_2( 15 );
    int prev_count = 0;
    while( b_loop_hypothesis_consumer )
    {
        rate.sleep();

        // if( hyp_manager == nullptr )
        if( is_loop_hypothesis_manager_allocated() == false )
        {
            cout << "[Cerebro::loop_hypothesis_consumer_thread] The shared_ptr, hyp_manager is not allocated yet...wait and do nothing until then\n";
            continue;
        }



        int curr_count = loop_hypothesis_count();
        if( curr_count > prev_count )
        {
            // something new
            __Cerebro__loop_hypothesis_consumer_thread(
            cout << "[Cerebro::loop_hypothesis_consumer_thread] ---loop_hypothesis_count=" << curr_count << endl; )
            for( int d=prev_count ; d<curr_count ; d++ )
            {
                __Cerebro__loop_hypothesis_consumer_thread(
                cout << TermColor::iCYAN() << "[Cerebro::loop_hypothesis_consumer_thread] Process Loop Hypothesis#" << d << TermColor::RESET() << endl; )
                dataManager->clean_up_pause();
                __Cerebro__loop_hypothesis_consumer_thread( cout << "[Cerebro::loop_hypothesis_consumer_thread]dataManager->clean_up_running_status()=" << dataManager->clean_up_running_status() << endl; )
                while( dataManager->clean_up_running_status() ) {
                    __Cerebro__loop_hypothesis_consumer_thread(
                    cout << "[Cerebro::loop_hypothesis_consumer_thread]waiting for cleanup thread to finish its current task" << endl;)
                    rate_2.sleep();
                }

                ElapsedTime geom_t("Bundled Pose computation hypothesis#"+to_string(d));
                // compute_geometry_for_loop_hypothesis_i( d ); //< LocalBundle
                compute_geometry_for_loop_hypothesis_i_lite( d ); //< Just uses 1 image pair (start_a, start_b)
                __Cerebro__loop_hypothesis_consumer_thread(
                cout << TermColor::uGREEN() << geom_t.toc() << TermColor::RESET() << endl; )


                dataManager->clean_up_play();
                __Cerebro__loop_hypothesis_consumer_thread(
                cout << TermColor::BLUE() << "[Cerebro::loop_hypothesis_consumer_thread] it took ms=" << geom_t.toc_milli() << " to process this loop hypothesis\n"; )
            }
        } else {
            __Cerebro__loop_hypothesis_consumer_thread(
            cout << "[Cerebro::loop_hypothesis_consumer_thread] nothing new, curr_count (of number of loop hyp)=" << curr_count << "\n"; )
            ;
        }

        prev_count = curr_count;
    }

    cout << TermColor::RED() <<  "[Cerebro::loop_hypothesis_consumer_thread] disable called, quitting loop_hypothesis_consumer_thread" << TermColor::RESET() << endl;

}





// This will take the index of hypothesis and compute the relative pose.
// This method used only the firsts of both sequences ie. seq_a_start --- seq_b_start
// Following is done:
//      a. GMS Matches (either at 0.5 resolution or 1.0 resolution )
//      b. if insufficient number of matches (declare it as false positive and end)
//      c.  if sufficient feature matches compute 3d points at these matches
//      d. pose compute by alternating minimization (global methods needs no initial guess)
//      e. refinement with edge-alignment.
bool Cerebro::compute_geometry_for_loop_hypothesis_i_lite( int i )
{
    cout << TermColor::bCYAN() << TermColor::TAB2() << "[Cerebro::compute_geometry_for_loop_hypothesis_i_lite] Process Loop Hypothesis#" << i << TermColor::RESET() << endl;
    ElapsedTime t_full_function_compuation_time( "Full Fuction");

    //----params
    const int N_TOO_FEW_POINT_MATCHES = 30;
    const int N_TOO_FEW_VALID_DEPTH_CORRESPONDENCES = 20;

    bool SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES = this->SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES;
    const string SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX = this->SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX;
    std::stringstream buffer;
    // buffer.clear();
    buffer.str(std::string());


    //----book keeping
    auto img_data_mgr = dataManager->getImageManagerRef();
    auto data_map = dataManager->getDataMapRef();


    ros::Time seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T;
    loop_hypothesis_i_T( i, seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T );

    int idx_a_start, idx_a_end, idx_b_start, idx_b_end;
    loop_hypothesis_i_idx( i, idx_a_start, idx_a_end, idx_b_start, idx_b_end );


    cout << TermColor::TAB2() << "timestamps: " ;
    cout << seq_a_start_T << "," << seq_a_end_T << "(duration=" << seq_a_end_T-seq_a_start_T << ")";
    cout << "<---->";
    cout << seq_b_start_T << "," << seq_b_end_T << "(duration=" << seq_b_end_T-seq_b_start_T << ")";
    cout << endl;
    cout << TermColor::TAB2() << "dataManager idx: " ;
    cout << idx_a_start << "," << idx_a_end ;
    cout << "<---->";
    cout << idx_b_start << "," << idx_b_end ;
    cout << endl;
    // if this is negative means `b` is previous `a` is next.
    cout << TermColor::TAB2() << "seq_b_start_T - seq_a_start_T = " << (seq_b_start_T - seq_a_start_T).toSec() << endl;

    //---- retrive necessary image, pose data
    int idx_a = idx_a_start;
    int idx_b = idx_b_start;
    ros::Time t_a = seq_a_start_T;
    ros::Time t_b = seq_b_start_T;
    cv::Mat left_image_a, depth_a;
    cv::Mat left_image_b, depth_b;
    Matrix4d w_T_a, w_T_b;
    bool status_a = retrive_image_data( t_a, left_image_a, depth_a, w_T_a );
    bool status_b = retrive_image_data( t_b, left_image_b, depth_b, w_T_b );
    if( status_a == false || status_b == false ) {
        cout << TermColor__LF << "[Cerebro::compute_geometry_for_loop_hypothesis_i_lite] retrive_image_data failed, so skip this pair\n" ;
        loop_hypothesis_i__append_debug_string(i, "retrive_image_data failed, so skip this pair" );
        return false;
    }

    //---- GMS Matches
    ElapsedTime t_im_correspondence;
    t_im_correspondence.tic("Image correspondences");
    MatrixXd uv_a, uv_b;
    StaticPointFeatureMatching::gms_point_feature_matches( left_image_a, left_image_b, uv_a, uv_b );
    // StaticPointFeatureMatching::gms_point_feature_matches_scaled( left_image_a, left_image_b, uv_a, uv_b, 0.5 );
    // StaticPointFeatureMatching::gms_point_feature_matches_scaled( left_image_a, left_image_b, uv_a, uv_b, 0.25 );

    auto im_correspondence_elapsed_time_ms = t_im_correspondence.toc_milli();
    cout << TermColor::TAB3() << TermColor::YELLOW() << "#matches=" << uv_a.cols() << TermColor::RESET() << "\t";
    cout << t_im_correspondence.toc() << "\t";
    cout << endl;

    // buffer.clear();
    buffer.str(std::string());
    buffer << "#matches=" << uv_a.cols() << "\t" << t_im_correspondence.toc();
    loop_hypothesis_i__append_debug_string(i, buffer.str()  );

    if( uv_a.cols() < N_TOO_FEW_POINT_MATCHES ) {
        cout << TermColor::TAB3() << TermColor::bYELLOW();
        cout << "number of matches (" << uv_a.cols() <<  ") is less than the threshold ("<< N_TOO_FEW_POINT_MATCHES << ")\n";
        cout << TermColor::RESET();

        // buffer.clear();
        buffer.str(std::string());
        buffer << "number of matches (" << uv_a.cols() <<  ") is less than the threshold ("<< N_TOO_FEW_POINT_MATCHES << ")";
        loop_hypothesis_i__append_debug_string(i, buffer.str()  );
        return false;
    }


    //---- Camera related geometry (normalized image cords, depth, 3dpts )
    ElapsedTime t_im_geometry("Geometry and Depth");
    VectorXd d_a = StaticPointFeatureMatching::depth_at_image_coordinates( uv_a, depth_a );
    VectorXd d_b = StaticPointFeatureMatching::depth_at_image_coordinates( uv_b, depth_b );

    //--- normalized image cords
    MatrixXd normed_uv_a = StaticPointFeatureMatching::image_coordinates_to_normalized_image_coordinates( dataManager->getAbstractCameraRef(), uv_a );
    MatrixXd normed_uv_b = StaticPointFeatureMatching::image_coordinates_to_normalized_image_coordinates( dataManager->getAbstractCameraRef(), uv_b );

    // 3d points
    MatrixXd aX = StaticPointFeatureMatching::normalized_image_coordinates_and_depth_to_3dpoints( normed_uv_a, d_a, true );
    MatrixXd bX = StaticPointFeatureMatching::normalized_image_coordinates_and_depth_to_3dpoints( normed_uv_b, d_b, true );


    // filter 3d points based on depth values
    double near = 0.5, far = 5.0;
    vector<bool> valids_a = MiscUtils::filter_near_far( d_a, near, far );
    vector<bool> valids_b = MiscUtils::filter_near_far( d_b, near, far );
    int nvalids_a = MiscUtils::total_true( valids_a );
    int nvalids_b = MiscUtils::total_true( valids_b );
    vector<bool> valids = MiscUtils::vector_of_bool_AND( valids_a, valids_b );
    int nvalids = MiscUtils::total_true( valids );

    cout <<  TermColor::TAB3();
    cout << TermColor::YELLOW() << "nvalids_a,nvalids_b,nvalids=(" << nvalids_a << "," << nvalids_b << "," << nvalids << ")\t" << TermColor::RESET();
    cout << t_im_geometry.toc() << "\t";
    cout << "far="<<far << ", near=" << near << "\t";
    cout << endl;
    #if 0
    cout << "uv_a,uv_b: " << uv_a.rows() << "x" << uv_a.cols() << "," << uv_b.rows() << "x" << uv_b.cols() << "\t";
    cout << "normed_uv_a,normed_uv_a: " << normed_uv_a.rows() << "x" << normed_uv_a.cols() << "," << normed_uv_b.rows() << "x" << normed_uv_b.cols() << "\t";
    cout << "aX,bX: " << aX.rows() << "x" << aX.cols() << "," << bX.rows() << "x" << bX.cols() << "\t";
    cout << endl;
    #endif


    // buffer.clear();
    buffer.str(std::string());
    buffer << "nvalids_a,nvalids_b,nvalids=(" << nvalids_a << "," << nvalids_b << "," << nvalids << ")\t";
    buffer << t_im_geometry.toc() << "\t";
    buffer << "far="<<far << ", near=" << near ;
    loop_hypothesis_i__append_debug_string(i, buffer.str()  );



    if( SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES )
    {
        cv::Mat dst_matcher;
        string msg_str = "plot (resize 0.5), took ms="+to_string(im_correspondence_elapsed_time_ms);
        msg_str += ";#valid depths="+to_string( nvalids);
        msg_str += ";#nvalids_a,nvalids_b,nvalids=("+ std::to_string( nvalids_a) + "," + std::to_string(nvalids_b) + "," + std::to_string(nvalids) + ")";
        MiscUtils::plot_point_pair( left_image_a, uv_a, idx_a,
                                    left_image_b, uv_b, idx_b,
                                    dst_matcher,
                                    #if 1 // make this to 1 to mark matches by spatial color codes (gms style). set this to 0 to mark the matches with lines
                                    3, msg_str
                                    #else
                                    cv::Scalar( 0,0,255 ), cv::Scalar( 0,255,0 ), false, msg_str
                                    #endif
                                );
        cv::resize(dst_matcher, dst_matcher, cv::Size(), 0.5, 0.5);

        string fname = SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX+"/hyp_" + to_string(i) + "_imagepair" + ".jpg";
        cout << TermColor::iWHITE() << "\t\t\timwriteFTGH(" << fname << ");\n" << TermColor::RESET();
        cv::imwrite( fname, dst_matcher );
    }

    if( nvalids < N_TOO_FEW_VALID_DEPTH_CORRESPONDENCES ) {
        cout << TermColor::TAB3() << TermColor::bYELLOW();
        cout << "#correspondences which have good depth (" << nvalids <<  ") is less than the threshold ("<< N_TOO_FEW_VALID_DEPTH_CORRESPONDENCES << "),so I cannot compute any pose info\n";
        cout << TermColor::RESET();

        // buffer.clear();
        buffer.str(std::string());
        buffer << "#correspondences which have good depth (" << nvalids <<  ") is less than the threshold ("<< N_TOO_FEW_VALID_DEPTH_CORRESPONDENCES << "),so I cannot compute any pose info";
        loop_hypothesis_i__append_debug_string(i, buffer.str()  );

        return false;
    }


    //---- Pose with 3d-3d alignment with alternating minimization (to generate a rough initial guess)
    cout << TermColor::bWHITE();
    cout << TermColor::TAB3() << "-----------------------------------------\n";
    cout << TermColor::TAB3() << "-----   ALTERNATING MINIMIZATIONS -------\n";
    cout << TermColor::TAB3() << "-----------------------------------------\n";
    cout << TermColor::RESET();
    Matrix4d a_T_b = Matrix4d::Identity();
    VectorXd switch_weights = VectorXd::Constant( aX.cols() , 1.0 );
    float minimization_metric = PoseComputation::alternatingMinimization( aX, bX, a_T_b, switch_weights );
    if( minimization_metric <  0 )
    {
        cout << TermColor::TAB3() << TermColor::bYELLOW();
        cout << "Alternating minimization return nagative: " << minimization_metric << " return false"<< endl;
        cout << TermColor::RESET() ;

        // buffer.clear();
        buffer.str(std::string());
        buffer << "Alternating minimization return nagative, indicating non convergence";
        loop_hypothesis_i__append_debug_string(i, buffer.str()  );
        return false;
    }

    cout << TermColor::TAB3();
    cout << "RESULT of AM: a_T_b: " << PoseManipUtils::prettyprintMatrix4d( a_T_b ) << endl;
    cout << TermColor::bWHITE() << TermColor::TAB3() << "----- DONE ALTERNATING MINIMIZATIONS -------\n" << TermColor::RESET();

    loop_hypothesis_i__append_debug_string(i, "RESULT of AM: a_T_b: " + PoseManipUtils::prettyprintMatrix4d( a_T_b )  );




    //---- Refinement
    //         option-A: minimization of reprojection error
    //         option-B: edge-alignment
    #if 0
    cout << TermColor::bWHITE();
    cout << TermColor::TAB3() << "------------------------------------------------------------------\n";
    cout << TermColor::TAB3() << "----- POSE REFINEMENT : Minimization of Reprojection Error -------\n";
    cout << TermColor::TAB3() << "------------------------------------------------------------------\n";
    cout << TermColor::RESET();

    //TODO may be do EPNP https://github.com/cvlab-epfl/EPnP/blob/master/cpp/main.cpp

    cout <<  TermColor::bWHITE() << TermColor::TAB3() << "----- DONE POSE REFINEMENT : Minimization of Reprojection Error -------\n" << TermColor::RESET();

    #endif //reprojection error

    #if 1
    cout << TermColor::bWHITE();
    cout << TermColor::TAB3() << "----------------------------------------------\n";
    cout << TermColor::TAB3() << "----- POSE REFINEMENT : Edge Alignment -------\n";
    cout << TermColor::TAB3() << "----------------------------------------------\n";
    cout << TermColor::RESET();

    {
        cv::Mat im_ref = left_image_a;
        cv::Mat im_curr = left_image_b;
        cv::Mat depth_curr = depth_b;
        auto cam = dataManager->getAbstractCameraRef();

        Matrix4d initial_guess____ref_T_curr = a_T_b; //if using b as ref, than change needed accordingly

        EdgeAlignment ealign( cam, im_ref, im_curr, depth_curr );
        if( SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES )
            ealign.set_make_representation_image();
        Matrix4d ref_T_curr_optvar; //ie. a_T_b

        ElapsedTime t_main_ea( "ealign.solve()");
        bool ea_status = ealign.solve( initial_guess____ref_T_curr, ref_T_curr_optvar );
        a_T_b = ref_T_curr_optvar;

        cout << TermColor::TAB3() << "ea_status=" << ea_status << "\t" ;
            cout << TermColor::uGREEN() << t_main_ea.toc() << TermColor::RESET() << endl;
        cout << TermColor::TAB3() << "initial_guess____ref_T_curr = " << PoseManipUtils::prettyprintMatrix4d( initial_guess____ref_T_curr ) << endl;
        cout << TermColor::TAB3() << "ref_T_curr_optvar           = " << PoseManipUtils::prettyprintMatrix4d( ref_T_curr_optvar ) << endl;
        cout << TermColor::TAB3() << ealign.getCeresSummary().BriefReport() << endl;


        loop_hypothesis_i__append_debug_string(i, "ea_status="+to_string(ea_status)+"\t"+t_main_ea.toc() );
        loop_hypothesis_i__append_debug_string(i, "initial_guess____ref_T_curr = " + PoseManipUtils::prettyprintMatrix4d( initial_guess____ref_T_curr )  );
        loop_hypothesis_i__append_debug_string(i, "ref_T_curr_optvar           = " + PoseManipUtils::prettyprintMatrix4d( ref_T_curr_optvar )   );
        loop_hypothesis_i__append_debug_string(i, ealign.getCeresSummary().BriefReport() );


        //   view debug image
        // cv::imshow( "debug_image_ealign", ealign.get_representation_image() );

        if( SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES )
        {
            string fname = SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX+"/hyp_" + to_string(i) + "_ea" + ".jpg";
            cv::imwrite( fname, ealign.get_representation_image() );

            // this will cause the inputs of edge align to be written to disk for further analysis.
            ealign.save_to_disk( SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX+"/hyp_" + to_string(i) + "_ea" , initial_guess____ref_T_curr);
        }
    }

    cout << TermColor::bWHITE() << TermColor::TAB3() << "----- DONE POSE REFINEMENT : Edge Alignment -------\n" << TermColor::RESET();

    #endif // edge alignment



    //---- Publish the pose
    // TODO make message and publish
    // publish_pose_from_seq( t_a, idx_a, seq_a_odom_pose,
    //                        t_b, idx_b, seq_b_odom_pose,
    //                         a_T_b );

    cout << TermColor::uWHITE() <<  "loop_hypothesis_i__set_computed_pose" <<TermColor::RESET() <<  endl;
    loop_hypothesis_i__set_computed_pose( i, a_T_b, "successful" );
    loop_hypothesis_i__append_debug_string(i, "successfully computed pose. "+ t_full_function_compuation_time.toc() );



    return false;

}

#define __Cerebro__compute_geometry_for_loop_hypothesis_i( msg ) msg;
// #define __Cerebro__compute_geometry_for_loop_hypothesis_i( msg );

// This will print info on each of the randomly drawn image pairs
// #define __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs( msg ) msg;
#define __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs( msg );
bool Cerebro::compute_geometry_for_loop_hypothesis_i( int i )
{
    __Cerebro__compute_geometry_for_loop_hypothesis_i(
    cout << TermColor::bCYAN() << "\t\t[Cerebro::compute_geometry_for_loop_hypothesis_i] Process Loop Hypothesis#" << i << TermColor::RESET() << endl;)

    //--- params
    const int N_RANDOM_PAIRS = 6;
    const bool PLOT_IMAGE_PAIR = false; // Plots the raindomly picked image pairs
    const string PLOT_IMAGE_PAIR__SAVE_DIR = "/app/tmp/cerebro/live_system/";

    int N_TOO_FEW_POINT_MATCHES = 30; // use 50 for GMS matches, 7 for ORB


    //
    #if 1
    bool SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES = this->SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES;
    const string SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX = this->SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX;
    #else
    bool SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES = true; // plots the observed keypoint matches and reprojections
    const string SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX = "/app/tmp/cerebro/reprojections/";
    #endif


    auto img_data_mgr = dataManager->getImageManagerRef();
    auto data_map = dataManager->getDataMapRef();




    //--- get the idx and w_T_c of the sequence
    ros::Time seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T;
    loop_hypothesis_i_T( i, seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T );

    vector<int> seq_a_idx, seq_b_idx; // these are the idx in data_map
    vector<Matrix4d> seq_a_odom_pose, seq_b_odom_pose;
    vector<Matrix4d> seq_odom_a0_T_a, seq_odom_b0_T_b;
    vector<ros::Time> seq_a_T, seq_b_T;
    retrive_full_sequence_info( seq_a_start_T, seq_a_end_T, seq_a_T, seq_a_idx, seq_a_odom_pose, seq_odom_a0_T_a );
    retrive_full_sequence_info( seq_b_start_T, seq_b_end_T, seq_b_T, seq_b_idx, seq_b_odom_pose, seq_odom_b0_T_b );



    __Cerebro__compute_geometry_for_loop_hypothesis_i(
    cout << "\t\tidx data_map: " << *(seq_a_idx.begin()) << "," << *(seq_a_idx.rbegin()) << "<---->" << *(seq_b_idx.begin()) << "," << *(seq_b_idx.rbegin()) << endl;
    cout << "\t\ttimestamps  : " << *(seq_a_T.begin()) << "," << *(seq_a_T.rbegin()) << "<---->" << *(seq_b_T.begin()) << "," << *(seq_b_T.rbegin()) << endl;
    )



    //--- Randomly draw image pair in the range
    std::default_random_engine generator;
    std::uniform_int_distribution<int> rand_a(0, (int)seq_a_idx.size() );
    std::uniform_int_distribution<int> rand_b(0, (int)seq_b_idx.size() );

    // collect all these for every image-pair
    vector<MatrixXd> all_a0_X, all_b0_X;
    vector<MatrixXd> all_uv_a, all_uv_b, all_normed_uv_a, all_normed_uv_b;
    vector< vector<bool> > all_valids;
    vector< Matrix4d > all_poses_a0_T_a; // not really needed, if you have the timestamps or idx of the pairs
    vector< Matrix4d > all_poses_b0_T_b;
    vector< VectorXd > all_d_a, all_d_b;

    vector< std::pair<int,int> > all_pair_idx;

    ElapsedTime t_draw_random_pairs( "Draw Random Image-pairs");
    __Cerebro__compute_geometry_for_loop_hypothesis_i(
    cout << "\t\tDraw random pairs N_RANDOM_PAIRS=" << N_RANDOM_PAIRS << endl;
    )
    int count_zero_feature_matches = 0;
    bool flag_false_positive = false;
    for( int itr=0 ; itr<N_RANDOM_PAIRS ; itr++ )
    {
        //--- random number
        int ra = rand_a(generator);
        int rb = rand_b(generator);
        ros::Time t_a = seq_a_T.at(ra);
        ros::Time t_b = seq_b_T.at(rb);

        __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs(
        cout << TermColor::iWHITE() << "\t\t\titr=" << itr << " of " << N_RANDOM_PAIRS << "\t" << TermColor::RESET();
        // cout << ra << "<-->" << rb << TermColor::RESET() << endl;
        // cout << "\t\t\t";
        cout << "ra=" << ra << "<--->rb=" << rb << "\t";
        cout << "seq_a_idx=" << seq_a_idx.at(ra) << "<--->seq_b_idx=" << seq_b_idx.at(rb) << "\t";
        cout << "t_a=" << t_a << "<--->t_b=" << t_b << endl;
        )


        //--- retrive data
        cv::Mat left_image_a, left_image_b;
        cv::Mat depth_a, depth_b;
        Matrix4d w_T_a, w_T_b;
        #if 1 // if depth image available
        bool status_a = retrive_image_data( t_a, left_image_a, depth_a, w_T_a );
        bool status_b = retrive_image_data( t_b, left_image_b, depth_b, w_T_b );

        if( status_a == false || status_b == false ) {
            cout << "[Cerebro::compute_geometry_for_loop_hypothesis_i] retrive_image_data failed, so skip this pair\n" ;
            continue;
        }
        assert( status_a && status_b );
        #endif

        #if 0
        // stereo with sgbm
        #endif

        #if 0
        // monocular, idea is that we need depth at the keypoints
        #endif



        //--- image correspondences
        MatrixXd uv_a, uv_b;

        ElapsedTime elp;
        elp.tic("Image correspondences");

        #if 1 //set this to 1 to use the GMS matcher for point feat correspondence (slow), set this to 0 to use simple orb based matcher (fast)
        __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs( cout << "\t\t\t\timage correspondence: GMS_scaled_0.25";
                    // cout << MiscUtils::cvmat_info( left_image_a) << "\t" << MiscUtils::cvmat_info(left_image_b );
                    cout << "\n"; )

        // StaticPointFeatureMatching::gms_point_feature_matches( left_image_a, left_image_b, uv_a, uv_b );
        StaticPointFeatureMatching::gms_point_feature_matches_scaled( left_image_a, left_image_b, uv_a, uv_b, 0.25 );
        #else
        __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs(
        cout << "\t\t\t\timage correspondence: Simple (ORB)";)

        StaticPointFeatureMatching::point_feature_matches( left_image_a, left_image_b, uv_a, uv_b ); //cv::findFundamentalMat strangely fails :()
        #endif

        __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs(
        cout << TermColor::YELLOW();
        cout << "\t\t\t\t" << elp.toc() << "\t#matches=" << uv_a.cols() << endl;
        cout << TermColor::RESET();
        )

        auto im_correspondence_elapsed_time_ms = elp.toc_milli();



        // Too few point correspondence?
        if( uv_a.cols() < N_TOO_FEW_POINT_MATCHES )
            count_zero_feature_matches++;

        // if in first 3 iterations, i see no matches, this means the hypothesis is a false-positive
        if( count_zero_feature_matches > 2 && itr == 3 ) {
            __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs(
            cout << TermColor::iCYAN() <<  "\t\t\tThis hypothesis looks like a false-positive. In 3 consecutive draws I get 2 draws with 0 feature correspondences\n" << TermColor::RESET(); )
            flag_false_positive = true;
            break;
        }

        if( uv_a.cols() < N_TOO_FEW_POINT_MATCHES ) {
            __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs(
            cout << TermColor::CYAN() << "\t\t\t\tuv_a.cols() is less than N_TOO_FEW_POINT_MATCHES="<< N_TOO_FEW_POINT_MATCHES << " (#matches=" << uv_a.cols() << "), don't proceed to get 3d points at these correspondences\n" << TermColor::RESET();
            )


            __Cerebro__compute_geometry_for_loop_hypothesis_i(
            cout << "\t\t\t";
            cout << "hyp#" << i << " itr#" << itr << "\t";
            cout << "ra=" << ra << " <--> rb=" << rb << "\t";
            cout << TermColor::YELLOW() << "feat_matches(low res)=" << uv_a.cols()  << TermColor::RESET();
            cout << " less than N_TOO_FEW_POINT_MATCHES="<< N_TOO_FEW_POINT_MATCHES << " so skip this pair";
            cout << endl;
            )
            continue;
        }
        // END Too few point correspondence?
        int n_gms_matches = uv_a.cols();
        __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs(
        cout << "\t\t\t\tgms matches on full resolution\n"; )
        ElapsedTime t_fullres( "GMS Matcher Full resolution");
        MatrixXd gms_uv_a, gms_uv_b;
        StaticPointFeatureMatching::gms_point_feature_matches( left_image_a, left_image_b, gms_uv_a, gms_uv_b ); //cv::findFundamentalMat strangely fails :()
        StaticPointFeatureMatching::refine_and_sparsify_matches( left_image_a, left_image_b, gms_uv_a, gms_uv_b, uv_a, uv_b  )
        __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs( cout << t_fullres.toc(); )
        auto im_correspondence_elapsed_time_fullres = t_fullres.toc_milli();



        //--- depths from depth_image
        VectorXd d_a = StaticPointFeatureMatching::depth_at_image_coordinates( uv_a, depth_a );
        VectorXd d_b = StaticPointFeatureMatching::depth_at_image_coordinates( uv_b, depth_b );

        //--- normalized image cords
        MatrixXd normed_uv_a = StaticPointFeatureMatching::image_coordinates_to_normalized_image_coordinates( dataManager->getAbstractCameraRef(), uv_a );
        MatrixXd normed_uv_b = StaticPointFeatureMatching::image_coordinates_to_normalized_image_coordinates( dataManager->getAbstractCameraRef(), uv_b );

        // 3d points
        MatrixXd aX = StaticPointFeatureMatching::normalized_image_coordinates_and_depth_to_3dpoints( normed_uv_a, d_a, true );
        MatrixXd bX = StaticPointFeatureMatching::normalized_image_coordinates_and_depth_to_3dpoints( normed_uv_b, d_b, true );

        // valids
        double near = 0.5, far = 5.0;
        vector<bool> valids_a = MiscUtils::filter_near_far( d_a, near, far );
        vector<bool> valids_b = MiscUtils::filter_near_far( d_b, near, far );
        int nvalids_a = MiscUtils::total_true( valids_a );
        int nvalids_b = MiscUtils::total_true( valids_b );
        vector<bool> valids = MiscUtils::vector_of_bool_AND( valids_a, valids_b );
        int nvalids = MiscUtils::total_true( valids );


        #if 0
        cout << "\t\t\t\t";
        cout << "uv_a,uv_b: " << uv_a.rows() << "x" << uv_a.cols() << "," << uv_b.rows() << "x" << uv_b.cols() << "\t";
        cout << "normed_uv_a,normed_uv_a: " << normed_uv_a.rows() << "x" << normed_uv_a.cols() << "," << normed_uv_b.rows() << "x" << normed_uv_b.cols() << "\t";
        cout << "aX,bX: " << aX.rows() << "x" << aX.cols() << "," << bX.rows() << "x" << bX.cols() << "\t";
        cout << endl;
        #endif

        // TODO PLOT_IMAGE_PAIR
        #if 1
        if( PLOT_IMAGE_PAIR )
        {
            cv::Mat dst_matcher;
            string msg_str = "plot (resize 0.5), took ms="+to_string(im_correspondence_elapsed_time_ms);
            msg_str += ";#valid depths="+to_string( nvalids);
            MiscUtils::plot_point_pair( left_image_a, uv_a, seq_a_idx.at(ra),
                                        left_image_b, uv_b, seq_b_idx.at(rb), dst_matcher,
                                        #if 0 // make this to 1 to mark matches by spatial color codes (gms style). set this to 0 to mark the matches with lines
                                        3, msg_str
                                        #else
                                        cv::Scalar( 0,0,255 ), cv::Scalar( 0,255,0 ), false, msg_str
                                        #endif
                                    );
            cv::resize(dst_matcher, dst_matcher, cv::Size(), 0.5, 0.5);

            string fname = PLOT_IMAGE_PAIR__SAVE_DIR+"/hyp_" + to_string(i) + "__itr="+to_string(itr) + ".jpg";
            cout << "\t\t\timwriteFTGH(" << fname << ");\n";

            cv::imwrite( fname, dst_matcher );

        }

            __Cerebro__compute_geometry_for_loop_hypothesis_i(
            cout << "\t\t\t";
            cout << "hyp#" << i << " itr#" << itr << "\t";
            cout << "ra=" << ra << " <--> rb=" << rb << "\t";
            cout << TermColor::YELLOW() << "feat_matches=" << uv_a.cols() << "\t" << TermColor::RESET();
            // cout << "nvalids_depths=" <<nvalids << "\t" ;
            cout << "valids(a,b,AND)=" << nvalids_a << ","<< nvalids_b << "," << nvalids << "\t";
            cout << "elapsed_ms(lowres,full)=" << im_correspondence_elapsed_time_ms << "," << im_correspondence_elapsed_time_fullres<< "\t";
            cout << TermColor::YELLOW() << "gms_lowres=" << n_gms_matches << "\t" << TermColor::RESET();
            cout << endl;
            )

        #endif


        // ---- Accumulate
        // -- Matched pts
        __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs( cout << "\t\t\tAccumulate matched pts\n"; )
        all_uv_a.push_back( uv_a );
        all_uv_b.push_back( uv_b );
        all_normed_uv_a.push_back(normed_uv_a);
        all_normed_uv_b.push_back(normed_uv_b);
        all_valids.push_back( valids );

        // -- Poses
        __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs( cout << "\t\t\tAccumulate poses\n"; )
        Matrix4d a0_T_a = seq_a_odom_pose[0].inverse() * seq_a_odom_pose[ra];
        Matrix4d b0_T_b = seq_b_odom_pose[0].inverse() * seq_b_odom_pose[rb];
        all_poses_a0_T_a.push_back( a0_T_a );
        all_poses_b0_T_b.push_back( b0_T_b );

        // -- 3d points
        __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs( cout << "\t\t\tAccumulate 3d points\n"; )
        MatrixXd a0_X = a0_T_a * aX;
        MatrixXd b0_X = b0_T_b * bX;
        all_a0_X.push_back( a0_X );
        all_b0_X.push_back( b0_X );

        all_d_a.push_back( d_a );
        all_d_b.push_back( d_b );

        __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs( cout << "\t\t\tAccumulate seq_A_idx and seq_B_idx\n"; )
        all_pair_idx.push_back( std::make_pair( seq_a_idx.at(ra), seq_b_idx.at(rb) ) );

        __Cerebro__compute_geometry_for_loop_hypothesis_i_randompairs( cout << "\t\t\tDone itr=" << itr << endl; )


    } // END     for( int itr=0 ; itr<N_RANDOM_PAIRS ; itr++ )

    __Cerebro__compute_geometry_for_loop_hypothesis_i(
    cout << "\t\t" << t_draw_random_pairs.toc() << endl; )
    if( flag_false_positive) {
        __Cerebro__compute_geometry_for_loop_hypothesis_i(
        cout << TermColor::YELLOW() << "\t\tThis was marked as false positive, return false\n" << TermColor::RESET() ; )
        return false ;
    }

    // gather data
    cout << "\t\t\tgather from n_pairs=" << all_a0_X.size() << "\n";
    if( all_a0_X.size() < 3 )
    {
        __Cerebro__compute_geometry_for_loop_hypothesis_i(
        cout << "\t\t\ttoo few valid pairs, need atleast 3 valid pairs, ie. 3 pairs which each contain atleast " << N_TOO_FEW_POINT_MATCHES << " valid points\n"; )
        return false;
    }

    MatrixXd dst0, dst1;
    MiscUtils::gather( all_a0_X, all_valids, dst0 );
    MiscUtils::gather( all_b0_X, all_valids, dst1 );

    __Cerebro__compute_geometry_for_loop_hypothesis_i(
    cout << "\t\t";
    cout << "gathered --> dst0: " << dst0.rows() << "x" << dst0.cols() << "\t";
    cout << "dst1: " << dst1.rows() << "x" << dst1.cols() << endl; )
    if( dst0.cols() < 50 ) {
        __Cerebro__compute_geometry_for_loop_hypothesis_i(
        cout  << "\t\t\tgathered only " << dst0.cols() << ", too few points, skip this hypothesis\n"; )
        return false;
    }



    // >>>>>>>> Closed form pose computation
    Matrix4d a_T_b = Matrix4d::Identity();
    VectorXd switch_weights = VectorXd::Constant( dst0.cols() , 1.0 );
    float minimization_metric = PoseComputation::alternatingMinimization( dst0, dst1, a_T_b, switch_weights );
    if( minimization_metric <  0 )
    {
        cout << "\t\tAlternating minimization return nagative: " << minimization_metric << " return false"<< endl;
        return false;
    }

    #define _BUNDLE_REPROJECTION_ERROR_ 0
    #if _BUNDLE_REPROJECTION_ERROR_
    cout << TermColor::bWHITE();
    cout << "-----------------------------------------\n";
    cout << "----- _BUNDLE_REPROJECTION_ERROR_ -------\n";
    cout << "-----------------------------------------\n";
    cout << TermColor::RESET();

    // bundle refinement
    LocalBundle bundle;
    bundle.inputOdometry( 0, seq_odom_a0_T_a );
    bundle.inputOdometry( 1, seq_odom_b0_T_b );
    bundle.inputInitialGuess( 0, 1, a_T_b );
    bundle.inputFeatureMatches( 0, 1, all_normed_uv_a, all_normed_uv_b );
    vector<VectorXd> all_sf; //not used, just place holder
    bundle.inputFeatureMatchesDepths( 0, 1, all_d_a, all_d_b, all_sf );
    bundle.inputFeatureMatchesPoses( 0, 1, all_poses_a0_T_a, all_poses_b0_T_b );
    bundle.inputFeatureMatchesImIdx( 0, 1, all_pair_idx );
    bundle.inputOdometryImIdx( 0, seq_a_idx );
    bundle.inputOdometryImIdx( 1, seq_b_idx );
    // bundle.print_inputs_info();
    // bundle.toJSON("/app/catkin_ws/src/gmm_pointcloud_align/resources/local_bundle/");
    bundle.solve();
    a_T_b = bundle.retrive_optimized_pose( 0, 0, 1, 0 );
    bundle.reprojection_error( dataManager->getAbstractCameraRef() );

    //debug....reprojection images
    if( SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES ) {
        vector<cv::Mat> seq_a_image_list, seq_b_image_list;
        retrive_full_sequence_image_info( seq_a_start_T, seq_a_end_T, seq_a_image_list );
        retrive_full_sequence_image_info( seq_b_start_T, seq_b_end_T, seq_b_image_list );

        if( seq_odom_a0_T_a.size() != seq_a_image_list.size() || seq_odom_b0_T_b.size() != seq_b_image_list.size() ) {
            cout << "[Cerebro::compute_geometry_for_loop_hypothesis_i] In debugging reprojection errors, error_code=hydlnk\n";
            exit(4);
        }
        bundle.inputSequenceImages( 0, seq_a_image_list );
        bundle.inputSequenceImages( 1, seq_b_image_list );

        //bundle.reprojection_debug_images_to_disk( dataManager->getAbstractCameraRef(), "/app/tmp/cerebro/reprojections/hyp_"+to_string(i)+"_" );
        bundle.reprojection_debug_images_to_disk( dataManager->getAbstractCameraRef(), SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX+"/hyp_"+to_string(i)+"_" );


        // if you wish you can also save `LocalBundle bundle` to disk,
        // something like:
        // const string BUNDLE_WRITE_PATH = "/app/catkin_ws/src/gmm_pointcloud_align/resources/local_bundle/";
        // dataManager->getAbstractCameraRef()->writeParametersToYamlFile( BUNDLE_WRITE_PATH+"/camera.yaml" )
        // bundle.toJSON(BUNDLE_WRITE_PATH);

    }
    #endif //#if _BUNDLE_REPROJECTION_ERROR_



    #define _BUNDLE_EDGE_ALIGN_ERROR_ 1
    #if _BUNDLE_EDGE_ALIGN_ERROR_
    cout << TermColor::bWHITE();
    cout << "---------------------------------------\n";
    cout << "----- _BUNDLE_EDGE_ALIGN_ERROR_ -------\n";
    cout << "---------------------------------------\n";
    cout << TermColor::RESET();


    LocalBundle bundle;
    bundle.inputOdometry( 0, seq_odom_a0_T_a );
    bundle.inputOdometry( 1, seq_odom_b0_T_b );
    bundle.inputInitialGuess( 0, 1, a_T_b );
    bundle.inputFeatureMatches( 0, 1, all_normed_uv_a, all_normed_uv_b );
    vector<VectorXd> all_sf; //not used, just place holder
    bundle.inputFeatureMatchesDepths( 0, 1, all_d_a, all_d_b, all_sf );
    bundle.inputFeatureMatchesPoses( 0, 1, all_poses_a0_T_a, all_poses_b0_T_b );
    bundle.inputFeatureMatchesImIdx( 0, 1, all_pair_idx );
    bundle.inputOdometryImIdx( 0, seq_a_idx );
    bundle.inputOdometryImIdx( 1, seq_b_idx );

    vector<cv::Mat> seq_a_image_list, seq_a_depthimage_list;
    vector<cv::Mat> seq_b_image_list, seq_b_depthimage_list;
    retrive_full_sequence_image_info( seq_a_start_T, seq_a_end_T, seq_a_image_list );
    // current implementation requires depth only at cuttent frame so, so no need to put depth images at ref frames
    // retrive_full_sequence_depthimage_info( seq_a_start_T, seq_a_end_T, seq_a_depthimage_list );
    retrive_full_sequence_image_info( seq_b_start_T, seq_b_end_T, seq_b_image_list );
    retrive_full_sequence_depthimage_info( seq_b_start_T, seq_b_end_T, seq_b_depthimage_list );

    #if 0
    cout << "seq_a_image_list.size()=" << seq_a_image_list.size() << "\t";
    cout << "seq_a_depthimage_list.size()=" << seq_a_depthimage_list.size() << "\t";
    cout << "seq_b_image_list.size()=" << seq_b_image_list.size() << "\t";
    cout << "seq_b_depthimage_list.size()=" << seq_b_depthimage_list.size() << "\t";
    cout << endl;
    #endif

    bundle.inputSequenceImages( 0, seq_a_image_list );
    // bundle.inputSequenceDepthMaps( 0, seq_a_depthimage_list );
    bundle.inputSequenceImages( 1, seq_b_image_list );
    bundle.inputSequenceDepthMaps( 1, seq_b_depthimage_list );

    if( seq_odom_a0_T_a.size() != seq_a_image_list.size() || seq_odom_b0_T_b.size() != seq_b_image_list.size() ) {
        cout << "[Cerebro::compute_geometry_for_loop_hypothesis_i] In debugging reprojection errors, error_code=hydlnk\n";
        exit(4);
    }

    bool ea_make_debug_images = true; //SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES;
    ElapsedTime t_solve_ea( "bundle.solve_ea");
    bundle.SECRET_PREFIX = SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX+"/hyp_"+to_string(i)+"_";
    bundle.solve_ea(dataManager->getAbstractCameraRef(), ea_make_debug_images);
    cout << "[Cerebro::compute_geometry_for_loop_hypothesis_i]" << t_solve_ea.toc() << endl;

    a_T_b = bundle.retrive_optimized_pose( 0, 0, 1, 0 );

    // if( SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES )
    {
        bundle.reprojection_error( dataManager->getAbstractCameraRef() );
        bundle.edgealignment_debug_images_to_disk( SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX+"/hyp_"+to_string(i)+"_" );
        bundle.debug_print_initial_and_final_poses( SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX+"/hyp_"+to_string(i)+"_" );
        bundle.reprojection_debug_images_to_disk( dataManager->getAbstractCameraRef(), SAVE_LOCALBUNDLE_REPROJECTION_DEBUG_IMAGES_PREFIX+"/hyp_"+to_string(i)+"_" );

    }


    #endif //#if _BUNDLE_EDGE_ALIGN_ERROR_

    // publish
    #if 0 //make this to 1 to get pose info from the bundle object, 0 to set the pose
    Matrix4d opt_a0_T_b0 = a_T_b;
    publish_pose_from_seq( seq_a_T, seq_a_idx, seq_a_odom_pose,
                           seq_b_T, seq_b_idx, seq_b_odom_pose,
                            opt_a0_T_b0 );
    #else
    publish_pose_from_seq( seq_a_T, seq_a_idx, seq_a_odom_pose,
                           seq_b_T, seq_b_idx, seq_b_odom_pose,
                            bundle );
    #endif


    // simply note the pose info in HypothesisManager.
    loop_hypothesis_i__set_computed_pose( i, a_T_b, "successful" );

    return false;

}


bool Cerebro::retrive_full_sequence_info(
    const ros::Time seq_start_T, const ros::Time seq_end_T,
    vector<ros::Time>& seq_T, vector<int>& seq_idx,
    vector<Matrix4d>& seq_odom_pose, vector<Matrix4d>& seq_odom_x0_T_x
)
{
    seq_idx.clear();
    seq_odom_pose.clear();
    seq_T.clear();
    seq_odom_x0_T_x.clear();
    auto data_map = dataManager->getDataMapRef();

    bool is___w_T_x0 = false;
    Matrix4d w_T_x0;

    auto it_seq_start = data_map->find( seq_start_T );
    auto it_seq_end   = data_map->find( seq_end_T );
    // cout << "\t\tIn SeqA: " ;
    for( auto it =  it_seq_start ; it != it_seq_end ; it++ )
    {
        if( it->second->isKeyFrame() == false )
            continue;

        assert( it->second->isPoseAvailable() );

        int data_map_idx = std::distance( data_map->begin(), it );
        Matrix4d xpose = it->second->getPose();
        ros::Time stamp = it->first;

        if( is___w_T_x0 == false ) {
            is___w_T_x0 = true;
            w_T_x0 = xpose;
        }

        seq_idx.push_back( data_map_idx );
        seq_odom_pose.push_back( xpose );
        seq_odom_x0_T_x.push_back( w_T_x0.inverse() * xpose );
        seq_T.push_back( stamp );
        // cout << data_map_idx << "\t";
    }
    // cout << endl;
    return true;

}



bool Cerebro::retrive_full_sequence_image_info(
    const ros::Time seq_start_T, const ros::Time seq_end_T,
    vector<cv::Mat>& seq_image_list
)
{
    // seq_idx.clear();
    // seq_odom_pose.clear();
    // seq_T.clear();
    // seq_odom_x0_T_x.clear();
    seq_image_list.clear();
    auto data_map = dataManager->getDataMapRef();

    // bool is___w_T_x0 = false;
    // Matrix4d w_T_x0;

    auto it_seq_start = data_map->find( seq_start_T );
    auto it_seq_end   = data_map->find( seq_end_T );
    // cout << "\t\tIn SeqA: " ;
    for( auto it =  it_seq_start ; it != it_seq_end ; it++ )
    {
        if( it->second->isKeyFrame() == false )
            continue;

        assert( it->second->isPoseAvailable() );

        int data_map_idx = std::distance( data_map->begin(), it );
        // Matrix4d xpose = it->second->getPose();
        ros::Time stamp = it->first;

        cv::Mat tmp_image;
        bool status = retrive_image_data( stamp, tmp_image );
        if( status == false ) {
            cout << "\n~~~~~[Cerebro::retrive_full_sequence_image_info] cannot retrive image....I was expecting this function only to be used for debugging not in production. This retrives all the images in a sequences. use only for short seq\n";
            exit(1);
        }
        seq_image_list.push_back( tmp_image );
        // if( is___w_T_x0 == false ) {
            // is___w_T_x0 = true;
            // w_T_x0 = xpose;
        // }

        // seq_idx.push_back( data_map_idx );
        // seq_odom_pose.push_back( xpose );
        // seq_odom_x0_T_x.push_back( w_T_x0.inverse() * xpose );
        // seq_T.push_back( stamp );
        // cout << data_map_idx << "\t";
    }
    // cout << endl;
    return true;

}



bool Cerebro::retrive_full_sequence_depthimage_info(
    const ros::Time seq_start_T, const ros::Time seq_end_T,
    vector<cv::Mat>& seq_depthimage_list
)
{
    // seq_idx.clear();
    // seq_odom_pose.clear();
    // seq_T.clear();
    // seq_odom_x0_T_x.clear();
    seq_depthimage_list.clear();
    auto data_map = dataManager->getDataMapRef();

    // bool is___w_T_x0 = false;
    // Matrix4d w_T_x0;

    auto it_seq_start = data_map->find( seq_start_T );
    auto it_seq_end   = data_map->find( seq_end_T );
    // cout << "\t\tIn SeqA: " ;
    for( auto it =  it_seq_start ; it != it_seq_end ; it++ )
    {
        if( it->second->isKeyFrame() == false )
            continue;

        assert( it->second->isPoseAvailable() );

        int data_map_idx = std::distance( data_map->begin(), it );
        // Matrix4d xpose = it->second->getPose();
        ros::Time stamp = it->first;

        cv::Mat tmp_depth_image;
        bool status = retrive_depthimage_data( stamp, tmp_depth_image );
        if( status == false ) {
            cout << "\n~~~~~[Cerebro::retrive_full_sequence_image_info] cannot retrive image....I was expecting this function only to be used for debugging not in production. This retrives all the images in a sequences. use only for short seq\n";
            exit(1);
        }
        seq_depthimage_list.push_back( tmp_depth_image );
        // if( is___w_T_x0 == false ) {
            // is___w_T_x0 = true;
            // w_T_x0 = xpose;
        // }

        // seq_idx.push_back( data_map_idx );
        // seq_odom_pose.push_back( xpose );
        // seq_odom_x0_T_x.push_back( w_T_x0.inverse() * xpose );
        // seq_T.push_back( stamp );
        // cout << data_map_idx << "\t";
    }
    // cout << endl;
    return true;

}


bool Cerebro::make_loop_hypothesis_representative_image_pair( int i, cv::Mat& dst )
{
    int seq_a_start, seq_a_end, seq_b_start, seq_b_end; //these index in `wholeImageComputedList`
    int idx_a_start, idx_a_end, idx_b_start, idx_b_end;
    ros::Time seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T;
    int n_hyp = hyp_manager->n_hypothesis();

    auto img_data_mgr = dataManager->getImageManagerRef();

    if( i<0 || i>= n_hyp ) {
        cout << "[Cerebro::make_loop_hypothesis_representative_image_pair] FATAL error, you requested image for hyp#" << i << " but n_hyp=" << n_hyp << endl;
        exit(3);
    }



    this->loop_hypothesis_i_idx( i,  seq_a_start, seq_a_end, seq_b_start, seq_b_end  );
    this->loop_hypothesis_i_T( i,   seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T );
    this->loop_hypothesis_i_datamap_idx( i, idx_a_start, idx_a_end, idx_b_start, idx_b_end  );


    // make images for ith hypothesis
    cv::Mat seq_a_start_im, seq_a_end_im, seq_b_start_im, seq_b_end_im;
    bool status = true;
    {
    if( img_data_mgr->isImageRetrivable( "left_image", seq_a_start_T ) )
        img_data_mgr->getImage( "left_image", seq_a_start_T, seq_a_start_im );
    else
        status = false;

    if( img_data_mgr->isImageRetrivable( "left_image", seq_a_end_T ) )
        img_data_mgr->getImage( "left_image", seq_a_end_T, seq_a_end_im );
    else
        status = false;

    if( img_data_mgr->isImageRetrivable( "left_image", seq_b_start_T ) )
        img_data_mgr->getImage( "left_image", seq_b_start_T, seq_b_start_im );
    else
        status = false;

    if( img_data_mgr->isImageRetrivable( "left_image", seq_b_end_T ) )
        img_data_mgr->getImage( "left_image", seq_b_end_T, seq_b_end_im );
    else
        status = false;
    }
    assert( status == true );

    if( status == false ) {cout << "Something is wrong, one or more images cannot be retrieved"; return false; }
    else
    {
        // cv::Mat dst;
        MiscUtils::side_by_side( seq_a_start_im, seq_b_start_im , dst );

        std::stringstream buffer;
        buffer << ";Hypothesis#" << i ;
        buffer << ";this: " << idx_a_start << "(ie. " << seq_a_start_T << ")";
        buffer << "  ... ";
        buffer << idx_b_start << "(ie. " << seq_b_start_T << ");";

        buffer << "timestamps #" << i << " ";
        buffer << "(" << seq_a_start_T << "," << seq_a_end_T << ")";
        buffer << "<----->";
        buffer << "(" << seq_b_start_T << "," << seq_b_end_T << ");";
        buffer << "index in wholeImageComputedList #" << i << " ";
        buffer << "(" << seq_a_start << "," << seq_a_end << ")";
        buffer << "<----->";
        buffer << "(" << seq_b_start << "," << seq_b_end << ");";

        // int idx_a_start = std::distance( data_map->begin(), data_map->find( seq_a_start_T )  );
        // int idx_a_end   = std::distance( data_map->begin(), data_map->find( seq_a_end_T )  );
        // int idx_b_start = std::distance( data_map->begin(), data_map->find( seq_b_start_T )  );
        // int idx_b_end   = std::distance( data_map->begin(), data_map->find( seq_b_end_T )  );
        buffer << "index in data_map #" << i << " ";
        buffer << "(" << idx_a_start << "," << idx_a_end << ")";
        buffer << "<----->";
        buffer << "(" << idx_b_start << "," << idx_b_end << ");";



        MiscUtils::append_status_image( dst , buffer.str() );

    }

    return true;
}


bool Cerebro::save_loop_hypothesis_representative_image_pair_to_disk( const string SAVE_DIR, int i )
{
    // const string SAVE_DIR = "/app/tmp/cerebro/";
    string fname = SAVE_DIR + "/loophyp_" + to_string(i) + ".jpg";

    cv::Mat dst;
    bool status = make_loop_hypothesis_representative_image_pair( i, dst );
    if( status == false )  {
        cout << "Cerebro::save_loop_hypothesis_representative_image_pair_to_disk return false\n";
        return false;
    }

    cout << "imwrite( " << fname << ")\n";
    cv::imwrite( fname, dst );
    return true;
}

// #define __Cerebro__retrive_image_data__( msg ) msg;
#define __Cerebro__retrive_image_data__( msg ) ;
bool Cerebro::retrive_image_data( ros::Time& stamp, cv::Mat& left_image, cv::Mat& depth_image, Matrix4d& w_T_c )
{
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_image_data] stamp=" << stamp << endl; )

    auto img_data_mgr = dataManager->getImageManagerRef();
    auto data_map = dataManager->getDataMapRef();

    if( data_map->count( stamp ) == 0  )
    {
        cout << "[Cerebro::retrive_image_data] FATAL-ERROR. timestamps="<< stamp << " not found in data_map. This is not possible. If this happens, this is definately a bug, report to the authors\n";
        exit(1);
    }

    const DataNode * node = data_map->at( stamp );
    bool is_pose = node->isPoseAvailable();
    if( is_pose == false )
    {
        cout << "[Cerebro::retrive_image_data] WARN cannot retrive pose\n";
        return false;
    }

    __Cerebro__retrive_image_data__( cout << "[Cerebro::retrive_image_data]pose is available\n" );


    #if 0
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_image_data] isImageRetrivable?\n"; )
    bool is_left_image = img_data_mgr->isImageRetrivable( "left_image", node->getT() );
    bool is_depth = img_data_mgr->isImageRetrivable( "depth_image", node->getT() );
    bool is_pose = node->isPoseAvailable();

    if( is_left_image == false || is_depth == false || is_pose == false )
    {
        cout << "[Cerebro::retrive_image_data] WARN cannot retrive data at stamp=" << stamp << "\t";
        cout << "is_left_image=" << is_left_image << "  is_depth=" << is_depth << "  is_pose=" << is_pose << endl;
        return false;
    }
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_image_data] isImageRetrivable ...YES\n"; )
    #endif


    #if 0
    cv::Mat tmp_left_image, tmp_depth_image;
    img_data_mgr->getImage( "left_image", node->getT(), tmp_left_image );
    img_data_mgr->getImage( "depth_image", node->getT(), tmp_depth_image );
    left_image = tmp_left_image.clone();
    depth_image = tmp_depth_image.clone();
    #endif

    #if 0
    // this causes threading issues, better this operation be atomic
    img_data_mgr->getImage( "left_image",  node->getT(),  left_image );
    img_data_mgr->getImage( "depth_image", node->getT(), depth_image );
    #endif


    vector<string> nsX;        nsX.push_back( "left_image"); nsX.push_back( "depth_image");
    vector<cv::Mat> ou;
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_image_data] img_data_mgr->getImage(), input vector<string> nsX.size=" << nsX.size() << " \n" )
    bool getim_status = img_data_mgr->getImage( nsX, node->getT(), ou );
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_image_data] ou.size = " << ou.size() << "\tgetim_status=" << getim_status << endl; )

    if( getim_status == false )
    {
        cout << "[Cerebro::retrive_image_data] WARN cannot retrive image from the image manager\n";
        return false;
    }

    assert( nsX.size() == ou.size() && nsX.size() == 2 );
    left_image = ou[0];//.clone();
    depth_image = ou[1];//.clone();
    // ou.clear();

    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_image_data]";


    cout << "left_image " << MiscUtils::cvmat_info( left_image ) << "\t";
    cout << "depth_image " << MiscUtils::cvmat_info( depth_image ) << "\n";
    )

    w_T_c = node->getPose();
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_image_data] w_T_c = " << PoseManipUtils::prettyprintMatrix4d( w_T_c ) << endl;
    cout << "[Cerebro::retrive_image_data] OK!\n";
    )

    return true;
}


bool Cerebro::retrive_image_data( ros::Time& stamp, cv::Mat& left_image )
{
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_image_data 2] stamp=" << stamp << endl; )

    auto img_data_mgr = dataManager->getImageManagerRef();
    auto data_map = dataManager->getDataMapRef();

    if( data_map->count( stamp ) == 0  )
    {
        cout << "[Cerebro::retrive_image_data 2] FATAL-ERROR. timestamps="<< stamp << " not found in data_map. This is not possible. If this happens, this is definately a bug, report to the authors\n";
        exit(1);
    }

    const DataNode * node = data_map->at( stamp );
    bool is_pose = node->isPoseAvailable();
    if( is_pose == false )
    {
        cout << "[Cerebro::retrive_image_data 2] WARN cannot retrive pose\n";
        return false;
    }

    __Cerebro__retrive_image_data__( cout << "[Cerebro::retrive_image_data 2]pose is available\n" );




    vector<string> nsX;        nsX.push_back( "left_image"); //nsX.push_back( "depth_image");
    vector<cv::Mat> ou;
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_image_data] img_data_mgr->getImage(), input vector<string> nsX.size=" << nsX.size() << " \n" )
    bool getim_status = img_data_mgr->getImage( nsX, node->getT(), ou );
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_image_data] ou.size = " << ou.size() << "\tgetim_status=" << getim_status << endl; )

    if( getim_status == false )
    {
        cout << "[Cerebro::retrive_image_data] WARN cannot retrive image from the image manager\n";
        return false;
    }

    assert( nsX.size() == ou.size()  );
    left_image = ou[0];//.clone();
    // ou.clear();

    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_image_data 2]";


    cout << "left_image " << MiscUtils::cvmat_info( left_image ) << "\n";
    )


    return true;
}


bool Cerebro::retrive_depthimage_data( ros::Time& stamp, cv::Mat& depth_image )
{
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_depthimage_data] stamp=" << stamp << endl; )

    auto img_data_mgr = dataManager->getImageManagerRef();
    auto data_map = dataManager->getDataMapRef();

    if( data_map->count( stamp ) == 0  )
    {
        cout << "[Cerebro::retrive_depthimage_data] FATAL-ERROR. timestamps="<< stamp << " not found in data_map. This is not possible. If this happens, this is definately a bug, report to the authors\n";
        exit(1);
    }

    const DataNode * node = data_map->at( stamp );
    bool is_pose = node->isPoseAvailable();
    if( is_pose == false )
    {
        cout << "[Cerebro::retrive_depthimage_data] WARN cannot retrive pose\n";
        return false;
    }

    __Cerebro__retrive_image_data__( cout << "[Cerebro::retrive_depthimage_data]pose is available\n" );




    vector<string> nsX;     nsX.push_back( "depth_image");
    vector<cv::Mat> ou;
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_depthimage_data] img_data_mgr->getImage(), input vector<string> nsX.size=" << nsX.size() << " \n" )
    bool getim_status = img_data_mgr->getImage( nsX, node->getT(), ou );
    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_depthimage_data] ou.size = " << ou.size() << "\tgetim_status=" << getim_status << endl; )

    if( getim_status == false )
    {
        cout << "[Cerebro::retrive_depthimage_data] WARN cannot retrive image from the image manager\n";
        return false;
    }

    assert( nsX.size() == ou.size()  );
    depth_image = ou[0];//.clone();
    // ou.clear();

    __Cerebro__retrive_image_data__(
    cout << "[Cerebro::retrive_depthimage_data 2]";


    cout << "depth_image " << MiscUtils::cvmat_info( depth_image ) << "\n";
    )


    return true;
}

void Cerebro::publish_pose_from_seq(
    const vector<ros::Time>& seq_a_T, const vector<int>& seq_a_idx, const vector<Matrix4d>& seq_a_odom_pose,
    const vector<ros::Time>& seq_b_T, const vector<int>& seq_b_idx, const vector<Matrix4d>& seq_b_odom_pose,
    const Matrix4d& opt_a0_T_b0
)
{
    cerebro::LoopEdge loopedge_msg;
    geometry_msgs::Pose pose;

    std::default_random_engine generator;
    std::uniform_int_distribution<int> rand_a(0, (int)seq_a_idx.size() );
    std::uniform_int_distribution<int> rand_b(0, (int)seq_b_idx.size() );

    // for( int g=0 ; g<10 ; g++ )
    {
    int ra = 0;//rand_a(generator);
    int rb = 0;//rand_b(generator);

    Matrix4d a0_T_ra = seq_a_odom_pose[0].inverse() * seq_a_odom_pose[ra];
    Matrix4d b0_T_rb = seq_b_odom_pose[0].inverse() * seq_b_odom_pose[rb];

    Matrix4d rb_T_ra = b0_T_rb.inverse() * opt_a0_T_b0.inverse() * a0_T_ra;

    loopedge_msg.timestamp0 = seq_a_T[ra];
    loopedge_msg.timestamp1 = seq_b_T[rb];
    PoseManipUtils::eigenmat_to_geometry_msgs_Pose( rb_T_ra, pose );
    loopedge_msg.pose_1T0 = pose;
    loopedge_msg.weight = 1.0; //1.0;
    loopedge_msg.description = to_string( *( seq_a_idx.begin()  ) )+","+to_string( *( seq_a_idx.rbegin() ) )+"<=>"+to_string( *(seq_b_idx.begin()) )+","+to_string(  *(seq_b_idx.rbegin()) );
    loopedge_msg.description += "    this pose is: "+to_string(seq_b_idx[rb])+"_T_"+to_string(seq_a_idx[ra]);
    // __Cerebro__compute_geometry_for_loop_hypothesis_i( cout << loopedge_msg << endl; )
    pub_loopedge.publish( loopedge_msg );
    }

}


void Cerebro::publish_pose_from_seq(
    const vector<ros::Time>& seq_a_T, const vector<int>& seq_a_idx, const vector<Matrix4d>& seq_a_odom_pose,
    const vector<ros::Time>& seq_b_T, const vector<int>& seq_b_idx, const vector<Matrix4d>& seq_b_odom_pose,
    const LocalBundle& bundle
)
{
    cerebro::LoopEdge loopedge_msg;
    geometry_msgs::Pose pose;

    std::default_random_engine generator;
    std::uniform_int_distribution<int> rand_a(0, (int)seq_a_idx.size() );
    std::uniform_int_distribution<int> rand_b(0, (int)seq_b_idx.size() );

    int ra=0, rb=0;
    for( int g=0 ; g<10 ; g++ )
    {
    ra = rand_a(generator);
    rb = rand_b(generator);


    Matrix4d rb_T_ra = bundle.retrive_optimized_pose( 0, ra, 1, rb ).inverse();

    loopedge_msg.timestamp0 = seq_a_T.at(ra);
    loopedge_msg.timestamp1 = seq_b_T.at(rb);
    PoseManipUtils::eigenmat_to_geometry_msgs_Pose( rb_T_ra, pose );
    loopedge_msg.pose_1T0 = pose;
    loopedge_msg.weight = 1.0; //1.0;
    loopedge_msg.description = to_string( *( seq_a_idx.begin()  ) )+","+to_string( *( seq_a_idx.rbegin() ) )+"<=>"+to_string( *(seq_b_idx.begin()) )+","+to_string(  *(seq_b_idx.rbegin()) );
    loopedge_msg.description += "    this pose is: "+to_string(seq_b_idx.at(rb))+"_T_"+to_string(seq_a_idx.at(ra));
    // __Cerebro__compute_geometry_for_loop_hypothesis_i( cout << loopedge_msg << endl; )
    pub_loopedge.publish( loopedge_msg );
    }

}

//------------------------------------------------------------------//
//------------- END Geometry for Loop Hypothesis -------------------//
//------------------------------------------------------------------//

#if 0
// ^^^ Make this to one to open the function related to geometry of loop-candidates.
//     It is completely usable, it is removed for clean up, as I was to use loop hypothesis as a final product.

//--------------------------------------------------------------//
//------------------ Geometry Thread ---------------------------//
//--------------------------------------------------------------//

// #define __Cerebro__loopcandi_consumer__(msg) msg;
#define __Cerebro__loopcandi_consumer__(msg)  ;
// ^This will also imshow image-pairs with gms-matches marked.

// #define __Cerebro__loopcandi_consumer__IMP( msg ) msg;
#define __Cerebro__loopcandi_consumer__IMP( msg ) ;
// ^Important Text only printing


#define __Cerebro__loopcandi_consumer__IMSHOW 0 // will not produce the images (ofcourse will not show as well)
// #define __Cerebro__loopcandi_consumer__IMSHOW 1 // produce the images and log them, will not imshow
// #define __Cerebro__loopcandi_consumer__IMSHOW 2 // produce the images and imshow them, don't log

// Just uncomment it to disable consistency check.
// #define __Cerebro__loopcandi_consumer__no_pose_consistency_check
void Cerebro::loopcandiate_consumer_thread()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    assert( b_loopcandidate_consumer && "you need to call loopcandidate_consumer_enable() before loopcandiate_consumer_thread() can start executing\n" );
    cout << TermColor::GREEN() <<   "[Cerebro::loopcandiate_consumer_thread] Start thread" << TermColor::RESET() << endl;

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

    cout << TermColor::RED() <<  "[Cerebro::loopcandiate_consumer_thread] disable called, quitting loopcandiate_consumer_thread" << TermColor::RESET() << endl;

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
    #if 0
    // raw stereo-images in gray
    if( !(node->isImageAvailable()) || !(node->isImageAvailable(1)) ) {
        cout << TermColor::RED() << "[Cerebro::retrive_stereo_pair] Either of the node images (stereo-pair was not available). This is probably not fatal, this loopcandidate will be skipped." << TermColor::RESET() << endl;
        return false;
    }

    const cv::Mat& bgr_left_image = node->getImage();
    const cv::Mat& bgr_right_image = node->getImage(1);
    #else
    auto img_data_mgr = dataManager->getImageManagerRef();

    if( img_data_mgr->isImageRetrivable( "left_image", node->getT() )==false )
    {
        cout << TermColor::RED() << "[Cerebro::retrive_stereo_pair] img_data_mgr->isImageRetrivable( \"left_image\"," << node->getT() << ") returned false" << TermColor::RESET() << endl;
        cout << TermColor::RED() << "[Cerebro::retrive_stereo_pair] Either of the node images (stereo-pair was not available). This is probably not fatal, this loopcandidate will be skipped." << TermColor::RESET() << endl;
        return false;
    }

    if (   img_data_mgr->isImageRetrivable( "right_image", node->getT() ) == false )
    {
        cout << TermColor::RED() << "[Cerebro::retrive_stereo_pair] img_data_mgr->isImageRetrivable( \"right_image\"," << node->getT() << ") returned false" << TermColor::RESET() << endl;
        cout << TermColor::RED() << "[Cerebro::retrive_stereo_pair] Either of the node images (stereo-pair was not available). This is probably not fatal, this loopcandidate will be skipped." << TermColor::RESET() << endl;
        // img_data_mgr->print_status();
        // cout << TermColor::RED() << "[Cerebro::retrive_stereo_pair] img_data_mgr->isImageRetrivable( \"right_image\"," << node->getT() << ") returned false" << TermColor::RESET() << endl;
        return false;
    }


    cv::Mat bgr_left_image, bgr_right_image;
    img_data_mgr->getImage( "left_image", node->getT(), bgr_left_image );
    img_data_mgr->getImage( "right_image", node->getT(), bgr_right_image );

            #if 0
            cout << "bgr_left_image: " << MiscUtils::cvmat_info( bgr_left_image ) << "\n";
            cout << "bgr_right_image: " << MiscUtils::cvmat_info( bgr_right_image ) << "\n";
            #endif
    if( !(bgr_left_image.data) || !(bgr_right_image.data) )
    {
         cout << "[Cerebro::retrive_stereo_pair] Invalid images bfhjreturn false.\n";
         return false;
    }
    #endif

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
    cout << "[Cerebro::retrive_stereo_pair]left_image: " << MiscUtils::cvmat_info( left_image ) << "\n";
    cout << "[Cerebro::retrive_stereo_pair]right_image: " << MiscUtils::cvmat_info( right_image ) << "\n";
    cout << endl;
    #endif
    return true;

}



bool Cerebro::process_loop_candidate_imagepair_consistent_pose_compute( int ii, ProcessedLoopCandidate& proc_candi )
{
    auto u = foundLoops_i( ii );
    ros::Time t_curr = std::get<0>(u);
    ros::Time t_prev = std::get<1>(u);
    double score = std::get<2>(u);

    auto data_map = dataManager->getDataMapRef();
    assert( data_map->count( t_curr ) > 0 && data_map->count( t_prev ) > 0  && "One or both of the timestamps in foundloops where not in the data_map. This cannot be happening...fatal...\n" );
    int idx_1 = std::distance( data_map->begin(), data_map->find( t_curr )  );
    int idx_2 = std::distance( data_map->begin(), data_map->find( t_prev )  );

    DataNode * node_1 = data_map->find( t_curr )->second;
    DataNode * node_2 = data_map->find( t_prev )->second;




    __Cerebro__loopcandi_consumer__IMP(
    cout << TermColor::BLUE() << "{"<<ii <<  "} process: "<< idx_1 << "<--->" << idx_2 << TermColor::RESET() << "\tt_curr=" << t_curr << " <--> t_prev=" << t_prev << endl;
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
    __Cerebro__loopcandi_consumer__( cout << "[Cerebro::process_loop_candidate_imagepair_consistent_pose_compute] 3d points from frame_a\n"; )
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
    __Cerebro__loopcandi_consumer__( cout << "[Cerebro::process_loop_candidate_imagepair_consistent_pose_compute] 3d points from frame_b\n"; )
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
    __Cerebro__loopcandi_consumer__( cout << "[Cerebro::process_loop_candidate_imagepair_consistent_pose_compute]point matches between a_left, b_left\n"; )
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
    #if 1
    //--
    float pnp_goodness = StaticTheiaPoseCompute::PNP( world_point_uv, feature_position_uv_d, op1__b_T_a, pnp__msg  );
    //--END
    #else
    //--
    op1__b_T_a = odom_b_T_a; // setting initial guess as odometry rel pose with translation as zero
    // op1__b_T_a(0,3) = 0.0; op1__b_T_a(1,3) = 0.0; op1__b_T_a(2,3) = 0.0;
    float pnp_goodness = StaticCeresPoseCompute::PNP( world_point_uv, feature_position_uv_d, op1__b_T_a, pnp__msg  );
    //--END
    #endif
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
    MiscUtils::append_status_image( pi_dst_img, string( "^this is image b=")+to_string(idx_2)+";plot( PI( op1__b_T_a * world_point_uv ) ) in green on imB;plot( PI( odom__b_T_a * world_point_uv ) ) in blue on imB;plot( uv ) in red on imB;plot( uv_d) in pink on imB;>> green and pink show alignment quality; >> ignore blue and red");


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
    #if 1
    //--
    float pnp_goodness_optioN_B = StaticTheiaPoseCompute::PNP( world_point_uv_d, feature_position_uv, op2__a_T_b, pnp__msg_option_B  );
    //--END
    #else
    //--
    op2__a_T_b = odom_b_T_a.inverse();  //initial guess same as odometry
    // op2__a_T_b(0,3)=0.0;op2__a_T_b(1,3)=0.0;op2__a_T_b(2,3)=0.0;
    float pnp_goodness_optioN_B = StaticCeresPoseCompute::PNP( world_point_uv_d, feature_position_uv, op2__a_T_b, pnp__msg_option_B  );
    //--END
    #endif

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
    MiscUtils::append_status_image( pi_dst_img, string( "^this is image a=")+to_string(idx_1)+";plot( PI( op2__a_T_b * world_point_uv_d ) ) in green on imA;plot( PI( odom__b_T_a * world_point_uv ) ) in blue on imA ;plot( uv ) in red on imA;plot( uv_d) in pink on imA;>> green and red show alignment quality;>> ignore blue and pink");


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
    #if 1
    //--
    float p3p_goodness = StaticTheiaPoseCompute::P3P_ICP( uv_X, uvd_Y, icp_b_T_a, p3p__msg );
    //--END
    #else
    //--
    icp_b_T_a = odom_b_T_a;
    // icp_b_T_a(0,3) = 0.0; icp_b_T_a(1,3) = 0.0; icp_b_T_a(2,3) = 0.0;
    float p3p_goodness = StaticCeresPoseCompute::P3P_ICP( uv_X, uvd_Y, icp_b_T_a, p3p__msg );

    //--END
    #endif


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
    cv::resize(pi_dst_img,pi_dst_img, cv::Size(), 0.6, 0.6 );
    MiscUtils::append_status_image( pi_dst_img, string( "^this is image b=")+to_string(idx_2)+";plot( PI( icp_b_T_a * world_point_uv ) ) in green on imB ;plot( PI( odom__b_T_a * world_point_uv ) ) in blue on imB;plot( uv ) in red on imB;plot( uv_d) in pink on imB");


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


//////////// END pose computation //////////////////////////////


//--------------------------------------------------------------//
//------------------ END Geometry Thread -----------------------//
//--------------------------------------------------------------//
#endif // entire geometry


///////////////////////////////////////////////////////////
//---------------------- KIDNAP -------------------------//
///////////////////////////////////////////////////////////

// Kidnap identification thread. This thread monitors dataManager->getDataMapRef().size
// for every new node added if there are zero tracked features means that, I have
// been kidnaped. It however declares kidnap only after 2 sec of kidnaped
// #define __Cerebro__kidnaped_thread__( msg ) msg;
#define __Cerebro__kidnaped_thread__( msg ) ;

// #define __Cerebro__kidnaped_thread__debug( msg ) msg;
#define __Cerebro__kidnaped_thread__debug( msg ) ;
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


    //---
    //--- Main Settings for this thread
    //---
    const int THRESH_N_FEATS = 15; //declare kidnap if number of tracked features fall below 15
    const ros::Duration WAIT_BEFORE_DECLARING_AS_KIDNAP = ros::Duration(3.0); // wait this many seconds, of low feature tracking count before declaring kidnap


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
        new_count = data_map->size();

        if( new_count <= prev_count ) {
            __Cerebro__kidnaped_thread__(cout << "[Cerebro::kidnaped_thread]Nothing new\n";)
            continue;
        }

        ros::Time lb = data_map->rbegin()->first - ros::Duration(5, 0); // look at recent 5sec.
        // auto S = data_map.begin();
        auto S = data_map->lower_bound( lb );
        auto E = data_map->end();
        __Cerebro__kidnaped_thread__debug( cout << "[Cerebro::kidnaped_thread] S=" << S->first << "  E=" << E->first <<  endl; )

        for( auto it = S ; it != E ; it++ )
        {

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

            __Cerebro__kidnaped_thread__debug(
                if( n_feats > 50 )
                    cout <<it->first << ": n_feats=" << TermColor::iGREEN() << n_feats << TermColor::RESET() << endl;
                if( n_feats > 30 && n_feats<=50 )
                    cout <<it->first << ": n_feats=" << TermColor::GREEN() << n_feats << TermColor::RESET() << endl;
                if( n_feats > 20 && n_feats<=30 )
                    cout <<it->first << ": n_feats=" << TermColor::YELLOW() << n_feats << TermColor::RESET() << endl;
                if( n_feats > 10 && n_feats<=20 )
                    cout <<it->first << ": n_feats=" << TermColor::RED() << n_feats << TermColor::RESET() << endl;
                if( n_feats<=10 )
                    cout <<it->first << ": n_feats=" << TermColor::iRED() << n_feats << TermColor::RESET() << endl;
            )

            last_known_keyframe = it->first;


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
    if( i>=0 && i<(int)start_of_kidnap.size() ) {
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
    for( int i=0 ; i<(int)start_of_kidnap.size() ; i++ )
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

bool Cerebro::is_kidnapped(){ return state_is_kidnapped; }


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






//////////////////////////////////////////////////////////////
//---------------------- KIDNAP END-------------------------//
//////////////////////////////////////////////////////////////



//-----------------------------------------------------------------
// Cerebro Private Utils
//-----------------------------------------------------------------
// private function for descriptor_dot_product thread
bool Cerebro::wait_until__connectedToDescServer_and_descSizeAvailable( int timeout_in_sec )  //blocking call
{
    assert( timeout_in_sec > 2 );
    ros::Rate rate(10);
    // wait until connected_to_descriptor_server=true and descriptor_size_available=true
    int wait_itr = 0;
    while( true ) {
        if( this->connected_to_descriptor_server && this->descriptor_size_available)
            break;
        __Cerebro__run__(cout << wait_itr << " [Cerebro::wait_until__connectedToDescServer_and_descSizeAvailable] waiting for `descriptor_size_available` to be true. timeout_in_sec=" << timeout_in_sec << "\n";)
        rate.sleep();
        wait_itr++;
        if( wait_itr > timeout_in_sec*10 ) {
            __Cerebro__run__( cout << TermColor::RED() << "[Cerebro::wait_until__connectedToDescServer_and_descSizeAvailable] `this->connected_to_descriptor_server && this->descriptor_size_available` has not become true dispite waiting for about "<< timeout_in_sec << " sec. So quiting the run thread.\n" << TermColor::RESET(); )
            return false;
        }
    }
    return true;
}


//-----------------------------------------------------------------
// END Cerebro Private Utils
//-----------------------------------------------------------------
