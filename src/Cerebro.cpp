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


///
/// This implements a simple loopclosure detection scheme based on dot-product of descriptor-vectors.
///
/// TODO: In the future more intelligent schemes can be experimented with. Besure to run those in new threads and disable this thread.
/// wholeImageComputedList is a list for which descriptors are computed. Similarly other threads can compute
/// scene-object labels, text etc etc in addition to currently computed whole-image-descriptor
#define __Cerebro__run__( msg ) msg ;
// #define __Cerebro__run__( msg ) ;
void Cerebro::run()
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

            if( abs(u_argmax - um_argmax) < 8 && abs(u_argmax-umm_argmax) < 8 && u_max > 0.92  )
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

                {
                std::lock_guard<std::mutex> lk_foundloops(m_foundLoops);
                foundLoops.push_back( std::make_tuple( wholeImageComputedList[l-1], wholeImageComputedList[u_argmax], u_max ) );
                }
            }

        }
        else {
            cout << "do nothing. not seen enough yet.\n";
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
    assert( nrow > 0 && ncols > 0 && desc_size > 0 );
    cout << "[Cerebro::descriptor_computer_thread] nrows=" << nrows << "  ncols=" << ncols << "  desc_size=" << desc_size << endl;
    this->descriptor_size = int(desc_size);
    descriptor_size_available = true;





    ros::Rate rate(20);
    while( b_descriptor_computer_thread )
    {
        auto data_map = dataManager->getDataMapRef();
        if( data_map.begin() == data_map.end() ) {
            ROS_INFO( "nothing to compute descriptor\n" );
            std::this_thread::sleep_for( std::chrono::milliseconds( 1000 )  );
            continue;
        }
        ros::Time lb = data_map.rbegin()->first - ros::Duration(10, 0); // look at recent 10sec.
        auto S = data_map.lower_bound( lb );
        auto E = data_map.end();
        for( auto it = S; it != E ; it++ ) {
            //descriptor does not exisit at this stamp, so compute it.
            // Here I compute the whole image descriptor only at keyframes, you may try something like, if the pose is available compute it.
            if( it->second->isWholeImageDescriptorAvailable() == false && it->second->isKeyFrame() )  {

                __Cerebro__descriptor_computer_thread(ElapsedTime _time);
                __Cerebro__descriptor_computer_thread(_time.tic());

                // use it->second->getImage() to compute descriptor. call the service
                const cv::Mat image_curr = it->second->getImage();
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
    assert( i >= 0 && i<foundLoops.size() );
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
