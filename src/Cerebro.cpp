#include "Cerebro.h"


Cerebro::Cerebro( ros::NodeHandle& nh )
{
    b_run_thread = false;
    b_descriptor_computer_thread = false;
    this->nh = nh;
}

void Cerebro::setDataManager( DataManager* dataManager )
{
    this->dataManager = dataManager;
    m_dataManager_available = true;
}


void Cerebro::run()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    assert( b_run_thread && "you need to call run_thread_enable() before run() can start executing\n" );

    ros::Rate rate(10);


    int l=0, last_l=0;
    int last_processed=0;
    MatrixXd M = MatrixXd::Zero( 8192, 5000 );
    while( b_run_thread )
    {
        rate.sleep();
        continue; 
        auto data_map = dataManager->getDataMapRef();
        {
            std::lock_guard<std::mutex> lk(m_wholeImageComputedList);
            l = wholeImageComputedList.size();
        }

        if( l - last_l < 3 ) {
            // cout << "nothing new\n";
            rate.sleep();
            continue;
        }

        cout << TermColor::RED() << "---" << TermColor::RESET() << endl;
        cout << "l=" << l << endl;

        assert(  data_map.count( wholeImageComputedList[l-1] ) > 0  &&
                 data_map.count( wholeImageComputedList[l-2] ) > 0  &&
                 data_map.count( wholeImageComputedList[l-3] ) > 0 &&
                 "either of l, l-1, l-2 is not available in the datamap"
             );

        // auto f = data_map.find( wholeImageComputedList[l-1] );
        // auto fm = data_map.find( wholeImageComputedList[l-2] );
        // auto fmm = data_map.find( wholeImageComputedList[l-3] );

        VectorXd v   = data_map[ wholeImageComputedList[l-1] ]->getWholeImageDescriptor();
        VectorXd vm  = data_map[ wholeImageComputedList[l-2] ]->getWholeImageDescriptor();
        VectorXd vmm = data_map[ wholeImageComputedList[l-3] ]->getWholeImageDescriptor();



        // This is very inefficient. Better to have a matrix-vector product and not getWholeImageDescriptor() all the time.
        assert( M.rows() == v.size() );
        assert( M.cols() > l );
        M.col( l-1 ) = v;
        M.col( l-2 ) = vm;
        M.col( l-3 ) = vmm;

        int k = l - 50; // given a stamp, l, get another stamp k

        //usable size of M is 8192xl, let k (k<l) be the length until which dot is needed by time.
        if( k > 5 ) {
            ElapsedTime s;
            s.tic();
            VectorXd u   = v.transpose() * M.leftCols( k );
            VectorXd um  = vm.transpose() * M.leftCols( k );
            VectorXd umm = vmm.transpose() * M.leftCols( k );
            cout << "Done in ms=" << s.toc_milli() << endl;
            cout << "<v , M[0 to "<< k << "]\n";

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
                cout << TermColor::RED() <<  u_argmax << "\t" << um_argmax << "\t" << umm_argmax << TermColor::RESET() << endl;

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
    cout << "Attempt connecting to ros-service for 10sec\n";
    ros::ServiceClient client = nh.serviceClient<cerebro::WholeImageDescriptorCompute>( "/whole_image_descriptor_compute" );
    client.waitForExistence( ros::Duration(10, 0) ); //wait maximum 10 sec
    if( !client.exists() ) {
        ROS_ERROR( "Connection to server NOT successful. Quiting the thread." );
        return;
    }
    else std::cout << TermColor::GREEN() <<  "Connection to ros-service ``" << client.getService() << "`` established" << TermColor::RESET() << endl;




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
