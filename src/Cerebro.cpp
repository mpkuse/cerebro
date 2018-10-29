#include "Cerebro.h"


Cerebro::Cerebro()
{
    b_run_thread = false;
    b_descriptor_computer_thread = false;
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

    while( b_run_thread )
    {
        cout << "Cerebro::run() " << dataManager->getDataMapRef().size() <<endl;;
        std::this_thread::sleep_for( std::chrono::milliseconds( 1000 )  );
    }
}

void Cerebro::descriptor_computer_thread()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Cerebro before execution of the run() thread can begin. You can set the dataManager by call to Cerebro::setDataManager()\n");
    assert( b_descriptor_computer_thread && "You need to call descriptor_computer_thread_enable() before spawning the thread\n" );

    ros::Rate rate(10);
    while( b_descriptor_computer_thread )
    {
        auto data_map = dataManager->getDataMapRef();
        if( data_map.begin() == data_map.end() ) {
            ROS_INFO( "nothing to compute descriptor\n" );
            std::this_thread::sleep_for( std::chrono::milliseconds( 1000 )  );
            continue;
        }
        ros::Time lb = data_map.rbegin()->first - ros::Duration(10);
        auto S = data_map.lower_bound( lb );
        auto E = data_map.end();
        for( auto it = S; it != E ; it++ ) {
            if( descriptors.count( it->first ) == 0 && it->second->isKeyFrame() )  { //descriptor does not exisit at this stamp, so compute it

                // use it->second->getImage() to compute descriptor. call the service
                std::this_thread::sleep_for( std::chrono::milliseconds( 30 )  );


                double result = sin(it->first.toSec() );
                descriptors[ it->first ] = result;
                cout << "Computed descriptor at t=" << it->first - dataManager->getPose0Stamp() << "\t" << it->first << endl;
            }
        }

        rate.sleep();
    }


    cout << "descriptors were compute at these timestamps : \n";
    for( auto it = descriptors.begin(); it != descriptors.end() ; it++ )
        cout << it->first - dataManager->getPose0Stamp() << "\t" <<  it->first << "\t" << it->second << endl;

}
