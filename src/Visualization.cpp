#include "Visualization.h"

Visualization::Visualization( ros::NodeHandle &nh )
{
    b_run_thread = false;
    this->nh = nh;

}

void Visualization::setDataManager( DataManager* dataManager )
{
    this->dataManager = dataManager;
    m_dataManager_available = true;
}

void Visualization::setVizPublishers( const string base_topic_name )
{
    //
    // Test Publisher
    string pub_topic_test = base_topic_name+"/chatter";
    ROS_INFO( "Visualization Publisher pub_topic_test: %s", pub_topic_test.c_str() );
    chatter_pub = nh.advertise<std_msgs::String>(pub_topic_test, 1000);

    //
    // Publish Markers of frame data
    string framedata_pub_topic = base_topic_name+"/framedata";
    ROS_INFO( "Visualization Publisher framedata_pub_topic: %s", framedata_pub_topic.c_str() );
    framedata_pub = nh.advertise<visualization_msgs::Marker>(framedata_pub_topic, 1000);

}

void Visualization::run()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Visualization before execution of the run() thread can begin. You can set the dataManager by call to Visualization::setDataManager()\n");
    assert( b_run_thread && "you need to call run_thread_enable() before run() can start executing\n" );

    while( b_run_thread )
    {
        cout << "Visualization::run() " << dataManager->getDataMapRef().size() <<endl;
        this->publish_frames();
        this->publish_test_string();
        std::this_thread::sleep_for( std::chrono::milliseconds( 100 )  );
    }
}

void Visualization::publish_frames()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Visualization before execution of the run() thread can begin. You can set the dataManager by call to Visualization::setDataManager()\n");

    static std::map<ros::Time, bool > XC;
    static visualization_msgs::Marker linestrip_marker;
    RosMarkerUtils::init_line_strip_marker( linestrip_marker );
    linestrip_marker.ns = "cam_line_strip";
    linestrip_marker.id = 0;


    auto data_map = dataManager->getDataMapRef();
    cout << "---\n";
    if( data_map.begin() == data_map.end() ) {
        cout << "nothing to vizualize\n";
        return;
    }

    // cout << data_map.rbegin()->first << "\t"<<  data_map.rbegin()->first - dataManager->getPose0Stamp() << endl;
    ros::Time lb = data_map.rbegin()->first - ros::Duration(3);
    for( auto it = data_map.lower_bound( lb ); it != data_map.end() ; it++ ) {
        // cout << std::distance( it, data_map.begin() ) << "] " << it->first << "\t" << it->first - dataManager->getPose0Stamp() << endl;

        if( XC.count( it->first) == 0 ) {
            cout << "sizeof(XC)=" << XC.size() << "  "<< it->first << "\t" << it->first - dataManager->getPose0Stamp() << endl;

            if( it->second->isPoseAvailable() ) {
                auto w_T_c = it->second->getPose();
                geometry_msgs::Point pt;
                pt.x = w_T_c(0,3);
                pt.y = w_T_c(1,3);
                pt.z = w_T_c(2,3);
                linestrip_marker.points.push_back( pt );
                framedata_pub.publish( linestrip_marker );
                XC[it->first] = true;
            }
        }
    }
    return ;

}

/*

void Visualization::publish_frames()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Visualization before execution of the run() thread can begin. You can set the dataManager by call to Visualization::setDataManager()\n");

    static std::map< ros::Time, bool > pub_status;

    int SHOW_MAX = 10;
    visualization_msgs::Marker m;
    RosMarkerUtils::init_camera_marker( m, 1.0f );
    m.ns = "cam";

    static visualization_msgs::Marker line_marker;
    RosMarkerUtils::init_line_strip_marker( line_marker );
    line_marker.ns = "cam_line_strip";
    line_marker.id = 0;

    int sze = dataManager->getDataMapRef().size();
    for( auto it = dataManager->getDataMapRef().rbegin() ; SHOW_MAX > 0 && it != dataManager->getDataMapRef().rend()   ; it++ , SHOW_MAX-- )
    {
        bool pose_available = it->second->isPoseAvailable();
        cout << "\tMap-key : " << it->first << "\t" << (pose_available?"Pose N/A":"Pose Available") <<  endl;

        if( false && pose_available ) {
            m.id = sze - SHOW_MAX;
            auto w_T_c = it->second->getPose();
            RosMarkerUtils::setpose_to_marker( w_T_c, m );
            framedata_pub.publish( m );
        }

        if( pose_available && pub_status.count( it->first ) == 0 ) {
            auto w_T_c = it->second->getPose();
            geometry_msgs::Point pt;
            pt.x = w_T_c(0,3);
            pt.y = w_T_c(1,3);
            pt.z = w_T_c(2,3);
            line_marker.points.push_back( pt );
            framedata_pub.publish( line_marker );
        }

        pub_status[it->first] = true;
    }
}
*/

void Visualization::publish_test_string()
{
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << ros::Time::now();
    msg.data = ss.str();
    chatter_pub.publish( msg );
}
