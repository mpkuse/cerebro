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

void Visualization::setCerebro( Cerebro* cerebro )
{
    this->cerebro = cerebro;
    m_cerebro_available = true;
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

    // can add image publish if need be here.
    string imagepaire_pub_topic = base_topic_name+"/imagepaire";
    ROS_INFO( "Visualization Publisher imagepair_pub_topic: %s", imagepaire_pub_topic.c_str() );
    imagepaire_pub = nh.advertise<sensor_msgs::Image>(imagepaire_pub_topic, 1000);
}

void Visualization::run( const int looprate    )
{
    assert( m_dataManager_available && "You need to set the DataManager in class Visualization before execution of the run() thread can begin. You can set the dataManager by call to Visualization::setDataManager()\n");
    assert( b_run_thread && "you need to call run_thread_enable() before run() can start executing\n" );
    assert( looprate > 0  && "[ Visualization::run] looprate need to be postive\n");

    // Adjust these for debugging.
    bool bpub_framepositions = false;          //< This publishes text-marker with node id
    bool bpub_loopcandidates = true;           //< this publishes line marker
    bool bpub_processed_loopcandidates = true; //this publoshes the image-pair as image.

    bool bpub_loop_hypothesis_candidates = true;

    ros::Rate rate( looprate );
    while( b_run_thread )
    {
        // cout << "Visualization::run() " << dataManager->getDataMapRef().size() <<endl;
        if( bpub_framepositions )
            this->publish_frames();

        if( bpub_loopcandidates )
            this->publish_loopcandidates(); //< this publishes marker
        // this->publish_test_string();

        #if 0
        if( bpub_processed_loopcandidates )
            this->publish_processed_loopcandidates(); //< this publoshes the image-pair as image.
        #endif

        if( bpub_loop_hypothesis_candidates ) {
            this->publish_loop_hypothesis_lines(); //< cerebro->loop_hypothesis_i \forall i
            this->publish_loop_hypothesis_image_pair(); //< image pair
        }

        rate.sleep();
    }
}

#if 0
// #define __Visualization___publish_processed_loopcandidates(msg) msg;
#define __Visualization___publish_processed_loopcandidates(msg) ;
void Visualization::publish_processed_loopcandidates()
{
    assert( m_dataManager_available && m_cerebro_available && "You need to set cerebro and DataManager in class Visualization  before execution of the run() thread can begin. You can set the cerebro by call to Visualization::setCerebro() and dataManager as setDataManager.\n");

    static int prev_count = -1;

    if( prev_count == cerebro->processedLoops_count() || prev_count<0 ) {
        prev_count = cerebro->processedLoops_count();
        // nothing new
        return;
    }

    int new_count = cerebro->processedLoops_count();
    // [prev_count , new_count ] are new

    // cout << "[Visualization::publish_processed_loopcandidates]" << new_count << endl;
    __Visualization___publish_processed_loopcandidates(
    cout << "#new procs_loops=" << new_count - prev_count << "  from [" << prev_count << "," << new_count-1 << "]\n";
    )

    visualization_msgs::Marker marker;
    RosMarkerUtils::init_line_marker( marker );
    marker.ns = "processed_loopcandidates_line";

    // loop over all the new
    int loop_start = prev_count;
    int loop_end = new_count;
    bool publish_image = true;
    bool publish_loop_line_marker = false;
    // if( rand()%100 < 2 ) {
        // loop_start=0;
        // publish_image=false;
    // }

    for( int i=loop_start ; i< loop_end; i++ )
    {
        // publish green colored line
        ProcessedLoopCandidate candidate_i =  cerebro->processedLoops_i( i );
        __Visualization___publish_processed_loopcandidates(
        cout << "---\nUsing processedLoop["<<i<<"]"<<endl;
        cout << candidate_i.node_1->getT() << "<--->" << candidate_i.node_2->getT() << endl;
        cout << "isPoseAvailable: " << candidate_i.node_1->isPoseAvailable() << endl;
        )


        if( publish_loop_line_marker )
        {
            Vector4d w_t_1 = candidate_i.node_1->getPose().col(3);
            Vector4d w_t_2 = candidate_i.node_2->getPose().col(3);
            RosMarkerUtils::add_point_to_marker( w_t_1, marker, true );
            RosMarkerUtils::add_point_to_marker( w_t_2, marker, false );
            RosMarkerUtils::setcolor_to_marker( 0.0, 1.0, 0.0 , marker );
            marker.ns = "processed_loopcandidates_line";
            marker.id = i;
            framedata_pub.publish( marker );


            // Publish marker line
            if( candidate_i.isSet_3d2d__2T1 == false )
            {
                cout << "[Visualization::publish_processed_loopcandidates] _3d2d__2T1 is not set. This means the final pose is not set. This is because the candidate relative poses do not appear to be consistent with each other. Ignoring this ProcessedLoopCandidate" << endl;;
                continue;
            }

            Matrix4d w_T_2__new = candidate_i.node_1->getPose() * (candidate_i._3d2d__2T1).inverse();
            Vector4d w_t_2__new = w_T_2__new.col(3);
            RosMarkerUtils::add_point_to_marker( w_t_1, marker, true );
            RosMarkerUtils::add_point_to_marker( w_t_2__new, marker, false );
            RosMarkerUtils::setcolor_to_marker( 1.0, 1.0, 1.0 , marker );
            marker.ns = "processed_loopcandidates_new_position_of_2";
            marker.id = i;
            framedata_pub.publish( marker );
        }


        // publish image
        if( publish_image)
        {
            #if  0
            if( candidate_i.node_1->isImageAvailable() && candidate_i.node_2->isImageAvailable() )
            #else
            auto img_data_mgr = dataManager->getImageManagerRef();
            if(
                img_data_mgr->isImageRetrivable("left_image", candidate_i.node_1->getT()) &&
                img_data_mgr->isImageRetrivable("left_image", candidate_i.node_2->getT())
              )
            #endif
            {

                #if 0 // code where data_node had image data, mark this for removal
                cv::Mat _node_1_im = candidate_i.node_1->getImage();
                cv::Mat _node_2_im = candidate_i.node_2->getImage();
                #else // image manager has the image data
                cv::Mat _node_1_im, _node_2_im;
                bool status1 = img_data_mgr->getImage( "left_image", candidate_i.node_1->getT(), _node_1_im );
                bool status2 = img_data_mgr->getImage( "left_image", candidate_i.node_2->getT(), _node_2_im );
                assert( status1 && status2 && "[Visualization::publish_processed_loopcandidates]\n");
                #endif

                // use image from node
                cv::Mat side_by_side_impair;
                MiscUtils::side_by_side( _node_1_im, _node_2_im, side_by_side_impair );

                string sgg = std::to_string( candidate_i.idx_from_datamanager_1  )
                                + "                    "+
                             std::to_string( candidate_i.idx_from_datamanager_2  );
                 string sgg_time = std::to_string( candidate_i.node_1->getT().toSec()  )
                                 + "             "+
                              std::to_string( candidate_i.node_2->getT().toSec()  );


                MiscUtils::append_status_image( side_by_side_impair, ";;"+sgg,
                        2.0, cv::Scalar(0,0,0), cv::Scalar(255,255,255), 3.0 );
                MiscUtils::append_status_image( side_by_side_impair, ";"+sgg_time+";;processedLoop["+to_string(i)+"]",
                        1.5, cv::Scalar(0,0,0), cv::Scalar(255,255,255), 3.0 );

                if( candidate_i.isSet_3d2d__2T1 ) {
                    MiscUtils::append_status_image( side_by_side_impair, ";Pose Estimation was consistent, so Accept",
                            1.5, cv::Scalar(0,0,0), cv::Scalar(0,255,0), 3.0 );
                } else {
                    MiscUtils::append_status_image( side_by_side_impair, ";Inconsistent Pose Estimation, so Reject",
                            1.5, cv::Scalar(0,0,0), cv::Scalar(0,0,255), 3.0 );
                }

                cv::Mat buffer;
                cv::resize(side_by_side_impair, buffer, cv::Size(), 0.5, 0.5 );

                cv_bridge::CvImage cv_image;
                cv_image.image = buffer;
                if( buffer.channels() == 1 )
                    cv_image.encoding = "mono8";
                else if( buffer.channels() == 3 )
                    cv_image.encoding = "bgr8";
                else {
                    cout << TermColor::RED() << "[Visualization::publish_processed_loopcandidates]invalid number of channels EXIT\n";
                    exit(1);
                }

                sensor_msgs::Image ros_image_msg;
                cv_image.toImageMsg(ros_image_msg);
                __Visualization___publish_processed_loopcandidates(
                    cout << MiscUtils::imgmsg_info( ros_image_msg ) << endl;
                )
                imagepaire_pub.publish( ros_image_msg );
            }
        }


    }
    prev_count = new_count;
}
#endif

// #define __Visualization__publish_loopcandidates(msg) msg
#define __Visualization__publish_loopcandidates(msg) ;
void Visualization::publish_loopcandidates()
{
    assert( m_dataManager_available && m_cerebro_available && "You need to set cerebro and DataManager in class Visualization  before execution of the run() thread can begin. You can set the cerebro by call to Visualization::setCerebro() and dataManager as setDataManager.\n");

    // last 10 publish
    int n = cerebro->foundLoops_count();

    int start = max( 0, n - 10 );
    // 5% of the time start from 0.
    if( rand() % 100 < 2 ) start = 0;
    __Visualization__publish_loopcandidates(cout << "[Visualization::publish_loopcandidates] start=" << start << " end=" << n << endl;)

    if( n <= 0 ) return;

    auto data_map = dataManager->getDataMapRef();
    visualization_msgs::Marker marker;
    RosMarkerUtils::init_line_marker( marker );
    marker.ns = "loopcandidates_line";
    RosMarkerUtils::setcolor_to_marker( 1.0, 0.0, 0.0 , marker );
    // TODO If need be can also publish text for each (which could be score of the edge)


    // for( int i=0 ; i<n ; i++ )
    for( int i=start ; i<n ; i++ )
    {

        marker.id = i;

        auto u = cerebro->foundLoops_i( i );
        ros::Time t_curr = std::get<0>(u);
        ros::Time t_prev = std::get<1>(u);
        double score = std::get<2>(u);

        assert( data_map->count( t_curr ) > 0 && data_map->count( t_prev ) > 0  && "One or both of the timestamps in foundloops where not in the data_map. This cannot be happening...fatal...\n" );
        int idx_1 = std::distance( data_map->begin(), data_map->find( t_curr )  );
        int idx_2 = std::distance( data_map->begin(), data_map->find( t_prev )  );

        Vector4d w_t_curr = data_map->at(t_curr)->getPose().col(3);
        Vector4d w_t_prev = data_map->at(t_prev)->getPose().col(3);

        // TODO - need to test. looks like alright.
        // add_point_to_marker with w_t_curr
        // add_point_to_marker with w_t_prev
        RosMarkerUtils::add_point_to_marker( w_t_curr, marker, true );
        RosMarkerUtils::add_point_to_marker( w_t_prev, marker, false );



        framedata_pub.publish( marker );
    }

}


// #define __Visualization__publish_frames( cmd ) cmd
#define __Visualization__publish_frames( cmd ) ;
void Visualization::publish_frames()
{
    assert( m_dataManager_available && "You need to set the DataManager in class Visualization before execution of the run() thread can begin. You can set the dataManager by call to Visualization::setDataManager()\n");

    // Adjust these manually to change behaviour
    bool publish_camera_visual = false;
    bool publish_camera_as_point = true;
    bool publish_txt = true;
    bool publish_verbose_txt = false;


    static std::map< ros::Time, int > XC;

    __Visualization__publish_frames(cout << TermColor::RED() << "---" << TermColor::RESET() << endl;)
    __Visualization__publish_frames(cout << "start... sizeof(XC)=" << XC.size() << endl;)
    auto data_map = dataManager->getDataMapRef();
    auto pose0 = dataManager->getPose0Stamp();


    visualization_msgs::Marker cam_vis; //visualize every pose as a cam-visual
    RosMarkerUtils::init_camera_marker( cam_vis , .5 );
    cam_vis.ns = "cam_pose_vis";


    visualization_msgs::Marker pt_vis; //visualize every pose as a point.
    RosMarkerUtils::init_points_marker( pt_vis );
    geometry_msgs::Point zer; zer.x =0.; zer.y=0.; zer.z = 0;
    pt_vis.points.push_back( zer );
    pt_vis.ns = "cam_pose_pt";
    pt_vis.scale.x = 0.015;
    pt_vis.scale.y = 0.015;


    visualization_msgs::Marker txt_vis;
    RosMarkerUtils::init_text_marker( txt_vis );
    txt_vis.ns = "cam_pose_txt";
    txt_vis.scale.z = 0.03;


    for( auto it = data_map->begin() ; it != data_map->end() ; it++ )
    {
        if( XC[ it->first ] < 10 || rand() % 100 < 2 ) { // publish only if not already published 10 times
            int seq_id = std::distance( data_map->begin() , it );
            cam_vis.id = seq_id;
            pt_vis.id = seq_id;
            txt_vis.id = seq_id;

            // Set Colors
            if( it->second->isKeyFrame() ) {
                RosMarkerUtils::setcolor_to_marker( 0., 1., 0. , cam_vis );
                RosMarkerUtils::setcolor_to_marker( 0., 1., 0. , pt_vis );
                RosMarkerUtils::setcolor_to_marker( 1., 1., 1. , txt_vis );

                // if( it->second->isWholeImageDescriptorAvailable() )
                    // RosMarkerUtils::setcolor_to_marker( 0., 0, 1. , cam_vis );
            }
            else {
                RosMarkerUtils::setcolor_to_marker( 1., 0., 0. , cam_vis );
                RosMarkerUtils::setcolor_to_marker( 1., 0., 0. , pt_vis );
                RosMarkerUtils::setcolor_to_marker( 1., 1., 1. , txt_vis );
            }


            // Set Pose
            if( it->second->isPoseAvailable() ) {
                auto wTc = it->second->getPose();
                RosMarkerUtils::setpose_to_marker( wTc , cam_vis );
                RosMarkerUtils::setpose_to_marker( wTc , pt_vis );

                RosMarkerUtils::setpose_to_marker( wTc , txt_vis );
                txt_vis.text = "";
                txt_vis.text += std::to_string(seq_id) + ";";
                if( publish_verbose_txt )  {
                    txt_vis.text += std::to_string( (it->first - pose0).toSec() ) +";";
                    txt_vis.text += std::to_string( (it->first).toSec() ) +";";
                }

                if( publish_camera_visual )
                    framedata_pub.publish( cam_vis );
                if( publish_camera_as_point )
                    framedata_pub.publish( pt_vis );
                if( publish_txt || publish_verbose_txt )
                    framedata_pub.publish( txt_vis );
            }


            if( it->second->isKeyFrame() )
                __Visualization__publish_frames(cout << TermColor::GREEN() );
            __Visualization__publish_frames( cout << "Publish seq_id=" << seq_id << "\t xc=" << XC[ it->first ] << "\t t="<< it->first - pose0 << "\t" << it->first << TermColor::RESET() << endl; )

            XC[ it->first ]++;
        }
    }
    __Visualization__publish_frames( cout << "Done... sizeof(XC)=" << XC.size() << endl; )
}
/*
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
*/

void Visualization::publish_test_string()
{
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << ros::Time::now();
    msg.data = ss.str();
    chatter_pub.publish( msg );
}


// #define __Visualization__publish_loop_hypothesis_lines_(msg) msg;
#define __Visualization__publish_loop_hypothesis_lines_(msg) ;
void Visualization::publish_loop_hypothesis_lines()
{

    assert( m_dataManager_available && m_cerebro_available && "You need to set cerebro and DataManager in class Visualization  before execution of the run() thread can begin. You can set the cerebro by call to Visualization::setCerebro() and dataManager as setDataManager.\n");
    if( cerebro->is_loop_hypothesis_manager_allocated() == false ) {
        __Visualization__publish_loop_hypothesis_lines_( cout << "[Visualization::publish_loop_hypothesis_lines] Loop Hypothesis manager seem to be not allocated, do nothing...\n"; )
        return;
    }



    visualization_msgs::Marker marker;
    RosMarkerUtils::init_line_marker( marker );
    marker.ns = "loop_hypothesis_line";
    RosMarkerUtils::setcolor_to_marker( 1.0, 0.0, 0.0 , marker );



    static int prev_count = 0;
    int istart, iend;
    int curr_count = cerebro->loop_hypothesis_count();

    istart = prev_count;
    iend   = curr_count;

    if( curr_count == prev_count )
    {
        if( rand() % 1000 < 5 )
            istart = 0; //once in a while publish all.
        else
            return;
    }


    auto data_map = dataManager->getDataMapRef();


    __Visualization__publish_loop_hypothesis_lines_(
    cout << "[Visualization::publish_loop_hypothesis_lines] i=" << prev_count << " to " << curr_count << endl;)
    for( int i=istart ; i<iend ; i++ )
    {
        ros::Time seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T;
        cerebro->loop_hypothesis_i_T( i, seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T );

        int  seq_a_start, seq_a_end, seq_b_start, seq_b_end;

        __Visualization__publish_loop_hypothesis_lines_(
        cerebro->loop_hypothesis_i_idx( i, seq_a_start, seq_a_end, seq_b_start, seq_b_end );
        cout << "---\n";
        cout << "#" << i << " ";
        cout << "(" << seq_a_start << "," << seq_a_end << ")";
        cout << "<----->";
        cout << "(" << seq_b_start << "," << seq_b_end << ")";
        cout << endl;
        cout << "#" << i << " ";
        cout << "(" << seq_a_start_T << "," << seq_a_end_T << ")";
        cout << "<----->";
        cout << "(" << seq_b_start_T << "," << seq_b_end_T << ")";
        cout << endl;

        )

        // get poses at these points
        assert( data_map->count( seq_a_start_T ) > 0 && data_map->count( seq_b_start_T ) > 0  && "One or both of the timestamps in loop_hypothesis_count where not in the data_map. This cannot be happening...fatal...\n" );

        #if 0
        // more elaborate code, useful for debugging...
        if( data_map->at(seq_a_start_T)->isPoseAvailable() == false ) {
            cout << TermColor::RED() << "[Visualization::publish_loop_hypothesis_lines] pose not available at i=" << i << "\tseq_a_start_T=" << seq_a_start_T << TermColor::RESET() << endl;
            data_map->at(seq_a_start_T)->prettyPrint();
            continue;
        }


        if( data_map->at(seq_b_start_T)->isPoseAvailable() == false ) {
            cout << TermColor::RED() << "[Visualization::publish_loop_hypothesis_lines] pose not available at i=" << i  << "\tseq_b_start_T" << seq_b_start_T << TermColor::RESET() << endl;
            data_map->at(seq_b_start_T)->prettyPrint();
            continue;
        }
        #else
        if(
            data_map->at(seq_a_start_T)->isPoseAvailable() == false ||
            data_map->at(seq_a_end_T)->isPoseAvailable() == false ||
            data_map->at(seq_b_start_T)->isPoseAvailable() == false ||
            data_map->at(seq_b_end_T)->isPoseAvailable() == false
        ) {
            cout << TermColor::YELLOW() << "[Visualization::publish_loop_hypothesis_lines] WARN pose not available. Usually 1st few seconds poses are not available while the VINS initializes. So, if a part of seq is in first few sec this happens. This is normal behaviour." << TermColor::RESET() << endl;
            continue;
        }
        #endif



        #if 1
        Vector4d w_t_a_start = data_map->at(seq_a_start_T)->getPose().col(3);
        Vector4d w_t_b_start = data_map->at(seq_b_start_T)->getPose().col(3);

        Vector4d w_t_a_end = data_map->at(seq_a_end_T)->getPose().col(3);
        Vector4d w_t_b_end = data_map->at(seq_b_end_T)->getPose().col(3);
        __Visualization__publish_loop_hypothesis_lines_( cout << i << " : " << w_t_a_start.transpose() << "<---------->" << w_t_b_start.transpose()<< endl; )

        RosMarkerUtils::add_point_to_marker( w_t_a_start, marker, true );
        RosMarkerUtils::add_point_to_marker( w_t_b_start, marker, false );

        RosMarkerUtils::add_point_to_marker( w_t_a_end, marker, false );
        RosMarkerUtils::add_point_to_marker( w_t_b_end, marker, false );
        marker.id = i;
        #endif
        framedata_pub.publish( marker );
    }

    prev_count = curr_count;

}



// #define __Visualization__publish_loop_hypothesis_image_pair(msg) msg;
#define __Visualization__publish_loop_hypothesis_image_pair(msg) ;

void Visualization::publish_loop_hypothesis_image_pair()
{
    assert( m_dataManager_available && m_cerebro_available && "You need to set cerebro and DataManager in class Visualization  before execution of the run() thread can begin. You can set the cerebro by call to Visualization::setCerebro() and dataManager as setDataManager.\n");
    if( cerebro->is_loop_hypothesis_manager_allocated() == false ) {
        __Visualization__publish_loop_hypothesis_image_pair( cout << "[Visualization::publish_loop_hypothesis_image_pair] Loop Hypothesis manager seem to be not allocated, do nothing...\n"; )
        return;
    }


    static int prev_count = 0;

    int curr_count = cerebro->loop_hypothesis_count();
    if( curr_count == prev_count )
        return;

    auto img_data_mgr = dataManager->getImageManagerRef();

    for( int i=prev_count ; i<curr_count ; i++ )
    {
        //make image for loophypothesis#i

        int seq_a_start, seq_a_end, seq_b_start, seq_b_end;
        ros::Time seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T;

        cerebro->loop_hypothesis_i_idx( i,  seq_a_start, seq_a_end, seq_b_start, seq_b_end  );
        cerebro->loop_hypothesis_i_T( i,   seq_a_start_T, seq_a_end_T, seq_b_start_T, seq_b_end_T );

        __Visualization__publish_loop_hypothesis_image_pair(
        cout << "#" << i << " ";
        cout << "(" << seq_a_start_T << "," << seq_a_end_T << ")";
        cout << "<----->";
        cout << "(" << seq_b_start_T << "," << seq_b_end_T << ")";
        cout << "\t";
        cout << "#" << i << " ";
        cout << "(" << seq_a_start << "," << seq_a_end << ")";
        cout << "<----->";
        cout << "(" << seq_b_start << "," << seq_b_end << ")";
        cout << endl;
        )

        cv::Mat seq_a_start_im, seq_a_end_im, seq_b_start_im, seq_b_end_im;
        bool status = true;
        {
        if( img_data_mgr->isImageRetrivable( "left_image", seq_a_start_T ) ) {
            // img_data_mgr->getImage( "left_image", seq_a_start_T, seq_a_start_im );
        }
        else
            status = false;

        if( img_data_mgr->isImageRetrivable( "left_image", seq_a_end_T ) ) {
            img_data_mgr->getImage( "left_image", seq_a_end_T, seq_a_end_im );
        }
        else
            status = false;

        if( img_data_mgr->isImageRetrivable( "left_image", seq_b_start_T ) ) {
            // img_data_mgr->getImage( "left_image", seq_b_start_T, seq_b_start_im );
        }
        else
            status = false;

        if( img_data_mgr->isImageRetrivable( "left_image", seq_b_end_T ) ) {
            img_data_mgr->getImage( "left_image", seq_b_end_T, seq_b_end_im );
        }
        else
            status = false;
        }
        assert( status == true );

        if( status == false ) {cout << "[Visualization::publish_loop_hypothesis_image_pair] WARN. Something is wrong, one or more images cannot be retrieved, so not publish image pair"; }
        else
        {
            cv::Mat dst, dst_org;
            __Visualization__publish_loop_hypothesis_image_pair(
            cout << "seq_a_end_im: " << MiscUtils::cvmat_info( seq_a_end_im ) << "\t";
            cout << "seq_b_end_im: " << MiscUtils::cvmat_info( seq_b_end_im ) << "\n";)
            MiscUtils::side_by_side( seq_a_end_im, seq_b_end_im , dst_org );

            std::stringstream buffer;
            buffer << ";Hypothesis#" << i ;
            buffer << ";this: " << seq_a_end << "(ie. " << seq_a_end_T << ")";
            buffer << "  ... ";
            buffer << seq_b_end << "(ie. " << seq_b_end_T << ")   resized(0.5);;";

            buffer << "#" << i << " ";
            buffer << "(" << seq_a_start_T << "," << seq_a_end_T << ")";
            buffer << ";;<----->;;";
            buffer << "(" << seq_b_start_T << "," << seq_b_end_T << ")";
            buffer << ";;";
            buffer << "#" << i << " ";
            buffer << "(" << seq_a_start << "," << seq_a_end << ")";
            buffer << "<----->";
            buffer << "(" << seq_b_start << "," << seq_b_end << ");";

            // TODO : Make a good informative string, rather than hap-hazard like this.
            // string status_string = "#" + to_string(i) + ": (" +  to_string(seq_a_start)+","+to_string(seq_a_end) + ") <---> (" + to_string(seq_b_start)+","+to_string(seq_b_end) + ")";
            MiscUtils::append_status_image( dst_org , buffer.str() ,
                            1.0,  cv::Scalar(0,0,0), cv::Scalar(255,255,255), 3 );


            cv::resize( dst_org, dst, cv::Size(), 0.5, 0.5 );


            // make this #if 1 to imshow, make this to 0 to publish image data, customize as needed
            #if 0
            cv::imshow( "pair", dst );
            cv::waitKey(10);
            #else
            cv_bridge::CvImage cv_image;
            cv_image.image = dst;
            if( dst.channels() == 1 )
                cv_image.encoding = "mono8";
            else if( dst.channels() == 3 )
                cv_image.encoding = "bgr8";
            else {
                cout << TermColor::RED() << "[Visualization::publish_loop_hypothesis_image_pair]invalid number of channels EXIT\n";
                exit(1);
            }

            sensor_msgs::Image ros_image_msg;
            cv_image.toImageMsg(ros_image_msg);
            imagepaire_pub.publish( ros_image_msg );
            #endif
        }

    }
    prev_count = curr_count;

}
