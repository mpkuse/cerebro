#include "ProcessedLoopCandidate.h"

ProcessedLoopCandidate::ProcessedLoopCandidate( int idx_from_raw_candidates_list,
        DataNode * node_1, DataNode * node_2 )
{
    // cout << "ProcessedLoopCandidate::ProcessedLoopCandidate\n";
    this->node_1 = node_1;
    this->node_2 = node_2;
    this->idx_from_raw_candidates_list = idx_from_raw_candidates_list;

    opX_b_T_a.clear();
    debug_images.clear();
}


bool ProcessedLoopCandidate::makeLoopEdgeMsg(  cerebro::LoopEdge& msg )
{
    if( isSet_3d2d__2T1 == false ) {
        cout << TermColor::RED() << "[ProcessedLoopCandidate::makeLoopEdgeMsg]\
        You asked me to make cerebro::LoopEdge of ProcessedLoopCandidate in\
        which there is no valid pose data" << TermColor::RESET() << endl;
        return false;
    }
    msg.timestamp0 = node_1->getT();
    msg.timestamp1 = node_2->getT();

    geometry_msgs::Pose pose;
    PoseManipUtils::eigenmat_to_geometry_msgs_Pose( _3d2d__2T1, pose );
    msg.pose_1T0 = pose;

    msg.weight = _3d2d__2T1__ransac_confidence; //1.0;
    msg.description = to_string(idx_from_datamanager_1)+"<=>"+to_string(idx_from_datamanager_2);
    msg.description += "    this pose is: "+to_string(idx_from_datamanager_2)+"_T_"+to_string(idx_from_datamanager_1);

    return true;
}

// #define __ProcessedLoopCandidate_makeLoopEdgeMsgWithConsistencyCheck( msg )  msg;
#define __ProcessedLoopCandidate_makeLoopEdgeMsgWithConsistencyCheck( msg ) ;
bool ProcessedLoopCandidate::makeLoopEdgeMsgWithConsistencyCheck( cerebro::LoopEdge& msg )
{
    if( opX_b_T_a.size() != 3 ) {
        cout << TermColor::RED() << "[ProcessedLoopCandidate::makeLoopEdgeMsgWithConsistencyCheck] \
        I was expecting only 3 candidates" << TermColor::RED() << endl;
        return false;
    }


    ros::Duration diff = node_1->getT() - node_2->getT();
    if(  abs(diff.sec) < 10 ) // the candidates are too near in time, so ignore them
    {
        __ProcessedLoopCandidate_makeLoopEdgeMsgWithConsistencyCheck(
        cout << "[ ProcessedLoopCandidate::makeLoopEdgeMsgWithConsistencyCheck] abs(diff.sec) < 10. diff= "<< diff << endl;
        )
        return false;
    }

    ///////////////////////////////////////////////////////////
    // Look at all the options and determine if they are consistent.


    // if |del_angle| < 2.0 degrees and |del_translation| < 0.1m then not consistent
    Matrix4d op1__b_T_a = opX_b_T_a[0];
    Matrix4d op2__b_T_a = opX_b_T_a[1];
    Matrix4d icp_b_T_a = opX_b_T_a[2];
    Matrix4d op1_m_op2 = op1__b_T_a.inverse() * op2__b_T_a;
    Matrix4d op1_m_icp = op1__b_T_a.inverse() * icp_b_T_a;
    Matrix4d op2_m_icp = op2__b_T_a.inverse() * icp_b_T_a;
    Vector3d op1_m_op2_ypr, op1_m_op2_tr; // ypr and translation of delta pose
    Vector3d op1_m_icp_ypr, op1_m_icp_tr;
    Vector3d op2_m_icp_ypr,op2_m_icp_tr;
    PoseManipUtils::eigenmat_to_rawyprt( op1_m_op2, op1_m_op2_ypr, op1_m_op2_tr);
    PoseManipUtils::eigenmat_to_rawyprt( op1_m_icp, op1_m_icp_ypr, op1_m_icp_tr);
    PoseManipUtils::eigenmat_to_rawyprt( op2_m_icp, op2_m_icp_ypr, op2_m_icp_tr);
    bool is_consistent_ypr = false, is_consistent_tr=false;

    if( op1_m_op2_ypr.lpNorm<Eigen::Infinity>() < 5.0  &&
        op1_m_icp_ypr.lpNorm<Eigen::Infinity>() < 5.0  &&
        op2_m_icp_ypr.lpNorm<Eigen::Infinity>() < 5.0
    )
    { is_consistent_ypr = true;}

    if( op1_m_icp_tr.lpNorm<Eigen::Infinity>() < .2  &&
        op1_m_icp_tr.lpNorm<Eigen::Infinity>() < .2  &&
        op2_m_icp_tr.lpNorm<Eigen::Infinity>() < .2
    )
    { is_consistent_tr = true;}

    #if 0 //just priniting
    __ProcessedLoopCandidate_makeLoopEdgeMsgWithConsistencyCheck(
    Matrix4d odom_b_T_a = node_2->getPose().inverse() * node_1->getPose();
    cout << "---" << to_string(idx_from_datamanager_1)+"<=>"+to_string(idx_from_datamanager_2);
    cout << "[ProcessedLoopCandidate::makeLoopEdgeMsgWithConsistencyCheck]\n" << TermColor::YELLOW() << "odom_b_T_a = " << PoseManipUtils::prettyprintMatrix4d( odom_b_T_a ) << TermColor::RESET() << endl;

    cout << "op1" << "confidence=" << opX_goodness[0]<< " " << PoseManipUtils::prettyprintMatrix4d( op1__b_T_a ) << endl;
    cout << "op2" << "confidence=" << opX_goodness[1]<< " " << PoseManipUtils::prettyprintMatrix4d( op2__b_T_a ) << endl;
    cout << "icp" << "confidence=" << opX_goodness[2]<< " " << PoseManipUtils::prettyprintMatrix4d( icp_b_T_a ) << endl;

    cout << "|op1 - op2|" << PoseManipUtils::prettyprintMatrix4d( op1__b_T_a.inverse() * op2__b_T_a ) << endl;
    cout << "|op1 - icp|" << PoseManipUtils::prettyprintMatrix4d( op1__b_T_a.inverse() * icp_b_T_a ) << endl;
    cout << "|op2 - icp|" << PoseManipUtils::prettyprintMatrix4d( op2__b_T_a.inverse() * icp_b_T_a ) << endl;
    cout << TermColor::YELLOW() ;
    cout << "is_consistent_ypr=" << is_consistent_ypr << "\t" ;
    cout << "is_consistent_tr=" << is_consistent_tr << "\t";
    cout << TermColor::RESET() << endl;
    )
    #endif


    ///////////////////////////////// Done with consistency check /////////////////////

    if( pf_matches > 800 && (is_consistent_ypr && is_consistent_tr) ) {
    // put the final pose into `3d2d__2T1`
        _3d2d__2T1 = opX_b_T_a[0];
        isSet_3d2d__2T1 = true;
        _3d2d__2T1__ransac_confidence = max( max( opX_goodness[0], opX_goodness[1] ), opX_goodness[2] );

        // call the above function,
        return makeLoopEdgeMsg( msg );
    }
    else {
        cout << "[ ProcessedLoopCandidate::makeLoopEdgeMsgWithConsistencyCheck] Insonsistent poses from 3 methods, so don't publish this pose" <<  endl;
        return false;
    }
}


bool ProcessedLoopCandidate::asJson( json& out_json )
{
    json x;
    if( node_1 == NULL || node_2 == NULL )
        return false;

    x["timestamp_a"] = node_1->getT().toSec();
    x["timestamp_b"] = node_2->getT().toSec();
    x["idx_a"] = idx_from_datamanager_1;
    x["idx_b"] = idx_from_datamanager_2;
    x["pf_matches"] = pf_matches;
    x["final_pose_available"] = isSet_3d2d__2T1;

    if( isSet_3d2d__2T1 ) {
        x["3d2d__2T1"] = PoseManipUtils::prettyprintMatrix4d( _3d2d__2T1 );
        x["_3d2d__2T1__ransac_confidence"] = _3d2d__2T1__ransac_confidence;
    }


    // write the poses which were computed with various methods
    for( int i=0 ; i<opX_b_T_a.size() ; i++ )
    {
        json u;
        u["name"] = opX_b_T_a_name[i];
        u["goodness"] = opX_goodness[i];
        u["debugmsg"] = opX_b_T_a_debugmsg[i];
        u["b_T_a"] = PoseManipUtils::prettyprintMatrix4d( opX_b_T_a[i] );

        #if 1
        Vector3d _ypr, _t;
        PoseManipUtils::eigenmat_to_rawyprt( opX_b_T_a[i], _ypr, _t );
        u["b_T_a_yaw"] =   _ypr(0);
        u["b_T_a_pitch"] = _ypr(1);
        u["b_T_a_roll"] =  _ypr(2);
        u["b_T_a_tx"] =      _t(0);
        u["b_T_a_ty"] =      _t(1);
        u["b_T_a_tz"] =      _t(2);
        #endif

        x["opX"].push_back( u );
    }

    out_json = x;
    return true;
}
