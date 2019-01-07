#include "ProcessedLoopCandidate.h"

ProcessedLoopCandidate::ProcessedLoopCandidate( int idx_from_raw_candidates_list,
        DataNode * node_1, DataNode * node_2 )
{
    // cout << "ProcessedLoopCandidate::ProcessedLoopCandidate\n";
    this->node_1 = node_1;
    this->node_2 = node_2;
    this->idx_from_raw_candidates_list = idx_from_raw_candidates_list;
}


bool ProcessedLoopCandidate::makeLoopEdgeMsg(  cerebro::LoopEdge& msg )
{
    if( isSet_3d2d__2T1 == false ) {
        cout << TermColor::RED() << "You asked me to make cerebro::LoopEdge of ProcessedLoopCandidate in which there is no valid pose data" << TermColor::RESET() << endl;
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

    if( isSet_3d2d__2T1 ) {
        x["3d2d__2T1"] = PoseManipUtils::prettyprintMatrix4d( _3d2d__2T1 );
    }

    if( isSet_2d3d__2T1 ) {
        x["2d3d__2T1"] = PoseManipUtils::prettyprintMatrix4d( _2d3d__2T1 );
    }

    out_json = x;
    return true;
}
