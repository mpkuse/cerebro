#include "ProcessedLoopCandidate.h"

ProcessedLoopCandidate::ProcessedLoopCandidate( int idx_from_raw_candidates_list,
        DataNode * node_1, DataNode * node_2 )
{
    // cout << "ProcessedLoopCandidate::ProcessedLoopCandidate\n";
    this->node_1 = node_1;
    this->node_2 = node_2;
    this->idx_from_raw_candidates_list = idx_from_raw_candidates_list;
}
