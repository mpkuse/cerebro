#include "HypothesisManager.h"

HypothesisManager::HypothesisManager()
{

}


bool HypothesisManager::add_node( int i, int j, double dot_product_score, int n_nn )
{
    // assert( i > j );
    if( n_accum == 0 )
        i_start = i;

    i_latest = i;
    // int I = int( i/W );
    int J = int( j/W );


    if( M.count(J) == 0 )
    {
        M[J] = 0;
    }

    if( dot_product_score > THRESH ) {//if greater than threshold boost is another 20%
        dot_product_score *= 1.2;
        n_greater_than_thresh++;
    }

    if( dot_product_score < LOW_THRESH ) {
        dot_product_score *= 0.2; // if lower than the LOW_THRESH, suppress this.
    }

    M[J] += dot_product_score * (0.1 * n_nn + 1.1);  // n_nn == 1 ==> 1.0; n_nn == 2 ==> 0.9 ...
    n_accum++;


    bool new_hyp_added = digest();
    return new_hyp_added;

}

// #define __HypothesisManager__digest___( msg ) msg;
#define __HypothesisManager__digest___( msg ) ;
bool HypothesisManager::digest()
{
    if( n_accum == FLUSH_AFTER_N_ACCUMULATES ) //flush after 50, this is approximately 2.5 second if using 5 nearest neighbours in cerebro.
    {
        __HypothesisManager__digest___(
        // print M
        cout << "\n========================\n";
        cout << i_start << " ------ " << i_latest << endl;
        cout << "========================\n";
        for( auto it = M.begin() ; it != M.end() ; it++ )
        {
            cout << W * (it->first)  << " : " << it->second << endl;
        }
        cout << "n_greater_than_thresh = " << n_greater_than_thresh << " of " << FLUSH_AFTER_N_ACCUMULATES << endl;
        cout << "========================\n";
        )



        //? any conclusions from this?
        // (i_start,i) <----> ( W * (it->first)  , W * (it->first+1) ) iff it->second > 20.
        bool new_hyp_added = false;
        for( auto it = M.begin() ; it != M.end() ; it++ ) {
            if( it->second > MANDATE_SCORE_THRESH && n_greater_than_thresh > 5 ) {
                std::lock_guard<std::mutex> lk(mutex_hyp_q);

                __HypothesisManager__digest___(
                cout << TermColor::GREEN();
                cout << "[HypothesisManager::digest]ADD : " << i_start << "," << i_latest <<  "<--->" << it->first * W << ", " << (it->first+1)*W << endl;
                cout << TermColor::RESET();
                )
                vector<int> tmp = {i_start,i_latest,     W * (it->first)  , W * (it->first+1) };
                hyp_q.push_back( tmp );
                new_hyp_added = true;
            }
        }



        n_greater_than_thresh = 0;
        n_accum = 0;
        M.clear();
        return new_hyp_added;
    }

    return false;
}


void HypothesisManager::print_hyp_q_all() const
{
    std::lock_guard<std::mutex> lk(mutex_hyp_q);

    cout << "[HypothesisManager::print_hyp_q_all] hyp_q.size() = " << hyp_q.size() << endl;
    for( int i=0 ; i<(int)hyp_q.size() ; i++ )
    {
        cout << "#" << i << " ";
        cout << "(" << hyp_q[i][0] << "," << hyp_q[i][1] << ")";
        cout << "<---->";
        cout << "(" << hyp_q[i][2] << "," << hyp_q[i][3] << ")";
        cout << endl;
    }
    cout << "[HypothesisManager::print_hyp_q_all] DONE "<< endl;

}


int HypothesisManager::n_hypothesis() const
{
    std::lock_guard<std::mutex> lk(mutex_hyp_q);
    return (int) hyp_q.size();
}


bool HypothesisManager::hypothesis_i(int i, int& seq_a_start, int&  seq_a_end, int&  seq_b_start, int&  seq_b_end ) const
{
    std::lock_guard<std::mutex> lk(mutex_hyp_q);
    assert( i>=0 && i<(int)hyp_q.size() );

    if( i<0 || i>= (int) hyp_q.size() )
    {
        cout << TermColor::RED() << "[HypothesisManager::hypothesis_i] ERROR, you requested " << i << "th hypothesis however hyp_q.size()="<< hyp_q.size() << endl << TermColor::RESET();
        return false;
    }

    seq_a_start = hyp_q[i][0];
    seq_a_end   = hyp_q[i][1];
    seq_b_start = hyp_q[i][2];
    seq_b_end   = hyp_q[i][3];
    return true;

}


//-------- Set/Get pose info for the specified hypothesis

void HypothesisManager::set_computed_pose( int i, const Matrix4d a_T_b, const string info_str )
{
    std::lock_guard<std::mutex> lk(mutex_hyp_q);
    if( i<0 || i>= (int)hyp_q.size() )
        throw "[HypothesisManager::set_computed_pose] invalid i";
    m_computed_pose[i] = true;
    computed_pose_a_T_b[i] = a_T_b;
    computed_pose_info[i] = info_str;
}

bool HypothesisManager::is_computed_pose_available( int i ) const
{
    std::lock_guard<std::mutex> lk(mutex_hyp_q);
    if( i<0 || i>= (int)hyp_q.size() )
        throw "[HypothesisManager::set_computed_pose] invalid i";

    if( m_computed_pose.count(i) > 0 )
        return true;
    else
        return false;
}

Matrix4d HypothesisManager::get_computed_pose( int i ) const
{
    if( i<0 || i>= (int)hyp_q.size()  )
        throw "[HypothesisManager::get_computed_pose] invalid i";

    if( is_computed_pose_available( i ) == false )
        throw "[HypothesisManager::get_computed_pose] pose not available at this i";


    std::lock_guard<std::mutex> lk(mutex_hyp_q);
    return computed_pose_a_T_b.at(i);
}

string HypothesisManager::get_computed_pose_info_string( int i ) const
{

    if( i<0 || i>= (int)hyp_q.size() )
        throw "[HypothesisManager::get_computed_pose_info_string] invalid i";

    if( is_computed_pose_available( i ) == false )
        throw "[HypothesisManager::get_computed_pose_info_string] info string not available at this i";

    std::lock_guard<std::mutex> lk(mutex_hyp_q);
    return computed_pose_info.at(i);

}
