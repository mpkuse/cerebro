#include "HypothesisManager.h"


HypothesisManager::HypothesisManager()
{
    std::cout << "HypothesisManager constructor\n";
}

HypothesisManager::~HypothesisManager()
{
    std::cout << "HypothesisManager descrutctor\n";
}


bool HypothesisManager::add_node( int a, int b, float dot_prod )
{
    // See if (a,b) is in a already active hypothesis ?
    //      |---(yes) give strength, possibly increase its time-to-live (ttl)
    //      |---(no ) is it worth creating new hypothesis?
    //              |--- (yes)
    //              |--- (no) discard this
    std::lock_guard<std::mutex> lk(mx);

    cout << "\n[HypothesisManager::add_node] add_node("<<a <<", " << b << ", " << dot_prod << ")\n";

    //-----------a
    bool was_in_one_of_the_active_hyp = false;
    cout << "[HypothesisManager::add_node] #active_hyp=" << active_hyp.size() << endl;
    for( auto it=active_hyp.begin() ; it!=active_hyp.end() ; it++ ) // loop over active hyp
    {
        if( was_in_one_of_the_active_hyp == true )
            break;

        #if 0
        // cout << "active_hyp#" << it-active_hyp.begin() << endl;
        auto header_str = string("");
        if( it == active_hyp.begin() )
            header_str = "\n---\n";
        header_str +="active_hyp#" + std::to_string(  it-active_hyp.begin() );
        it->print_active_hypothesis(header_str);
        #endif

        for( auto c=it->list_of_nodes_in_this_hypothesis.rbegin() ;
                  c!=it->list_of_nodes_in_this_hypothesis.rend() ;
                  c++    )
        {
            int _a = std::get<0>( *c );
            int _b = std::get<1>( *c );
            float _dot_prod = std::get<2>( *c );

            if( abs(a-_a) < 7 && abs(b-_b) < 7 ) {
                it->list_of_nodes_in_this_hypothesis.push_back( make_tuple(a,b,dot_prod) );
                was_in_one_of_the_active_hyp = true;
                printf( "Added (%d,%d,%f) in active_hyp#%d\n", a,b,dot_prod, it-active_hyp.begin() );

                printf( "Increment time_to_live of this hypothesis\n" );
                it->increment_ttl();
                break;
            }
        }
    }



    //-------------b
    if( was_in_one_of_the_active_hyp == false ) {
        cout << "New Hypothesis added: "<< a<< ", " << b << ", " << dot_prod << endl;
        active_hyp.push_back( Hypothesis( a,b, dot_prod) );
    }


}

void HypothesisManager::digest()
{
    std::lock_guard<std::mutex> lk(mx);

    for( auto it=active_hyp.begin() ; it!=active_hyp.end() ; it++ )
    {
        it->decrement_ttl();
        it->decrement_ttl();
        it->decrement_ttl();
        it->decrement_ttl();
        // it->decrement_ttl();
    }
}


void HypothesisManager::monitoring_thread()
{
    cout << TermColor::GREEN() << "[HypothesisManager::monitoring_thread] thread started\n" << TermColor::RESET() << endl;
    while( b_monitoring_thread )
    {
        {   // threadsafe zone
            std::lock_guard<std::mutex> lk(mx);
            ofstream myfile;
            myfile.open ("/dev/pts/1");
            int n_actives = 0;
            for( int i=0 ; i<active_hyp.size() ; i++ )
            {
                if( i==0 )
                    myfile << "---\n";

                if( active_hyp.at(i).is_hypothesis_active() ) {
                    n_actives++;
                    myfile << "i=" << i << "  " << active_hyp.at(i).info_string() << endl;
                }
            }

            if( n_actives > 0 ) {
                myfile << TermColor::GREEN() << "Currently active hypothesis=" << n_actives <<  TermColor::RESET() << endl;
            }
            else {
                myfile << TermColor::RED() << "No active hypothesis" << TermColor::RESET() << endl;
            }
            myfile.close();

        } // end of threadsafe zone

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }

    cout << TermColor::RED() << "[HypothesisManager::monitoring_thread] thread Ended\n" << TermColor::RESET() << endl;
}
