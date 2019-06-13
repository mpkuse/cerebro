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

    cout << "[HypothesisManager::add_node] add_node("<<a <<", " << b << ", " << dot_prod << ")\n";

    //-----------a
    bool was_in_one_of_the_active_hyp = false;
    cout << "[HypothesisManager::add_node] #active_hyp=" << active_hyp.size() << endl;
    for( auto it=active_hyp.begin() ; it!=active_hyp.end() ; it++ ) // loop over active hyp
    {
        if( was_in_one_of_the_active_hyp == true )
            break;

        cout << "active_hyp#" << it-active_hyp.begin() << endl;
        it->print_active_hypothesis();

        for( auto c=it->list_of_nodes_in_this_hypothesis.begin() ;
                  c!=it->list_of_nodes_in_this_hypothesis.end() ;
                  c++    )
        {
            int _a = std::get<0>( *c );
            int _b = std::get<1>( *c );
            float _dot_prod = std::get<2>( *c );

            if( abs(a-_a) < 5 && abs(b-_b) < 5 ) {
                it->list_of_nodes_in_this_hypothesis.push_back( make_tuple(a,b,dot_prod) );
                was_in_one_of_the_active_hyp = true;
                printf( "Added (%d,%d,%f) in active_hyp#\n", a,b,dot_prod, it-active_hyp.begin() );
                break;
            }
        }
    }



    //-------------b
    if( was_in_one_of_the_active_hyp == false ) {
        cout << "New hyp "<< a<< ", " << b << ", " << dot_prod << endl;
        active_hyp.push_back( Hypothesis( a,b, dot_prod) );
    }


}
