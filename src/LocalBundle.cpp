#include "LocalBundle.h"


LocalBundle::LocalBundle()
{

}

//----------------------------------------------------------------------------//
//              INPUT
//----------------------------------------------------------------------------//
// #define __LocalBundle__input(msg) msg;
#define __LocalBundle__input(msg) ;

void LocalBundle::inputOdometry( int seqJ, vector<Matrix4d> _x0_T_c )
{
    if( this->x0_T_c.count( seqJ) != 0 ) {
        cout << TermColor::RED() << "[LocalBundle::inputOdometry] ERROR, the seqID=" << seqJ << ", which was the input already exisit. This mean for this seq you have already input the odometry.\n" << TermColor::RESET();
        cout << "....exit....\n";
        exit(2);
    }

    __LocalBundle__input(
    cout << "[LocalBundle::inputOdometry] set odometry for seqID=" << seqJ << " this seq has " << _x0_T_c.size() << " frames" <<  endl; )
    x0_T_c[ seqJ ] = _x0_T_c;
}


void LocalBundle::inputInitialGuess(  int seqa, int seqb, Matrix4d ___a0_T_b0 )
{
    auto p = std::make_pair( seqa, seqb );
    if( this->a0_T_b0.count(p) != 0 ) {
        cout << TermColor::RED() << "[LocalBundle::inputInitialGuess] ERROR the initial guess between 0th frames of seq= " << seqa << " and seq=" << seqb << " already exist. You are trying to set it again. This is not the intended purpose.\n" << TermColor::RESET();
        cout << "....exit....\n";
        exit(2);
    }

    __LocalBundle__input(
    cout << "[LocalBundle::inputInitialGuess] set initial guess for a0_T_b0={" << seqa << "}0_T_{" << seqb<< "}0" << endl; )
    this->a0_T_b0[ p ] = ___a0_T_b0;
}


void LocalBundle::inputOdometry_a0_T_b0( int seqa, int seqb, Matrix4d odom_a0_T_b0 )
{
    auto p = std::make_pair( seqa, seqb );
    if( this->odom__a0_T_b0.count(p) != 0 ) {
        cout << TermColor::RED() << "[LocalBundle::inputOdometry_a0_T_b0] ERROR the initial guess between 0th frames of seq= " << seqa << " and seq=" << seqb << " already exist. You are trying to set it again. This is not the intended purpose.\n" << TermColor::RESET();
        cout << "....exit....\n";
        exit(2);
    }

    __LocalBundle__input(
    cout << "[LocalBundle::inputOdometry_a0_T_b0] set odometry for a0_T_b0={" << seqa << "}0_T_{" << seqb<< "}0" << endl; )
    this->odom__a0_T_b0[ p ] = odom_a0_T_b0;
}


void LocalBundle::inputFeatureMatches( int seq_a, int seq_b,
    const vector<MatrixXd> all_normed_uv_a, const vector<MatrixXd> all_normed_uv_b )
{
    assert( all_normed_uv_a.size() == all_normed_uv_b.size() && all_normed_uv_a.size() > 0 );
    for( int i=0 ; i<(int)all_normed_uv_a.size() ; i++ )
    {
        assert( all_normed_uv_a[i].cols() == all_normed_uv_b[i].cols() );
        assert( all_normed_uv_a[i].rows() == all_normed_uv_b[i].rows() && (all_normed_uv_b[i].rows() == 2 || all_normed_uv_b[i].rows()==3) );
        // cout << "[LocalBundle::inputFeatureMatches]image-pair#" << i << " has " << all_normed_uv_a[i].cols() << " feature correspondences\n";
    }

    auto p = std::make_pair( seq_a, seq_b );
    if( this->normed_uv_a.count(p) !=0 || this->normed_uv_b.count(p) != 0 ) {
        cout << TermColor::RED() << "[LocalBundle::inputFeatureMatches] ERROR this pair " << seq_a << "," << seq_b << " already exists\n" << TermColor::RESET();
        cout << "exit...\n";
        exit(2);
    }

    __LocalBundle__input(
    cout << "[LocalBundle::inputFeatureMatches] Set correspondences for " << all_normed_uv_a.size() << " image-pairs in seqa=" << seq_a << ", seqb=" << seq_b << endl;
    for( int i=0 ; i<(int)all_normed_uv_a.size() ; i++ )
    {
        cout << ">>>> [LocalBundle::inputFeatureMatches]image-pair#" << i << " has " << all_normed_uv_a[i].cols() << " feature correspondences\n";
    }
    )
    normed_uv_a[p] = all_normed_uv_a;
    normed_uv_b[p] = all_normed_uv_b;
}


void LocalBundle::inputFeatureMatchesDepths( int seq_a, int seq_b,
    const vector<VectorXd> all_d_a, const vector<VectorXd> all_d_b, const vector<VectorXd> all_sf )
{
    assert( all_d_a.size() == all_d_b.size() && all_d_a.size() > 0 );
    for( int i=0 ; i<(int)all_d_a.size() ; i++ )
    {
        assert( all_d_a[i].size() == all_d_b[i].size() && all_d_a[i].size() > 0 );
        __LocalBundle__input(
        cout << "[LocalBundle::inputFeatureMatchesDepths]image-pair#" << i << " has " << all_d_a[i].size() << " depth values\n"; )
    }

    auto p = std::make_pair( seq_a, seq_b );
    if( this->d_a.count(p) !=0 || this->d_b.count(p) != 0 ) {
        cout << TermColor::RED() << "[LocalBundle::inputFeatureMatchesDepths] ERROR this pair " << seq_a << "," << seq_b << " already exists\n" << TermColor::RESET();
        cout << "exit...\n";
        exit(2);
    }

    __LocalBundle__input(
    cout << "[LocalBundle::inputFeatureMatchesDepths] Set depths for " << all_d_a.size() << " image-pairs in seqa=" << seq_a << ", seqb=" << seq_b << endl;
    for( int i=0 ; i<(int)all_d_a.size() ; i++ )
    {
        __LocalBundle__input(
        cout << ">>>> [LocalBundle::inputFeatureMatchesDepths]image-pair#" << i << " has " << all_d_a[i].size() << " depth values\n"; )
    }
    )

    d_a[p] = all_d_a;
    d_b[p] = all_d_b;
    // TODO save all_sf???
}


void LocalBundle::inputFeatureMatchesPoses( int seq_a, int seq_b,
    const vector<Matrix4d> all_a0_T_a, const vector<Matrix4d> all_b0_T_b )
{
    assert( all_a0_T_a.size() == all_b0_T_b.size() && all_a0_T_a.size() > 0 );

    auto p = std::make_pair( seq_a, seq_b );
    if( this->a0_T_a.count(p) !=0 || this->b0_T_b.count(p) != 0 ) {
        cout << TermColor::RED() << "[LocalBundle::inputFeatureMatchesPoses] ERROR this pair " << seq_a << "," << seq_b << " already exists\n" << TermColor::RESET();
        cout << "exit...\n";
        exit(2);
    }
    __LocalBundle__input(
    cout << "[LocalBundle::inputFeatureMatchesPoses] Set poses (a0_T_a and b0_T_b) for " << all_a0_T_a.size() << " image-pairs in seqa=" << seq_a << ", seqb=" << seq_b << endl; )
    a0_T_a[p] = all_a0_T_a;
    b0_T_b[p] = all_b0_T_b;

}


void LocalBundle::inputFeatureMatchesImIdx( int seq_a, int seq_b, vector< std::pair<int,int> > all_pair_idx )
{
    __LocalBundle__input(
    cout << "[LocalBundle::inputFeatureMatchesImIdx] setting feature matches image index between seq_a=" << seq_a << " and seq_b=" << seq_b << "\tall_pair_idx.size()=" << all_pair_idx.size() << endl; )
    auto p = std::make_pair( seq_a, seq_b );
    this->all_pair_idx[ p ] = all_pair_idx;
}

void LocalBundle::inputOdometryImIdx( int seqJ, vector<int> odom_seqJ_idx )
{
    __LocalBundle__input(
    cout << "[LocalBundle::inputOdometryImIdx] set " << odom_seqJ_idx.size() << " images-idx for seqJ=" << seqJ << endl; )
    seq_x_idx[ seqJ ] = odom_seqJ_idx;
}


void LocalBundle::inputSequenceImages( int seqJ, vector<cv::Mat> images_seq_j )
{
    __LocalBundle__input(
    cout << "[LocalBundle::inputSequenceImages] set " << images_seq_j.size() << " images for seqJ=" << seqJ << endl; )
    seq_x_images[ seqJ ] = images_seq_j;
}


//----------------------------------------------------------------------------//
//             END INPUT
//----------------------------------------------------------------------------//




//----------------------------------------------------------------------------//
//      print, json IO
//----------------------------------------------------------------------------//

json LocalBundle::odomSeqJ_toJSON( int j ) const
{
    json obj;
    obj["seqID"] = j;

    assert( x0_T_c.count(j) != 0 && seq_x_idx.count(j) != 0 );


    // save `x0_T_c[j]`
    // save `seq_x_idx[j]`
    obj["data"] = json();
    auto odom_poses = x0_T_c.at( j );
    auto odom_idx = seq_x_idx.at( j );


    assert( odom_poses.size() == odom_idx.size() && odom_poses.size() > 0 );
    for( int i=0 ; i<(int)odom_poses.size() ; i++ )
    {
        json tmp;
        tmp["c0_T_c"] = RawFileIO::write_eigen_matrix_tojson( odom_poses.at(i)  );
        tmp["idx"] = odom_idx.at(i);

        obj["data"].push_back( tmp );
    }

    return obj;
}

json LocalBundle::matches_SeqPair_toJSON( int seq_a, int seq_b ) const
{
    json obj;
    obj["seq_a"] = seq_a;
    obj["seq_b"] = seq_b;
    auto p = std::make_pair( seq_a, seq_b );
    assert( this->a0_T_b0.count(p) > 0 );


    // save `a0_T_b0`
    obj["initial_guess____a0_T_b0"] = RawFileIO::write_eigen_matrix_tojson( a0_T_b0.at(p) );
    obj["odom__a0_T_b0"] = RawFileIO::write_eigen_matrix_tojson( odom__a0_T_b0.at(p) );

    assert( this->normed_uv_a.count(p) > 0 );
    assert( this->d_a.count(p) > 0 );
    assert( this->a0_T_a.count(p) > 0 );

    assert( this->normed_uv_a.count(p) > 0 );
    assert( this->d_a.count(p) > 0 );
    assert( this->a0_T_a.count(p) > 0 );

    assert( this->all_pair_idx.count(p) > 0 );


    obj["data"] = json();

    int N_im_pairs = (int)this->a0_T_a.at(p).size();
    for( int im_pair_i = 0 ; im_pair_i < N_im_pairs ; im_pair_i++ )
    {
        json tmp;
        tmp["idx_a"] = all_pair_idx.at(p).at( im_pair_i ).first;
        tmp["idx_b"] = all_pair_idx.at(p).at( im_pair_i ).second;
        tmp["npoint_feature_matches"] = normed_uv_a.at(p).at( im_pair_i ).cols();


        assert( normed_uv_a.at(p).at( im_pair_i ).cols() == normed_uv_b.at(p).at( im_pair_i ).cols() );
        assert( d_a.at(p).at( im_pair_i ).size() == d_a.at(p).at( im_pair_i ).size() );
        // uv_a <---> uv_b
        tmp["normed_uv_a"] = RawFileIO::write_eigen_matrix_tojson(  normed_uv_a.at(p).at( im_pair_i ) );
        tmp["normed_uv_b"] = RawFileIO::write_eigen_matrix_tojson(  normed_uv_b.at(p).at( im_pair_i ) );

        // d_a <--> d_b
        tmp["d_a"] = RawFileIO::write_eigen_matrix_tojson(  d_a.at(p).at( im_pair_i ) );
        tmp["d_b"] = RawFileIO::write_eigen_matrix_tojson(  d_b.at(p).at( im_pair_i ) );


        // a0_T_a <---> b0_T_b
        tmp["a0_T_a"] = RawFileIO::write_eigen_matrix_tojson(  a0_T_a.at(p).at( im_pair_i ) );
        tmp["b0_T_b"] = RawFileIO::write_eigen_matrix_tojson(  b0_T_b.at(p).at( im_pair_i ) );


        obj["data"].push_back( tmp );
    }

    return obj;



}

void LocalBundle::toJSON(const string BASE) const
{
    // const string BASE = "/app/catkin_ws/src/gmm_pointcloud_align/resources/local_bundle/";

    // Save Sequences
    for( auto it = x0_T_c.begin() ; it != x0_T_c.end() ; it++ )
    {
        json tmp_i = odomSeqJ_toJSON( it->first );

        string fname = BASE+ "/odomSeq" + to_string(it->first) + ".json";
        cout << TermColor::iGREEN() << "Open File: " << fname << TermColor::RESET() << endl;
        std::ofstream o(fname);
        o << std::setw(4) << tmp_i << std::endl;
    }


    // Save matches
    for( auto it = all_pair_idx.begin() ; it != all_pair_idx.end() ; it++ )
    {
        auto p = it->first;
        json tmp_p = matches_SeqPair_toJSON( p.first, p.second );

        string fname = BASE+ "/seqPair_" + to_string(p.first) + "_" + to_string(p.second) + ".json";
        cout << TermColor::iGREEN() << "Open File: " << fname << TermColor::RESET() << endl;
        std::ofstream o(fname);
        o << std::setw(4) << tmp_p << std::endl;
    }


    // save IMAGES if available
    for( auto it = seq_x_images.begin() ; it!=seq_x_images.end() ; it++ )
    {
        cout << "seq_x_images[" << it->first << "] has " << (it->second).size() << " images\n";
        int seqJ = it->first ;

        for( auto it_im= it->second.begin() ; it_im != it->second.end() ; it_im++ )
        {
            int im_i = std::distance( it->second.begin(), it_im );
            string fname = BASE+"/odomSeq"+std::to_string( seqJ ) +"_im_" + to_string( im_i ) + ".jpg";
            cv::imwrite( fname, *it_im );

            cout << "\t#" << im_i << ": ";
            cout << MiscUtils::cvmat_info( *it_im ) << "\n\t";
            cout << "imwrite(" << fname << ")" ;
            cout << endl;
        }
    }

}

//---

bool LocalBundle::odomSeqJ_fromJSON( const string BASE, int j)
{
    // const string BASE = "/app/catkin_ws/src/gmm_pointcloud_align/resources/local_bundle/";
    // int j = 0;

    string fname = BASE + "/odomSeq" + std::to_string( j ) + ".json";
    cout << "Load: " << fname << endl;

    // read json
    std::ifstream json_file(fname);
    json obj;
    json_file >> obj;


    int seqID = obj["seqID"];
    cout << "seqID = " << seqID << endl;
    assert( seqID == j );

    int N = (int)obj["data"].size();
    cout << "There are " << N << " seq items\n";
    this->x0_T_c[seqID] = vector<Matrix4d>();
    this->seq_x_idx[seqID] = vector<int>();
    for( int i=0 ; i<N ; i++ )
    {
        json tmp_pose = obj["data"][i]["c0_T_c"];
        Matrix4d c0_T_c;
        RawFileIO::read_eigen_matrix4d_fromjson( tmp_pose, c0_T_c );

        int idx = obj["data"][i]["idx"];
        #if 0
        cout << idx << "\t";
        cout << PoseManipUtils::prettyprintMatrix4d( c0_T_c ) << endl;
        cout << c0_T_c << endl;
        #endif

        // set this data in `this`
        this->x0_T_c[seqID].push_back( c0_T_c );
        this->seq_x_idx[seqID].push_back( idx );

    }

    return true;
}

bool LocalBundle::matches_SeqPair_fromJSON( const string BASE, int seqa, int seqb )
{
    // const string BASE = "/app/catkin_ws/src/gmm_pointcloud_align/resources/local_bundle/";
    // int seqa = 0;
    // int seqb = 1;

    string fname = BASE + "/seqPair_" + std::to_string( seqa ) + "_" + std::to_string( seqb ) + ".json";
    cout << "Load: " << fname << endl;

    // read json
    std::ifstream json_file(fname);
    json obj;
    json_file >> obj;

    int json_seqa = obj["seq_a"];
    int json_seqb = obj["seq_b"];
    assert( seqa == json_seqa && seqb == json_seqb );
    auto pyp = std::make_pair( seqa, seqb );
    cout << "json_seqa=" << json_seqa<< "\tjson_seqb=" << json_seqb << endl;


    json tmp = obj["initial_guess____a0_T_b0"];
    a0_T_b0[pyp] = Matrix4d::Identity();
    RawFileIO::read_eigen_matrix4d_fromjson( tmp, a0_T_b0[pyp] );
    cout << "a0_T_b0[(" << pyp.first << "," << pyp.second  << ")]=" << a0_T_b0[pyp] << endl;


    json tmp2 = obj["odom__a0_T_b0"];
    odom__a0_T_b0[pyp] = Matrix4d::Identity();
    RawFileIO::read_eigen_matrix4d_fromjson( tmp2, odom__a0_T_b0[pyp] );
    cout << "odom__a0_T_b0[(" << pyp.first << "," << pyp.second  << ")]=" << odom__a0_T_b0[pyp] << endl;


    //
    int N = (int)obj["data"].size();
    cout << "There are " << N << "data items\n";
    normed_uv_a[pyp] =  vector<MatrixXd> ();
    d_a[pyp] = vector<VectorXd> ();
    a0_T_a[pyp] = vector<Matrix4d>();

    normed_uv_b[pyp] =  vector<MatrixXd> ();
    d_b[pyp] = vector<VectorXd> ();
    b0_T_b[pyp] = vector<Matrix4d>();

    all_pair_idx[pyp] =  vector< std::pair<int,int> >();

    for( int i=0 ; i<N ; i++ )
    {
        json tmp_data = obj["data"][i];

        Matrix4d tmp_a0_T_a;
        RawFileIO::read_eigen_matrix4d_fromjson( tmp_data["a0_T_a"], tmp_a0_T_a );

        Matrix4d tmp_b0_T_b;
        RawFileIO::read_eigen_matrix4d_fromjson( tmp_data["b0_T_b"], tmp_b0_T_b );

        int idx_a = tmp_data["idx_a"];
        int idx_b = tmp_data["idx_b"];


        VectorXd tmp_d_a;
        RawFileIO::read_eigen_vector_fromjson( tmp_data["d_a"], tmp_d_a );

        VectorXd tmp_d_b;
        RawFileIO::read_eigen_vector_fromjson( tmp_data["d_b"], tmp_d_b );


        MatrixXd tmp_normed_uv_a;
        RawFileIO::read_eigen_matrix_fromjson( tmp_data["normed_uv_a"], tmp_normed_uv_a );

        MatrixXd tmp_normed_uv_b;
        RawFileIO::read_eigen_matrix_fromjson( tmp_data["normed_uv_b"], tmp_normed_uv_b );


        cout << "i=" << i << "\t";
        cout << idx_a << "<--->" << idx_b << "\t";
        cout << "d_a, d_b = " << tmp_d_a.size() << ", " << tmp_d_b.size() << "\t";
        cout << "uv_a=" << tmp_normed_uv_a.rows() << "x" << tmp_normed_uv_a.cols() << "\t";
        cout << "uv_b=" << tmp_normed_uv_a.rows() << "x" << tmp_normed_uv_a.cols() << "\t";
        cout << endl;


        // set this data in `this`
        all_pair_idx[pyp].push_back( make_pair( idx_a, idx_b) );

        normed_uv_a[pyp].push_back( tmp_normed_uv_a );
        d_a[pyp].push_back( tmp_d_a );
        a0_T_a[pyp].push_back( tmp_a0_T_a );

        normed_uv_b[pyp].push_back( tmp_normed_uv_b );
        d_b[pyp].push_back( tmp_d_b );
        b0_T_b[pyp].push_back( tmp_b0_T_b );


    }
    return true;

}

void LocalBundle::fromJSON( const string BASE )
{
    // TODO ideally should scan this directory, but itz ok.
    odomSeqJ_fromJSON(BASE, 0);
    odomSeqJ_fromJSON(BASE, 1);
    matches_SeqPair_fromJSON(BASE, 0, 1);


    // Load Images (if available)
    #if 1
    cout << "[LocalBundle::fromJSON] Load Images if available\n";
    for( auto it = x0_T_c.begin() ; it != x0_T_c.end() ; it++ )
    {
        int seqJ = it->first;
        int n_images_in_seqJ = it->second.size();
        cout << "\tLoad Images for seqJ=" << seqJ << ", I will try to load " << n_images_in_seqJ << " images\n";

        bool status = true;
        for( int im_i=0 ; im_i<n_images_in_seqJ ; im_i++ )
        {
            string fname = BASE+"/odomSeq"+std::to_string( seqJ ) +"_im_" + to_string( im_i ) + ".jpg";
            bool is_file_exist = RawFileIO::if_file_exist_2( fname );
            if( is_file_exist ) {
                // cout << "\t\tExist: " << TermColor::GREEN() << fname << TermColor::RESET() << endl;
            }
            else {
                cout << "\t\tDoesn't Exist: " << TermColor::RED() << fname << TermColor::RESET() << endl;
                status = false;
            }

        }


        if( status == false )
            cout << "[LocalBundle::fromJSON] Cannot load images for seqJ=" << seqJ << " because, I cannot see the image files\n";



        // Now that it is confirmed, all images exisit, load those
        seq_x_images[ seqJ ] = vector<cv::Mat>();
        for( int im_i=0 ; im_i<n_images_in_seqJ ; im_i++ )
        {
            string fname = BASE+"/odomSeq"+std::to_string( seqJ ) +"_im_" + to_string( im_i ) + ".jpg";
            cout << "\t\timread : " << fname << endl;
            cv::Mat im = cv::imread( fname , 0 );
            if (im.empty())
            {
                std::cout << "[LocalBundle::fromJSON] !!! Failed imread(): image not found" << std::endl;
                exit(3);
            }

            seq_x_images[ seqJ ].push_back( im );

        }

    }
    #endif
}
//---

void LocalBundle::print_inputs_info() const
{
    cout << TermColor::GREEN() << "-----------------------------------\n---LocalBundle::print_inputs_info---\n---------------------------------------\n";
    cout << "There are " << x0_T_c.size() << " sequences\n";
    vector<char> char_list = { 'a', 'b', 'c', 'd', 'e', 'f' };

    int i=0;
    for( auto it=x0_T_c.begin() ; it!=x0_T_c.end() ; it++ )
    {
        cout << "Seq#" << it->first  << ": (n_items=" << it->second.size() << ")";

        if( seq_x_images.count(it->first) == 0 )
            cout << "(n_images=0)";
        else
            cout << "(n_images=" << seq_x_images.at( it->first ).size() << ")";

        cout << "\n";
        cout << char_list[i] << 0 << "  , " << char_list[i] << 1 << " ... " << char_list[i] << it->second.size()-1 << endl;
        cout << *( seq_x_idx.at( it->first ).begin() ) << "," << *( seq_x_idx.at( it->first ).begin()+1 ) << " ... " << *( seq_x_idx.at( it->first ).rbegin() ) << endl;
        cout << endl;
        i++;
    }


    #if 0
    // too much details
    for( auto it=seq_x_idx.begin() ; it!=seq_x_idx.end() ; it++ )
    {
        auto Y = it->second;
        cout << "\nSeq#" << it->first << ": ";
        for( auto ity = Y.begin() ; ity!= Y.end() ; ity++ )
            cout << *ity << "\t";
        cout << endl;

    }
    #endif


    cout << "There are " << all_pair_idx.size() << " seq-pairs\n";
    for( auto it = all_pair_idx.begin() ; it != all_pair_idx.end() ; it++ )
    {
        cout << "seq-pair#" << std::distance( it , all_pair_idx.begin() ) << " between seq#" << it->first.first << " and seq#" << it->first.second << endl; ;

        #if 0
        auto y = it->second;
        for( auto itz=y.begin() ; itz!=y.end() ; itz++ )
            cout << itz->first << "<--->" << itz->second << "\n";
        cout << endl;
        #endif

        auto y = it->second;
        auto pyp = it->first;
        for( int k=0 ; k<(int)y.size() ; k++ ) {
            cout << "\t#" << k << "\t";
            cout << all_pair_idx.at(pyp).at( k ).first << "<--->" << all_pair_idx.at(pyp).at( k ).second<< "\t";
            cout << normed_uv_a.at(pyp).at( k ).rows() << "x" << normed_uv_a.at(pyp).at( k ).cols() << "\t";
            cout << normed_uv_b.at(pyp).at( k ).rows() << "x" << normed_uv_b.at(pyp).at( k ).cols() << "\t";
            cout << d_a.at(pyp).at(k).size() << "\t" <<  d_b.at(pyp).at(k).size() << "\t";
            a0_T_a.at(pyp).at(k);
            b0_T_b.at(pyp).at(k);

            cout << endl;
        }


    }

    cout << TermColor::GREEN() << "-----------------------------------\n---END LocalBundle::print_inputs_info---\n---------------------------------------\n" << TermColor::RESET();
}



//----------------------------------------------------------------------------//
//      END print, json IO
//----------------------------------------------------------------------------//

void LocalBundle::deallocate_optimization_vars()
{
    assert( m_opt_vars_allocated == true );
    for( auto it = opt_var_xyz.begin() ; it!=opt_var_xyz.end() ; it++ ) {
        delete [] opt_var_xyz[ it->first ];
        delete [] opt_var_qxqyqzqw[ it->first ];
    }
    m_opt_vars_allocated = false;
}


double * LocalBundle::get_raw_ptr_to_opt_variable_q(int seqID, int u ) const
{
    assert( opt_var_qxqyqzqw.count(seqID) > 0 );
    assert( u>=0 && u<x0_T_c.at(seqID).size() );

    return &( opt_var_qxqyqzqw.at( seqID )[ 4*u ] );
}


double * LocalBundle::get_raw_ptr_to_opt_variable_t(int seqID, int u ) const
{
    assert( opt_var_xyz.count(seqID) > 0 );
    assert( u>=0 && u<x0_T_c.at(seqID).size() );

    return &( opt_var_xyz.at( seqID )[ 3*u ] );
}

#define __LocalBundle__allocate_and_init_optimization_vars( msg ) msg;
// #define __LocalBundle__allocate_and_init_optimization_vars( msg ) ;
void LocalBundle::allocate_and_init_optimization_vars()
{
    assert( m_opt_vars_allocated == false );
    __LocalBundle__allocate_and_init_optimization_vars(
    cout << TermColor::iGREEN() << "allocate_and_init_optimization_vars" << TermColor::RESET() << endl;
    cout << "Pose of each frame is an optimization variable.\n"; )
    for( auto it = x0_T_c.begin() ; it != x0_T_c.end() ; it++ ) // loop over every series
    {
        int seqID = it->first;
        int n_frame_in_this_series = (it->second).size();
        __LocalBundle__allocate_and_init_optimization_vars(
        cout << "n_frame_in_this_series(seqID=" << seqID << ") = " << n_frame_in_this_series << endl;
        )

        double * S_qxqyqzqw = new double[n_frame_in_this_series*4];
        double * S_xyz = new double[n_frame_in_this_series*3];

        opt_var_qxqyqzqw[ seqID ] = S_qxqyqzqw;
        opt_var_xyz[ seqID ] = S_xyz;


        // set values from it->second
        auto pose_list = it->second;
        for( int i=0 ; i< (int)pose_list.size() ; i++ )
        {
            Matrix4d use;
            if( seqID == 0 )
                use = pose_list[i];
            else if(seqID == 1 )
            {
                // use the initial guess to bring the seq-a and seq-b to same co-ordinate frame
                assert( a0_T_b0.count(std::make_pair(0,1)) > 0 );
                use = a0_T_b0.at( std::make_pair(0,1) ) * pose_list[i];
            } else {
                cout << "NOT IMPLEMENTED....\n";
                exit(1);
            }

            PoseManipUtils::eigenmat_to_raw_xyzw( use, &S_qxqyqzqw[4*i], &S_xyz[3*i] );

            #if 0
            // verification
            cout << "\t#" << i << " ";
            cout << "pose_list[i] : " << PoseManipUtils::prettyprintMatrix4d( pose_list[i] ) << "\t";
            cout << "use : " << PoseManipUtils::prettyprintMatrix4d(use) << endl;

            Matrix4d tmp_w_T_i;
            PoseManipUtils::raw_xyzw_to_eigenmat( get_raw_ptr_to_opt_variable_q(seqID, i), get_raw_ptr_to_opt_variable_t(seqID, i), tmp_w_T_i );
            cout << "\t\ttmp_w_T_i: " << PoseManipUtils::prettyprintMatrix4d( tmp_w_T_i ) << endl;
            #endif

        }

    }
    m_opt_vars_allocated = true;
    __LocalBundle__allocate_and_init_optimization_vars(
    cout << TermColor::iGREEN() << "END allocate_and_init_optimization_vars" << TermColor::RESET() << endl;
    )
}

#define __LocalBundle__set_params_constant_for_seq( msg ) msg;
// #define __LocalBundle__set_params_constant_for_seq( msg ) ;
void LocalBundle::set_params_constant_for_seq( int seqID, ceres::Problem& problem )
{
    assert( m_opt_vars_allocated == true );
    assert( x0_T_c.count(seqID) > 0 );
    __LocalBundle__set_params_constant_for_seq(
    cout << TermColor::YELLOW() << "[LocalBundle::set_params_constant_for_seq]Set nodes with seqID=" << seqID << " as constant. There are " << x0_T_c.at( seqID ).size() << " such nodes\n" << TermColor::RESET();
    )

    for( int i=0 ; i<(int)x0_T_c.at( seqID ).size() ; i++ )
    {
        problem.SetParameterBlockConstant( get_raw_ptr_to_opt_variable_q(seqID, i)   );
        problem.SetParameterBlockConstant( get_raw_ptr_to_opt_variable_t(seqID, i)   );
    }
}


#define __LocalBundle__set_params_constant_for_seq( msg ) msg;
// #define __LocalBundle__set_params_constant_for_seq( msg ) ;

// #define __LocalBundle__set_params_constant_for_seq__debug( msg ) msg;
#define __LocalBundle__set_params_constant_for_seq__debug( msg ) ;
void LocalBundle::add_odometry_residues( ceres::Problem& problem )
{
    assert( m_opt_vars_allocated );
    __LocalBundle__set_params_constant_for_seq(
    cout << TermColor::iGREEN() << "odometry residues" << TermColor::RESET() << endl; )
    for( auto it = x0_T_c.begin() ; it != x0_T_c.end() ; it++ ) // loop over every series
    {
        int seqID = it->first;
        auto pose_list = it->second;
        int n_frame_in_this_series = pose_list.size();
        __LocalBundle__set_params_constant_for_seq(
        cout << "n_frame_in_this_series( this=" << it->first << ") = " << n_frame_in_this_series << endl; )

        // add u<-->u-1, add u<-->u-2, add u<-->u-3, add u<-->u-4
        for( int u=0 ; u<(int)pose_list.size() ; u++ )
        {
            for( int f=1 ; f<=4 ; f++ )
            {
                if( u-f < 0 )
                    continue;

                __LocalBundle__set_params_constant_for_seq__debug(
                cout  << u << "=" << u-f << ","; )

                Matrix4d u_M_umf = pose_list[u].inverse() * pose_list[u-f];
                double odom_edge_weight = 1.0;
                ceres::CostFunction * cost_function = SixDOFError::Create( u_M_umf, odom_edge_weight  );
                problem.AddResidualBlock( cost_function, NULL,
                        get_raw_ptr_to_opt_variable_q(seqID, u), get_raw_ptr_to_opt_variable_t(seqID, u),
                        get_raw_ptr_to_opt_variable_q(seqID, u-f), get_raw_ptr_to_opt_variable_t(seqID, u-f) );


                #if 0
                // verification
                cout << endl;
                cout << "u_M_umf:" << PoseManipUtils::prettyprintMatrix4d( u_M_umf ) << "\n";
                Matrix4d tmp_w_T_u, tmp_w_T_umf;
                PoseManipUtils::raw_xyzw_to_eigenmat( get_raw_ptr_to_opt_variable_q(seqID, u), get_raw_ptr_to_opt_variable_t(seqID, u), tmp_w_T_u );
                PoseManipUtils::raw_xyzw_to_eigenmat( get_raw_ptr_to_opt_variable_q(seqID, u-f), get_raw_ptr_to_opt_variable_t(seqID, u-f), tmp_w_T_umf );
                Matrix4d tmp_u_T_umf = tmp_w_T_u.inverse() * tmp_w_T_umf;
                cout << "tmp_u_T_umf:" << PoseManipUtils::prettyprintMatrix4d( tmp_u_T_umf ) << "\t";
                cout << endl;
                #endif

            }
            __LocalBundle__set_params_constant_for_seq__debug(
            cout << "\t"; )
        }
        __LocalBundle__set_params_constant_for_seq__debug(
        cout << endl; )

    }
    __LocalBundle__set_params_constant_for_seq(
    cout << TermColor::iGREEN() << "END odometry residues" << TermColor::RESET() << endl; )

}



#define __LocalBundle__add_correspondence_residues( msg ) msg;
// #define __LocalBundle__add_correspondence_residues( msg ) ;
void LocalBundle::add_correspondence_residues( ceres::Problem& problem )
{
    assert( m_opt_vars_allocated );

    auto p = std::make_pair( 0, 1 );
    __LocalBundle__add_correspondence_residues(
    cout << TermColor::iGREEN() << "correspondence residues" << TermColor::RESET() << endl;
    cout << "p = (" << p.first << "," << p.second << ")\n";
    cout << "normed_uv_a[p].size()=" << normed_uv_a[p].size() << endl; )
    auto robust_loss = new CauchyLoss(.01) ;

    for( int i=0 ; i<(int)normed_uv_a[p].size() ; i++ )
    {
            __LocalBundle__add_correspondence_residues(
        cout << "---frame-pair#" << i << "\t"; )
        // cout << "\n";
        #if 0
        cout << "\t" << "normed_uv_a[p][i].cols()=" << normed_uv_a[p][i].cols() << endl;
        cout << "\t" << "d_a[p][i].cols()=" << d_a[p][i].rows() << endl;
        cout << "\t" << all_pair_idx[p][i].first << " <> " << all_pair_idx[p][i].second << endl; //frame# of the correspondence
        #endif

        auto f_it = std::find( seq_x_idx.at(0).begin(), seq_x_idx.at(0).end(), all_pair_idx[p][i].first );
        auto h_it = std::find( seq_x_idx.at(1).begin(), seq_x_idx.at(1).end(), all_pair_idx[p][i].second );

        if( f_it == seq_x_idx.at(0).end() || h_it == seq_x_idx.at(1).end() ) {
            cout << "\t[LocalBundle::add_correspondence_residues]NA\n";
            assert( false );
            exit(4);
        }
        else {

        }

        int ff = std::distance( seq_x_idx.at(0).begin(), f_it );
        int hh = std::distance( seq_x_idx.at(1).begin(), h_it );
        __LocalBundle__add_correspondence_residues(
        cout << "\t#pts=" <<  normed_uv_a[p][i].cols() << " ";
        cout << "\tframe_a#" << ff << " <> " << "frame_b#" << hh << "\t"; )
        double * tmp_ff_q = get_raw_ptr_to_opt_variable_q(0, ff );
        double * tmp_ff_t = get_raw_ptr_to_opt_variable_t(0, ff );
        double * tmp_hh_q = get_raw_ptr_to_opt_variable_q(1, hh);
        double * tmp_hh_t = get_raw_ptr_to_opt_variable_t(1, hh );


        int n_good_depths = 0;
        for( int j=0 ; j<normed_uv_a[p][i].cols() ; j++ )
        {
            Vector4d a_3d, b_3d;
            Vector3d a_2d, b_2d;
            double ___d_a = d_a[p][i](j);
            double ___d_b = d_b[p][i](j);
            if( ___d_a < 0.5 || ___d_a > 5.0 || ___d_b < 0.5 || ___d_b > 5.0 )
                continue;

            if( n_good_depths > 500 ) // dont add more than 500 terms per image-pair.
                break;

            a_3d << ___d_a * normed_uv_a[p][i].col(j).topRows(3), 1.0;
            b_3d << ___d_b * normed_uv_b[p][i].col(j).topRows(3), 1.0;
            a_2d << normed_uv_a[p][i].col(j).topRows(3);
            b_2d << normed_uv_b[p][i].col(j).topRows(3);

            // 3d points from a, 2d points from b
            ceres::CostFunction * cost_function_1 = ProjectionError::Create( a_3d, b_2d  );
            problem.AddResidualBlock( cost_function_1, robust_loss, tmp_ff_q, tmp_ff_t, tmp_hh_q, tmp_hh_t );


            // 3d point from b, 2d point from a
            ceres::CostFunction * cost_function_2 = ProjectionError::Create( b_3d, a_2d  );
            problem.AddResidualBlock( cost_function_2, robust_loss, tmp_hh_q, tmp_hh_t, tmp_ff_q, tmp_ff_t );
            n_good_depths++;

        }
        __LocalBundle__add_correspondence_residues(
        cout << "\t"<< n_good_depths << " correspondence residues added\n"; )
    }
    __LocalBundle__add_correspondence_residues(
    cout << TermColor::iGREEN() << "END correspondence residues" << TermColor::RESET() << endl; )
}

#if 0 //TODO removal, donest work as expected,
void LocalBundle::add_batched_correspondence_residues( ceres::Problem& problem )
{
    cout << TermColor::iGREEN() << "batched correspondence residues\n" << TermColor::RESET();
    auto p = std::make_pair(0,1);
    cout << "p = (" << p.first << "," << p.second << ")\n";
    cout << "normed_uv_a[p].size()=" << normed_uv_a[p].size() << endl;
    auto robust_loss = new CauchyLoss(.01) ;

    for( int i=0 ; i<normed_uv_a[p].size() ; i++ ) // loop over frames-pair
    {
        cout << "---\n";
        cout << "\t" << "normed_uv_a[p][i].cols()=" << normed_uv_a[p][i].cols() << endl;
        cout << "\t" << "d_a[p][i].cols()=" << d_a[p][i].rows() << endl;
        cout << "\t" << all_pair_idx[p][i].first << " <> " << all_pair_idx[p][i].second << endl; //frame# of the correspondence

        auto f_it = std::find( seq_x_idx.at(0).begin(), seq_x_idx.at(0).end(), all_pair_idx[p][i].first );
        auto h_it = std::find( seq_x_idx.at(1).begin(), seq_x_idx.at(1).end(), all_pair_idx[p][i].second );

        if( f_it == seq_x_idx.at(0).end() || h_it == seq_x_idx.at(1).end() ) {
            cout << "\tNA\n";
            assert( false );
        }
        else {
            cout << "\t";
            cout << std::distance( seq_x_idx.at(0).begin(), f_it );
            cout << " <> ";
            cout << std::distance( seq_x_idx.at(1).begin(), h_it ) << endl;
        }

        int ff = std::distance( seq_x_idx.at(0).begin(), f_it );
        int hh = std::distance( seq_x_idx.at(1).begin(), h_it );
        cout << "\tframe_a#" << ff << " <> " << "frame_b#" << hh << endl;
        double * tmp_ff_q = get_raw_ptr_to_opt_variable_q(0, ff );
        double * tmp_ff_t = get_raw_ptr_to_opt_variable_t(0, ff );
        double * tmp_hh_q = get_raw_ptr_to_opt_variable_q(1, hh);
        double * tmp_hh_t = get_raw_ptr_to_opt_variable_t(1, hh );

        MatrixXd a_3d, b_3d;
        MatrixXd a_2d, b_2d;
        a_2d = normed_uv_a[p][i];
        b_2d = normed_uv_b[p][i];
        VectorXd _da = d_a[p][i];
        VectorXd _db = d_b[p][i];

        cout << "\t";
        cout << "a_2d: " << a_2d.rows() << "x" << a_2d.cols() << "\t";
        cout << "b_2d: " << b_2d.rows() << "x" << b_2d.cols() << "\t";
        cout << "_da : " << _da.size() << "\t";
        cout << "_db : " << _db.size() << "\t";


        a_3d = MatrixXd::Constant( 4, a_2d.cols(), 1.0 );
        a_3d.row(0) = a_2d.row(0).array() * _da.transpose().array();
        a_3d.row(1) = a_2d.row(1).array() * _da.transpose().array();
        a_3d.row(2) = a_2d.row(2).array() * _da.transpose().array();
        // a_3d = a_2d.rowwise() * d_a[p][i];
        cout << "a_3d: " << a_3d.rows() << "x" << a_3d.cols() << "\t";


        b_3d = MatrixXd::Constant( 4, b_2d.cols(), 1.0 );
        b_3d.row(0) = b_2d.row(0).array() * _db.transpose().array();
        b_3d.row(1) = b_2d.row(1).array() * _db.transpose().array();
        b_3d.row(2) = b_2d.row(2).array() * _db.transpose().array();
        cout << "b_3d: " << b_3d.rows() << "x" << b_3d.cols() << "\t";

        cout << endl;
        cout << "a_2d\n" << a_2d.leftCols(10) << endl;
        cout << "a_3d\n" << a_3d.leftCols(10) << endl;
        cout << "_da:" << _da.topRows(10).transpose() << endl;

        cout << endl;
        cout << "b_2d\n" << b_2d.leftCols(10) << endl;
        cout << "b_3d\n" << b_3d.leftCols(10) << endl;
        cout << "_db:" << _db.topRows(10).transpose() << endl;


        // make weights
        VectorXd weight_a = VectorXd::Zero( a_2d.cols() );
        VectorXd weight_b = VectorXd::Zero( b_2d.cols() );
        // TODO: set appropriate weights based on depth

        for( int qq=0 ; qq<weight_a.size() ; qq++ )
        {
            if( _da(qq) > 0.5 && _da(qq) < 5.0 )
                weight_a(qq) = 1.0;
            else
                weight_a(qq) = 0.0;

            if( _db(qq) > 0.5 && _db(qq) < 5.0 )
                weight_b(qq) = 1.0;
            else
                weight_b(qq) = 0.0;
        }


        // 3d points from a, 2d points from b
        ceres::CostFunction * cost_function_1 = BatchProjectionError::Create( a_3d, b_2d, weight_a, 1.0 );
        problem.AddResidualBlock( cost_function_1, robust_loss, tmp_ff_q, tmp_ff_t, tmp_hh_q, tmp_hh_t );


    }



    cout << TermColor::iGREEN() << "END batched correspondence residues\n" << TermColor::RESET();

}

#endif

void LocalBundle::solve()
{
    ceres::Problem problem;

    // setup optization variable's initial guess
    //      7 optimization variables for every frame
    allocate_and_init_optimization_vars();


    // add parameter blocks to the ceres::Problem
        // If you are using Eigen’s Quaternion object, whose layout is x,y,z,w, then you should use EigenQuaternionParameterization.
    ceres::LocalParameterization * eigenquaternion_parameterization = new ceres::EigenQuaternionParameterization;
    for( auto it = x0_T_c.begin() ; it != x0_T_c.end() ; it++ )
    {
        int seqID = it->first;
        auto pose_list = it->second;
        for( int u=0 ; u<(int)pose_list.size() ; u++ )
        {
            problem.AddParameterBlock( get_raw_ptr_to_opt_variable_q(seqID, u), 4 );
            problem.SetParameterization( get_raw_ptr_to_opt_variable_q(seqID, u),  eigenquaternion_parameterization );
            problem.AddParameterBlock( get_raw_ptr_to_opt_variable_t(seqID, u), 3 );

        }
    }
    set_params_constant_for_seq( 0, problem );


    // Setup residues - odometry
    add_odometry_residues( problem );

    #if 1
    // set residues - correspondence
    add_correspondence_residues( problem );
    #endif


    #if 0
    // the for loop in the ceres kills this. zThis is too slow, best avoided
    add_batched_correspondence_residues(problem)
    #endif


    // solve
    ElapsedTime t_solve( "LocalBundle::Solve");
    cout << TermColor::iGREEN() << "||||||SOLVE||||||" << TermColor::RESET() << endl;
    ceres::Solver::Options reint_options;
    ceres::Solver::Summary reint_summary;
    // reint_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    reint_options.linear_solver_type = ceres::DENSE_QR;
    reint_options.minimizer_progress_to_stdout = true;
    reint_options.max_num_iterations = 50;
    // reint_options.enable_fast_removal = true;
    ceres::Solve( reint_options, &problem, &reint_summary );
    cout << reint_summary.BriefReport() << endl;
    cout << TermColor::iGREEN() << "||||||END SOLVE||||||" << TermColor::RESET() << endl;


    // look at the solution a0_T_b0
       {
       Matrix4d optz__w_T_a0, optz__w_T_b0;
       PoseManipUtils::raw_xyzw_to_eigenmat( get_raw_ptr_to_opt_variable_q(0,0), get_raw_ptr_to_opt_variable_t(0,0), optz__w_T_a0 );
       PoseManipUtils::raw_xyzw_to_eigenmat( get_raw_ptr_to_opt_variable_q(1,0), get_raw_ptr_to_opt_variable_t(1,0), optz__w_T_b0 );
       Matrix4d optz__a0_T_b0 = optz__w_T_a0.inverse() * optz__w_T_b0;
       assert( a0_T_b0.count( std::make_pair(0,1) ) > 0 );
       cout << t_solve.toc() << endl;
       cout << "initial       : " << PoseManipUtils::prettyprintMatrix4d( a0_T_b0.at( std::make_pair(0,1) ) ) << endl;
       cout << "optz__a0_T_b0 : " << PoseManipUtils::prettyprintMatrix4d( optz__a0_T_b0 ) << endl;
       cout << "optz__a0_T_b0 :\n" << optz__a0_T_b0 << endl;
       }


}



//------------------------------
Matrix4d LocalBundle::retrive_optimized_pose( int seqID_0, int frame_0, int seqID_1, int frame_1 ) const
{
    assert( m_opt_vars_allocated );
    assert( frame_0 >=0 && frame_0 < x0_T_c.at(seqID_0).size() );
    assert( frame_1 >=0 && frame_1 < x0_T_c.at(seqID_1).size() );
    assert( a0_T_b0.count( std::make_pair(0,1) ) > 0 );

    // retrive solution
    Matrix4d optz__w_T_ai, optz__w_T_bj;
    PoseManipUtils::raw_xyzw_to_eigenmat( get_raw_ptr_to_opt_variable_q(seqID_0,frame_0), get_raw_ptr_to_opt_variable_t(seqID_0,frame_0), optz__w_T_ai );
    PoseManipUtils::raw_xyzw_to_eigenmat( get_raw_ptr_to_opt_variable_q(seqID_1,frame_1), get_raw_ptr_to_opt_variable_t(seqID_1,frame_1), optz__w_T_bj );
    Matrix4d optz__ai_T_bj = optz__w_T_ai.inverse() * optz__w_T_bj;
    // cout << "initial       : " << PoseManipUtils::prettyprintMatrix4d( a0_T_b0.at( std::make_pair(0,1) ) ) << endl;
    // cout << "optz__a0_T_b0 : " << PoseManipUtils::prettyprintMatrix4d( optz__a0_T_b0 ) << endl;

    return optz__ai_T_bj;


}

void LocalBundle::reprojection_test( const camodocal::CameraPtr camera )
{
    int seq_a = 0;
    int seq_b = 1;
    // int im_pair_idx = 3;
    auto pyp = std::make_pair( seq_a, seq_b );

    int n_im_pairs = (int) all_pair_idx.at( pyp ).size();
    cout << "# of image pairs = " << n_im_pairs << endl;

    for( int h=0 ; h<n_im_pairs ; h++ ) {
        reprojection_test_for_this_image_pair( camera, h );
        reprojection_error_for_this_image_pair( camera, h );

        char key = cv::waitKey(0);

        if( key == 27 )
            break;

    }

    cv::destroyWindow( "observed point feat matching");
    cv::destroyWindow( "plot( im_bj, uv_b )");
    cv::destroyWindow( "plot( im_ai, uv_a )");
}


void LocalBundle::reprojection_test_for_this_image_pair( const camodocal::CameraPtr camera, int im_pair_idx )
{
    cout << TermColor::iCYAN() << "=====LocalBundle::reprojection_test_for_this_image_pair=====" << im_pair_idx << TermColor::RESET() << endl;
    int seq_a = 0;
    int seq_b = 1;
    // int im_pair_idx = 3;
    auto pyp = std::make_pair( seq_a, seq_b );
    int n_im_pairs = (int) all_pair_idx.at( pyp ).size();
    if( im_pair_idx < 0 || im_pair_idx >= n_im_pairs ) {
        cout << __FILE__ << ":" << __LINE__ << "[LocalBundle::reprojection_error_for_this_image_pair]invalid im_pair_idx=" << im_pair_idx << " has to be between 0 and " <<   n_im_pairs << endl;
        exit(2);
    }


    // randomly picking the 3rd image pair
    int ai = all_pair_idx.at( pyp ).at( im_pair_idx ).first;
    int bj = all_pair_idx.at( pyp ).at( im_pair_idx ).second;
    cout << "ai="<< ai << "\tbj=" << bj << "\t";

    int local_ai = std::distance( seq_x_idx.at( seq_a ).begin(),     std::find( seq_x_idx.at( seq_a).begin(), seq_x_idx.at( seq_a).end(), ai ) );
    int local_bj = std::distance( seq_x_idx.at( seq_b ).begin(),     std::find( seq_x_idx.at( seq_b).begin(), seq_x_idx.at( seq_b).end(), bj ) );
    cout << "local_ai=" << local_ai << "\tlocal_bj=" << local_bj << endl;

    cv::Mat im_ai, im_bj;
    MatrixXd uv_a_observed, uv_b_observed;

    // plot( im_ai, uv_ai | im_bj, uv_bj ) : The point feature matches
    {
        im_ai = seq_x_images.at( seq_a ).at( local_ai );
        im_bj = seq_x_images.at( seq_b ).at( local_bj );
        if( im_ai.empty() || im_bj.empty() ) {
            cout << "PPP@ Cannot load im_ai or im_bj\n";
            exit(3);
        }

        uv_a_observed = StaticPointFeatureMatching::normalized_image_cordinates_to_image_coordinates( camera, normed_uv_a.at(pyp).at(im_pair_idx) );
        uv_b_observed = StaticPointFeatureMatching::normalized_image_cordinates_to_image_coordinates( camera, normed_uv_b.at(pyp).at(im_pair_idx) );

        cv::Mat dst_p;
        MiscUtils::plot_point_pair( im_ai, uv_a_observed, local_ai,
                                    im_bj, uv_b_observed, local_bj,
                                    dst_p,
                                    3
                                );
        MiscUtils::append_status_image( dst_p, ";im_pair_idx="+to_string( im_pair_idx )+" to total="+to_string(n_im_pairs), 1.0 );
        MiscUtils::imshow( "observed point feat matching", dst_p, 0.5 );

    }

    //---POSE---//
    #if 1
    //  optimized pose
    cout << TermColor::iYELLOW() << " Using optz__ai_T_bj <--- this->retrive_optimized_pose()\n" << TermColor::RESET();
    Matrix4d optz__ai_T_bj = this->retrive_optimized_pose( seq_a, local_ai, seq_b, local_bj );
    #endif

    #if 0
    // retrive_odometry_pose
    cout << TermColor::iYELLOW() << " Using optz__ai_T_bj <--- this->retrive_odometry_pose()\n" << TermColor::RESET();
    Matrix4d optz__ai_T_bj = x0_T_c.at(0).at( local_ai ).inverse() * odom__a0_T_b0.at( make_pair(0,1) ) * x0_T_c.at(1).at( local_bj );
    #endif

    #if 0
    //  retrive_initial_guess_pose
    cout << TermColor::iYELLOW() << " Using optz__ai_T_bj <--- this->retrive_initial_guess_pose()\n" << TermColor::RESET();
    Matrix4d optz__ai_T_bj = x0_T_c.at(0).at( local_ai ).inverse() * a0_T_b0.at( make_pair(0,1) ) * x0_T_c.at(1).at( local_bj );
    #endif


    // plot( im_bj, uv_b ), plotting the observed points
    cv::Mat dst_im_bj;
    MiscUtils::plot_point_sets( im_bj, uv_b_observed, dst_im_bj, cv::Scalar(0,255,255) , false, "plot( im_bj, uv_b )" );

    // plot( im_ai, uv_a ), plotting the observed points
    cv::Mat dst_im_ai;
    MiscUtils::plot_point_sets( im_ai, uv_a_observed, dst_im_ai, cv::Scalar(0,255,255) , false, "plot( im_ai, uv_a )" );



    // plot( im_bj, PI( bj_T_ai * ai_3d ) )
    //      and
    // plot( im_ai, PI( ai_T_bj * bj_3d) )
    int n_good_depths = 0;
    MatrixXd reproj_of__a_3d__in_frame_of_ref_of_bj = MatrixXd::Zero( 2, uv_a_observed.cols() );
    MatrixXd reproj_of__b_3d__in_frame_of_ref_of_ai = MatrixXd::Zero( 2, uv_b_observed.cols() );

    for( int k=0 ; k<normed_uv_a.at(pyp).at(im_pair_idx).cols() ; k++ )
    {
        Vector4d a_3d, b_3d;
        double ___d_a = d_a.at(pyp).at(im_pair_idx)(k);
        double ___d_b = d_b.at(pyp).at(im_pair_idx)(k);
        if( ___d_a < 0.5 || ___d_a > 7.0 || ___d_b < 0.5 || ___d_b > 7.0 )
            continue;
        // these 2 3d points are in the co-ordinates system of their own cameras.
        a_3d << ___d_a * normed_uv_a.at(pyp).at(im_pair_idx).col(k).topRows(3), 1.0;
        b_3d << ___d_b * normed_uv_b.at(pyp).at(im_pair_idx).col(k).topRows(3), 1.0;


        // 3d points from a, 2d observed points from b; plot on im_bj
        Vector4d k__a_3d__in_frame_of_ref_of_bj = optz__ai_T_bj.inverse() * a_3d;
        Vector2d k__repro_of_a_3d_in_bj;
        camera->spaceToPlane( k__a_3d__in_frame_of_ref_of_bj.topRows(3), k__repro_of_a_3d_in_bj );
        reproj_of__a_3d__in_frame_of_ref_of_bj.col( n_good_depths ) = k__repro_of_a_3d_in_bj;


        // 3d point from b, 2d observed point from a, plot on im_ai
        Vector4d k__b_3d__in_frame_of_ref_of_ai = optz__ai_T_bj * b_3d;
        Vector2d k__repro_of_b_3d_in_ai;
        camera->spaceToPlane( k__b_3d__in_frame_of_ref_of_ai.topRows(3), k__repro_of_b_3d_in_ai );
        reproj_of__b_3d__in_frame_of_ref_of_ai.col( n_good_depths ) = k__repro_of_b_3d_in_ai;



        n_good_depths++;

    }
    MiscUtils::plot_point_sets( dst_im_bj, reproj_of__a_3d__in_frame_of_ref_of_bj.leftCols(n_good_depths),
            cv::Scalar(0,0,255) , false, ";;plot( im_bj, PI( bj_T_ai * ai_3d ) )" );

    MiscUtils::plot_point_sets( dst_im_ai, reproj_of__b_3d__in_frame_of_ref_of_ai.leftCols(n_good_depths),
            cv::Scalar(0,0,255) , false, ";;plot( im_ai, PI( ai_T_bj * bj_3d) )" );
    // cout << "n_good_depths=" << n_good_depths << endl;

    MiscUtils::append_status_image( dst_im_ai,  ";ai="+to_string(ai)+", local_ai="+to_string(local_ai)+";;n_good_depths="+to_string(n_good_depths)   , 1.0 );
    MiscUtils::append_status_image( dst_im_bj,  ";bj="+to_string(bj)+", local_bj="+to_string(local_bj)+";;n_good_depths="+to_string(n_good_depths)  ,1.0  );


    MiscUtils::imshow( "plot( im_bj, uv_b )", dst_im_bj, 0.5 );
    MiscUtils::imshow( "plot( im_ai, uv_a )", dst_im_ai, 0.5 );

    cout << "Showing:\n";
    cout << "\t1. Observed Point feature matching\n";
    cout << "\t2. observed feats of im_ai and reprojections of b_3d on camera-ai\n";
    cout << "\t3. observed feats of im_bj and reprojections of a_3d on camera-bj\n";


    cout << TermColor::iCYAN() << "=====END LocalBundle::reprojection_test_for_this_image_pair=====" << TermColor::RESET() << endl;

}





void LocalBundle::reprojection_error( const camodocal::CameraPtr camera )
{
    cout << TermColor::bWHITE() << "[LocalBundle::reprojection_error]\n" << TermColor::RESET();
    int seq_a = 0;
    int seq_b = 1;
    // int im_pair_idx = 3;
    auto pyp = std::make_pair( seq_a, seq_b );

    int n_im_pairs = (int) all_pair_idx.at( pyp ).size();
    cout << "# of image pairs = " << n_im_pairs << endl;

    for( int h=0 ; h<n_im_pairs ; h++ ) {
        reprojection_error_for_this_image_pair( camera, h );
    }
    cout << TermColor::bWHITE() << "[LocalBundle::reprojection_error] END\n" << TermColor::RESET();
}

void LocalBundle::reprojection_error_for_this_image_pair( const camodocal::CameraPtr camera, int im_pair_idx )
{
    // cout << TermColor::iCYAN() << "=====LocalBundle::reprojection_error=====" << im_pair_idx << TermColor::RESET() << endl;
    int seq_a = 0;
    int seq_b = 1;
    // int im_pair_idx = 3;
    auto pyp = std::make_pair( seq_a, seq_b );
    int n_im_pairs = (int) all_pair_idx.at( pyp ).size();
    if( im_pair_idx < 0 || im_pair_idx >= n_im_pairs ) {
        cout << __FILE__ << ":" << __LINE__ << "[LocalBundle::reprojection_error_for_this_image_pair]invalid im_pair_idx=" << im_pair_idx << " has to be between 0 and " <<   n_im_pairs << endl;
        exit(2);
    }


    // randomly picking the 3rd image pair `im_pair_idx`
    int ai = all_pair_idx.at( pyp ).at( im_pair_idx ).first;
    int bj = all_pair_idx.at( pyp ).at( im_pair_idx ).second;
    cout << "im_pair_idx=" << im_pair_idx << "\t";
    cout << "ai="<< ai << "\tbj=" << bj << "\t";

    int local_ai = std::distance( seq_x_idx.at( seq_a ).begin(),     std::find( seq_x_idx.at( seq_a).begin(), seq_x_idx.at( seq_a).end(), ai ) );
    int local_bj = std::distance( seq_x_idx.at( seq_b ).begin(),     std::find( seq_x_idx.at( seq_b).begin(), seq_x_idx.at( seq_b).end(), bj ) );
    cout << "local_ai=" << local_ai << "\tlocal_bj=" << local_bj << "\t";

    // cv::Mat im_ai, im_bj;
    MatrixXd uv_a_observed, uv_b_observed;
    uv_a_observed = StaticPointFeatureMatching::normalized_image_cordinates_to_image_coordinates( camera, normed_uv_a.at(pyp).at(im_pair_idx) );
    uv_b_observed = StaticPointFeatureMatching::normalized_image_cordinates_to_image_coordinates( camera, normed_uv_b.at(pyp).at(im_pair_idx) );


    //---POSE---//
    #if 1
    //  optimized pose
    // cout << TermColor::iYELLOW() << " Using optz__ai_T_bj <--- this->retrive_optimized_pose()\n" << TermColor::RESET();
    Matrix4d optz__ai_T_bj = this->retrive_optimized_pose( seq_a, local_ai, seq_b, local_bj );
    #endif

    #if 0
    // retrive_odometry_pose
    // cout << TermColor::iYELLOW() << " Using optz__ai_T_bj <--- this->retrive_odometry_pose()\n" << TermColor::RESET();
    Matrix4d optz__ai_T_bj = x0_T_c.at(0).at( local_ai ).inverse() * odom__a0_T_b0.at( make_pair(0,1) ) * x0_T_c.at(1).at( local_bj );
    #endif


    #if 0
    // TODO retrive_initial_guess_pose
    Matrix4d optz__ai_T_bj = x0_T_c.at(0).at( local_ai ).inverse() * a0_T_b0.at( make_pair(0,1) ) * x0_T_c.at(1).at( local_bj );
    #endif


    // plot( im_bj, PI( bj_T_ai * ai_3d ) )
    //      and
    // plot( im_ai, PI( ai_T_bj * bj_3d) )
    int n_good_depths = 0;
    double sum_del_a = 0.0, sum_del_b = 0.0;

    for( int k=0 ; k<normed_uv_a.at(pyp).at(im_pair_idx).cols() ; k++ )
    {
        Vector4d a_3d, b_3d;
        double ___d_a = d_a.at(pyp).at(im_pair_idx)(k);
        double ___d_b = d_b.at(pyp).at(im_pair_idx)(k);
        if( ___d_a < 0.5 || ___d_a > 7.0 || ___d_b < 0.5 || ___d_b > 7.0 )
            continue;
        // these 2 3d points are in the co-ordinates system of their own cameras.
        a_3d << ___d_a * normed_uv_a.at(pyp).at(im_pair_idx).col(k).topRows(3), 1.0;
        b_3d << ___d_b * normed_uv_b.at(pyp).at(im_pair_idx).col(k).topRows(3), 1.0;


        // 3d points from a, 2d observed points from b; plot on im_bj
        Vector4d k__a_3d__in_frame_of_ref_of_bj = optz__ai_T_bj.inverse() * a_3d;
        Vector2d k__repro_of_a_3d_in_bj;
        camera->spaceToPlane( k__a_3d__in_frame_of_ref_of_bj.topRows(3), k__repro_of_a_3d_in_bj );



        // 3d point from b, 2d observed point from a, plot on im_ai
        Vector4d k__b_3d__in_frame_of_ref_of_ai = optz__ai_T_bj * b_3d;
        Vector2d k__repro_of_b_3d_in_ai;
        camera->spaceToPlane( k__b_3d__in_frame_of_ref_of_ai.topRows(3), k__repro_of_b_3d_in_ai );

        double del_a = (uv_a_observed.col(k).topRows(2) - k__repro_of_b_3d_in_ai).norm();
        double del_b = (uv_b_observed.col(k).topRows(2) - k__repro_of_a_3d_in_bj).norm();
        sum_del_a += del_a;
        sum_del_b += del_b;

        #if 0
        cout << "[k="<< k << "]\tdel_a=" << del_a << "\tdel_b="<< del_b << "\n";
        cout << "\tuv_a_observed=" << uv_a_observed.col(k).topRows(3).transpose() << "\tk__repro_of_b_3d_in_ai=" << k__repro_of_b_3d_in_ai.transpose() << endl;
        cout << "\tuv_b_observed=" << uv_b_observed.col(k).topRows(3).transpose() << "\tk__repro_of_a_3d_in_bj=" << k__repro_of_a_3d_in_bj.transpose() << endl;
        cout << endl;
        #endif

        n_good_depths++;

    }
    if( n_good_depths != 0 )  {
        cout << TermColor::YELLOW();
        cout << "avg_del_a=" << sum_del_a / n_good_depths << "\t";
        cout << "avg_del_b=" << sum_del_b / n_good_depths << "\t";
        cout << TermColor::RESET();
    }
    cout << "n_good_depths = " << n_good_depths << endl;


    // cout << TermColor::iCYAN() << "=====END LocalBundle::reprojection_error_for_this_image_pair=====" << TermColor::RESET() << endl;

}





void LocalBundle::reprojection_debug_images_for_this_image_pair( const camodocal::CameraPtr camera, int im_pair_idx,
    cv::Mat& _dst_observed_correspondence_,  cv::Mat& _dst_image_a, cv::Mat& _dst_image_b )
{
    // cout << TermColor::iCYAN() << "=====LocalBundle::reprojection_test_for_this_image_pair=====" << im_pair_idx << TermColor::RESET() << endl;
    int seq_a = 0;
    int seq_b = 1;
    // int im_pair_idx = 3;
    auto pyp = std::make_pair( seq_a, seq_b );
    int n_im_pairs = (int) all_pair_idx.at( pyp ).size();
    if( im_pair_idx < 0 || im_pair_idx >= n_im_pairs ) {
        cout << __FILE__ << ":" << __LINE__ << "[LocalBundle::reprojection_debug_images_for_this_image_pair]invalid im_pair_idx=" << im_pair_idx << " has to be between 0 and " <<   n_im_pairs << endl;
        exit(2);
    }


    // randomly picking the 3rd image pair
    int ai = all_pair_idx.at( pyp ).at( im_pair_idx ).first;
    int bj = all_pair_idx.at( pyp ).at( im_pair_idx ).second;
    // cout << "ai="<< ai << "\tbj=" << bj << "\t";

    int local_ai = std::distance( seq_x_idx.at( seq_a ).begin(),     std::find( seq_x_idx.at( seq_a).begin(), seq_x_idx.at( seq_a).end(), ai ) );
    int local_bj = std::distance( seq_x_idx.at( seq_b ).begin(),     std::find( seq_x_idx.at( seq_b).begin(), seq_x_idx.at( seq_b).end(), bj ) );
    // cout << "local_ai=" << local_ai << "\tlocal_bj=" << local_bj << endl;

    cv::Mat im_ai, im_bj;
    MatrixXd uv_a_observed, uv_b_observed;

    // plot( im_ai, uv_ai | im_bj, uv_bj ) : The point feature matches
    {
        im_ai = seq_x_images.at( seq_a ).at( local_ai );
        im_bj = seq_x_images.at( seq_b ).at( local_bj );
        if( im_ai.empty() || im_bj.empty() ) {
            cout << "[reprojection_debug_images_for_this_image_pair]PPP@ Cannot load im_ai or im_bj\n";
            exit(3);
        }

        uv_a_observed = StaticPointFeatureMatching::normalized_image_cordinates_to_image_coordinates( camera, normed_uv_a.at(pyp).at(im_pair_idx) );
        uv_b_observed = StaticPointFeatureMatching::normalized_image_cordinates_to_image_coordinates( camera, normed_uv_b.at(pyp).at(im_pair_idx) );

        cv::Mat dst_p;
        MiscUtils::plot_point_pair( im_ai, uv_a_observed, local_ai,
                                    im_bj, uv_b_observed, local_bj,
                                    dst_p,
                                    #if 0
                                    // 3
                                    #else
                                    cv::Scalar( 0,0,255 ), cv::Scalar( 0,255,0 ), false
                                    #endif
                                );
        MiscUtils::append_status_image( dst_p, "im_pair#"+to_string( im_pair_idx )+" of total-pairs="+to_string(n_im_pairs), 1.0 );
        // MiscUtils::imshow( "observed point feat matching", dst_p, 0.5 );
        _dst_observed_correspondence_ = dst_p;

    }

    //---POSE---//
    #if 1
    //  optimized pose
    // cout << TermColor::iYELLOW() << " Using optz__ai_T_bj <--- this->retrive_optimized_pose()\n" << TermColor::RESET();
    Matrix4d optz__ai_T_bj = this->retrive_optimized_pose( seq_a, local_ai, seq_b, local_bj );
    #endif

    #if 0
    // retrive_odometry_pose
    // cout << TermColor::iYELLOW() << " Using optz__ai_T_bj <--- this->retrive_odometry_pose()\n" << TermColor::RESET();
    Matrix4d optz__ai_T_bj = x0_T_c.at(0).at( local_ai ).inverse() * odom__a0_T_b0.at( make_pair(0,1) ) * x0_T_c.at(1).at( local_bj );
    #endif

    #if 0
    //  retrive_initial_guess_pose
    // cout << TermColor::iYELLOW() << " Using optz__ai_T_bj <--- this->retrive_initial_guess_pose()\n" << TermColor::RESET();
    Matrix4d optz__ai_T_bj = x0_T_c.at(0).at( local_ai ).inverse() * a0_T_b0.at( make_pair(0,1) ) * x0_T_c.at(1).at( local_bj );
    #endif


    // plot( im_bj, uv_b ), plotting the observed points
    cv::Mat dst_im_bj;
    MiscUtils::plot_point_sets( im_bj, uv_b_observed, dst_im_bj, cv::Scalar(0,255,255) , false, "plot( im_bj, uv_b )" );

    // plot( im_ai, uv_a ), plotting the observed points
    cv::Mat dst_im_ai;
    MiscUtils::plot_point_sets( im_ai, uv_a_observed, dst_im_ai, cv::Scalar(0,255,255) , false, "plot( im_ai, uv_a )" );



    // plot( im_bj, PI( bj_T_ai * ai_3d ) )
    //      and
    // plot( im_ai, PI( ai_T_bj * bj_3d) )
    int n_good_depths = 0;
    double sum_del_a = 0.0, sum_del_b = 0.0;

    MatrixXd reproj_of__a_3d__in_frame_of_ref_of_bj = MatrixXd::Zero( 2, uv_a_observed.cols() );
    MatrixXd reproj_of__b_3d__in_frame_of_ref_of_ai = MatrixXd::Zero( 2, uv_b_observed.cols() );

    int n_correspondences = normed_uv_a.at(pyp).at(im_pair_idx).cols();
    for( int k=0 ; k<n_correspondences ; k++ )
    {
        Vector4d a_3d, b_3d;
        double ___d_a = d_a.at(pyp).at(im_pair_idx)(k);
        double ___d_b = d_b.at(pyp).at(im_pair_idx)(k);
        if( ___d_a < 0.5 || ___d_a > 7.0 || ___d_b < 0.5 || ___d_b > 7.0 )
            continue;
        // these 2 3d points are in the co-ordinates system of their own cameras.
        a_3d << ___d_a * normed_uv_a.at(pyp).at(im_pair_idx).col(k).topRows(3), 1.0;
        b_3d << ___d_b * normed_uv_b.at(pyp).at(im_pair_idx).col(k).topRows(3), 1.0;


        // 3d points from a, 2d observed points from b; plot on im_bj
        Vector4d k__a_3d__in_frame_of_ref_of_bj = optz__ai_T_bj.inverse() * a_3d;
        Vector2d k__repro_of_a_3d_in_bj;
        camera->spaceToPlane( k__a_3d__in_frame_of_ref_of_bj.topRows(3), k__repro_of_a_3d_in_bj );
        reproj_of__a_3d__in_frame_of_ref_of_bj.col( n_good_depths ) = k__repro_of_a_3d_in_bj;


        // 3d point from b, 2d observed point from a, plot on im_ai
        Vector4d k__b_3d__in_frame_of_ref_of_ai = optz__ai_T_bj * b_3d;
        Vector2d k__repro_of_b_3d_in_ai;
        camera->spaceToPlane( k__b_3d__in_frame_of_ref_of_ai.topRows(3), k__repro_of_b_3d_in_ai );
        reproj_of__b_3d__in_frame_of_ref_of_ai.col( n_good_depths ) = k__repro_of_b_3d_in_ai;

        double del_a = (uv_a_observed.col(k).topRows(2) - k__repro_of_b_3d_in_ai).norm();
        double del_b = (uv_b_observed.col(k).topRows(2) - k__repro_of_a_3d_in_bj).norm();
        sum_del_a += del_a;
        sum_del_b += del_b;

        n_good_depths++;

    }
    MiscUtils::plot_point_sets( dst_im_bj, reproj_of__a_3d__in_frame_of_ref_of_bj.leftCols(n_good_depths),
            cv::Scalar(0,0,255) , false, ";plot( im_bj, PI( bj_T_ai * ai_3d ) )" );

    MiscUtils::plot_point_sets( dst_im_ai, reproj_of__b_3d__in_frame_of_ref_of_ai.leftCols(n_good_depths),
            cv::Scalar(0,0,255) , false, ";plot( im_ai, PI( ai_T_bj * bj_3d) )" );
    // cout << "n_good_depths=" << n_good_depths << endl;

    MiscUtils::append_status_image( dst_im_ai,  "ai="+to_string(ai)+", local_ai="+to_string(local_ai)+";n_good_depths="+to_string(n_good_depths)   , 1.0 );
    MiscUtils::append_status_image( dst_im_bj,  "bj="+to_string(bj)+", local_bj="+to_string(local_bj)+";n_good_depths="+to_string(n_good_depths)  ,1.0  );

    // also append_status_image the avg_reprojection error float value
    if( n_good_depths != 0 )  {
        MiscUtils::append_status_image( dst_im_ai,  "avg_reprojection_error_a=" + to_string( sum_del_a / n_good_depths )  , 1.0 );
        MiscUtils::append_status_image( dst_im_bj,  "avg_reprojection_error_b=" + to_string( sum_del_b / n_good_depths )  ,1.0  );
    }


    // MiscUtils::imshow( "plot( im_bj, uv_b )", dst_im_bj, 0.5 );
    // MiscUtils::imshow( "plot( im_ai, uv_a )", dst_im_ai, 0.5 );
    _dst_image_a = dst_im_ai;
    _dst_image_b = dst_im_bj;

    // cout << "Showing:\n";
    // cout << "\t1. Observed Point feature matching\n";
    // cout << "\t2. observed feats of im_ai and reprojections of b_3d on camera-ai\n";
    // cout << "\t3. observed feats of im_bj and reprojections of a_3d on camera-bj\n";
    //
    //
    // cout << TermColor::iCYAN() << "=====END LocalBundle::reprojection_test_for_this_image_pair=====" << TermColor::RESET() << endl;

}



void LocalBundle::reprojection_debug_images_to_disk( const camodocal::CameraPtr camera, const string PREFIX )
{
    cout << TermColor::bWHITE() << "[LocalBundle::reprojection_debug_images_to_disk]PREFIX=" << PREFIX << "\n" << TermColor::RESET();
    int seq_a = 0;
    int seq_b = 1;
    // int im_pair_idx = 3;
    auto pyp = std::make_pair( seq_a, seq_b );

    int n_im_pairs = (int) all_pair_idx.at( pyp ).size();
    // cout << "# of image pairs = " << n_im_pairs << endl;

    for( int h=0 ; h<n_im_pairs ; h++ ) {
        cv::Mat _dst_observed_correspondence_, _dst_image_a, _dst_image_b;
        reprojection_debug_images_for_this_image_pair( camera, h,
            _dst_observed_correspondence_, _dst_image_a, _dst_image_b);

        cout << TermColor::bGREEN() << "Write 3 images in PREFIX=" << PREFIX+"image_pair_" + to_string(h) << TermColor::RESET() << endl;
        cv::imwrite( PREFIX+"image_pair_" + to_string(h) + "_dst_correspondence_observed.jpg", _dst_observed_correspondence_ );
        cv::imwrite( PREFIX+"image_pair_" + to_string(h) + "_dst_image_a.jpg", _dst_image_a );
        cv::imwrite( PREFIX+"image_pair_" + to_string(h) + "_dst_image_b.jpg", _dst_image_b );
    }
    cout << TermColor::bWHITE() << "[LocalBundle::reprojection_debug_images_to_disk] END\n" << TermColor::RESET();
}
