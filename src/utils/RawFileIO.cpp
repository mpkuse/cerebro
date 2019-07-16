#include "RawFileIO.h"


void RawFileIO::write_image( string fname, const cv::Mat& img)
{
    __RawFileIO__write_image_debug_dm( std::cout << "write_image: "<< fname << endl );
    cv::imwrite( (fname).c_str(), img );
}


void RawFileIO::write_string( string fname, const string& my_string)
{
    __RawFileIO__write_image_debug_dm( std::cout << "write_string: "<< fname << endl );
    std::ofstream outfile( fname );
    outfile << my_string << endl;
    outfile.close();
}

// templated static function canot only exist in header files.
//  so this was moved to the header file
// template <typename Derived>
// void RawFileIO::write_EigenMatrix(const string& filename, const MatrixBase<Derived>& a)
// {
//   // string base = string("/home/mpkuse/Desktop/bundle_adj/dump/datamgr_mateigen_");
//   std::ofstream file(filename);
//   if( file.is_open() )
//   {
//     // file << a.format(CSVFormat) << endl;
//     file << a << endl;
//     write_image_debug_dm(std::cout << "\033[1;32m" <<"Written to file: "<< filename  << "\033[0m\n" );
//   }
//   else
//   {
//     cout << "\033[1;31m" << "FAIL TO OPEN FILE for writing: "<< filename << "\033[0m\n";
//
//   }
// }


void RawFileIO::write_Matrix2d( const string& filename, const double * D, int nRows, int nCols )
{
  // string base = string("/home/mpkuse/Desktop/bundle_adj/dump/datamgr_mat2d_");
  std::ofstream file(filename);
  if( file.is_open() )
  {
    int c = 0 ;
    for( int i=0; i<nRows ; i++ )
    {
      file << D[c];
      c++;
      for( int j=1 ; j<nCols ; j++ )
      {
        file << ", " << D[c] ;
        c++;
      }
      file << "\n";
    }
    __RawFileIO__write_image_debug_dm( std::cout << "\033[1;32m" <<"write_Matrix2d to file: "<< filename  << "\033[0m\n" );
  }
  else
  {
    std::cout << "\033[1;31m" << "FAIL TO OPEN FILE for writing: "<< filename << "\033[0m\n";

  }

}

void RawFileIO::write_Matrix1d( const string& filename, const double * D, int n  )
{
  std::ofstream file(filename);
  if( file.is_open() )
  {
    file << D[0];
    for( int i=1 ; i<n ; i++ )
      file << ", " << D[i] ;
    file << "\n";
    __RawFileIO__write_image_debug_dm(std::cout << "\033[1;32m" <<"write_Matrix1d: "<< filename  << "\033[0m\n");
  }
  else
  {
    std::cout << "\033[1;31m" << "FAIL TO OPEN FILE for writing: "<< filename << "\033[0m\n";

  }

}




bool RawFileIO::read_eigen_matrix( string filename, MatrixXd& result )
    {
    int cols = 0, rows = 0;
    const int MAXBUFSIZE = ((int) 1e6);
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
    __RawFileIO__write_image_debug_dm( std::cout << "\033[1;32m" << "read_eigen_matrix: "<< filename << "\033[0m"  );
    ifstream infile;
    infile.open(filename);
    if( !infile ) {
        cout << "\n\033[1;31m" << "failed to open file " << filename << "\033[0m" << endl;
        return false;
    }
    while (! infile.eof())
        {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
        }

    infile.close();

    rows--;

    // Populate matrix with numbers.
    result = MatrixXd::Zero(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

    // return result;
    __RawFileIO__write_image_debug_dm( cout << "\tshape=" << result.rows() << "x" << result.cols() << endl; )

    return true;
    };


bool RawFileIO::read_eigen_matrix( string filename, Matrix4d& result )
    {
    int cols = 0, rows = 0;
    const int MAXBUFSIZE = ((int) 2000);
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
    __RawFileIO__write_image_debug_dm( std::cout << "\033[1;32m"  << "read_eigen_matrix: "<< filename << "\033[0m" );
    ifstream infile;
    infile.open(filename);
    if( !infile ) {
        cout << "\n\033[1;31m" << "failed to open file " << filename << "\033[0m" << endl;
        return false;
    }
    while (! infile.eof())
        {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
        }

    infile.close();

    rows--;

    assert( rows==4 && cols==4 && "\n[RawFileIO::read_eigen_matrix( string filename, Matrix4d& result )] The input file is not 4x4" );

    // Populate matrix with numbers.
    result = Matrix4d::Zero();
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

    // return result;
    __RawFileIO__write_image_debug_dm( cout << "\tshape=" << result.rows() << "x" << result.cols() << endl; )
    return true;
    };



bool RawFileIO::read_eigen_matrix( string filename, Matrix3d& result )
{
    int cols = 0, rows = 0;
    const int MAXBUFSIZE = ((int) 2000);
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
    __RawFileIO__write_image_debug_dm( std::cout << "\033[1;32m"  << "read_eigen_matrix: "<< filename << "\033[0m" );
    ifstream infile;
    infile.open(filename);
    if( !infile ) {
        cout << "\n\033[1;31m" << "failed to open file " << filename << "\033[0m" << endl;
        return false;
    }
    while (! infile.eof())
        {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
        }

    infile.close();

    rows--;

    assert( rows==3 && cols==3 && "\n[RawFileIO::read_eigen_matrix( string filename, Matrix3d& result )] The input file is not 3x3" );

    // Populate matrix with numbers.
    result = Matrix3d::Zero();
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

    // return result;
    __RawFileIO__write_image_debug_dm( cout << "\tshape=" << result.rows() << "x" << result.cols() << endl; )
    return true;
}


bool RawFileIO::read_eigen_matrix( string filename, VectorXi& result )
{
    int cols = 0, rows = 0;
    const int MAXBUFSIZE = ((int) 2000);
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
    __RawFileIO__write_image_debug_dm( std::cout << "\033[1;32m"  << "read_eigen_matrix: "<< filename << "\033[0m" );
    ifstream infile;
    infile.open(filename);
    if( !infile ) {
        cout << "\n\033[1;31m" << "failed to open file " << filename << "\033[0m" << endl;
        return false;
    }
    while (! infile.eof())
        {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
        }

    infile.close();

    rows--;

    assert( cols==1 && "\n[RawFileIO::read_eigen_matrix( string filename, VectorXi& result )] The input file is not row-vector" );

    // Populate matrix with numbers.
    result = VectorXi::Zero(rows);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = (int) buff[ cols*i+j ];

    // return result;
    __RawFileIO__write_image_debug_dm( cout << "\tshape=" << result.rows() << "x" << result.cols() << endl; )
    return true;
};


bool RawFileIO::read_eigen_matrix( const std::vector<double>& ary, Matrix4d& result )
{
    assert( ary.size() == result.rows() * result.cols() && "[RawFileIO::read_eigen_matrix] size of vector<double> need to be equal to the size of Eigen::Matrix");

    for( int i=0 ; i<ary.size() ; i++ ) {
        result(i/4, i%4) = ary[i];
    }
}


bool RawFileIO::if_file_exist( char * fname )
{
  ifstream f(fname);
  return f.good();
}

bool RawFileIO::if_file_exist( string fname ) { return if_file_exist( fname.c_str() ); }

bool RawFileIO::is_path_a_directory(const char* path)
{
    struct stat buf;
    stat(path, &buf);
    return S_ISDIR(buf.st_mode);
}

bool RawFileIO::is_path_a_directory(const string path) { return is_path_a_directory(path.c_str() ); }

int RawFileIO::exec_cmd( const string& system_cmd ) //< Executes a unix command.
{
    cout <<"\033[1;33m"  << system_cmd << "\033[0m" << endl;
    const int _err_code = system( system_cmd.c_str() );
    return _err_code;
}


#ifdef WITH_NLOHMANN_JSON
bool RawFileIO::read_eigen_matrix_fromjson( const json str, MatrixXd&  output )
{
    int ncols = str["cols"];
    int nrows = str["rows"];
    string data = str["data"];
    if( ncols > 0 && ncols > 0 )
        output = MatrixXd::Zero(nrows, ncols );
    else {
        cout << "[RawFileIO::read_eigen_matrix_fromjson] ERROR, nrows and cols should be positive\n";
        return false;
    }


    // split( data, '\n')
    vector<string> all_rows = MiscUtils::split( data, '\n' );
    if( nrows != all_rows.size() )
    {
        cout << "[RawFileIO::read_eigen_matrix_fromjson] ERROR, requested " << nrows << " but actually are " << all_rows.size() << endl;
        return false;
    }
    for( int r=0 ; r<all_rows.size() ; r++ )
    {
        vector<string> all_cols_for_this_row = MiscUtils::split( all_rows[r], ',' );
        if( ncols != all_cols_for_this_row.size() )
        {
            cout << "[RawFileIO::read_eigen_matrix_fromjson] ERROR, requested " << ncols << " but actually are " << all_cols_for_this_row.size() << " for row=" << r << endl;
            return false;
        }

        for( int c=0 ; c<all_cols_for_this_row.size() ; c++ )
        {
            output(r, c) = std::stod( all_cols_for_this_row[c] );
        }
    }
    return true;

}
#endif //WITH_NLOHMANN_JSON


#ifdef WITH_NLOHMANN_JSON
// The input json need to be something like:
//{
//            "cols": 4,
//            "rows": 4,
//            "data": "0.2857131543876468, -0.2530077727001951, 0.9243132912401226, -0.02953755668229465,\t\n-0.9566719337894203, -0.01884313243445668, 0.2905576491157474, 0.2114882406102183,\t\n-0.05609638588601445, -0.9672807262182707, -0.2474291659792429, 0.04534835279466286,\t\n0, 0, 0, 1"
// }
bool RawFileIO::read_eigen_matrix4d_fromjson( const json str, Matrix4d&  output )
{
    output = Matrix4d::Zero();
    int ncols = str["cols"];
    int nrows = str["rows"];
    string data = str["data"];

    if( ncols != 4 || nrows != 4 )
    {
        cout << "[RawFileIO::read_eigen_matrix4d_fromjson] ERROR, you request to convert to Matrix4d however the input json rows and cols != 4x4\n";
        return false;
    }


    // split( data, '\n')
    vector<string> all_rows = MiscUtils::split( data, '\n' );
    if( nrows != all_rows.size() )
    {
        cout << "[RawFileIO::read_eigen_matrix4d_fromjson] ERROR, requested " << nrows << " but actually are " << all_rows.size() << endl;
        return false;
    }
    for( int r=0 ; r<all_rows.size() ; r++ )
    {
        vector<string> all_cols_for_this_row = MiscUtils::split( all_rows[r], ',' );
        if( ncols != all_cols_for_this_row.size() )
        {
            cout << "[RawFileIO::read_eigen_matrix4d_fromjson] ERROR, requested " << ncols << " but actually are " << all_cols_for_this_row.size() << " for row=" << r << endl;
            return false;
        }

        for( int c=0 ; c<all_cols_for_this_row.size() ; c++ )
        {
            output(r, c) = std::stod( all_cols_for_this_row[c] );
        }
    }
    return true;

}
#endif //WITH_NLOHMANN_JSON

#ifdef WITH_NLOHMANN_JSON
bool RawFileIO::read_eigen_vector_fromjson( const json str, VectorXd&  output )
{
    int ncols = str["cols"];
    int nrows = str["rows"];
    string data = str["data"];

    if( nrows > 0 ) {
        output = VectorXd::Zero( nrows );
    }
    else {
        cout << "[RawFileIO::read_eigen_vector_fromjson] ERROR, nrows should be positive\n";
        return false;
    }

    if( ncols != 1 )
    {
        cout << "[RawFileIO::read_eigen_vector_fromjson] ERROR, you request to convert to VectorXd however the input json cols != 1\n";
        return false;
    }

    vector<string> all_rows = MiscUtils::split( data, '\n' );
    if( nrows != all_rows.size() )
    {
        cout << "[RawFileIO::read_eigen_vector_fromjson] ERROR, requested " << nrows << " but actually are " << all_rows.size() << endl;
        return false;
    }

    for( int r=0 ; r<all_rows.size() ; r++ )
    {
        vector<string> all_cols_for_this_row = MiscUtils::split( all_rows[r], ',' );
        if( all_cols_for_this_row.size() != 1 )
        {
            cout << "[RawFileIO::read_eigen_vector_fromjson] ERROR, requested " << ncols << " but actually are " << all_cols_for_this_row.size() << " for row=" << r << endl;
            return false;
        }

        output( r ) = std::stod( all_cols_for_this_row[0] );
    }
    return true;


}
#endif //WITH_NLOHMANN_JSON
