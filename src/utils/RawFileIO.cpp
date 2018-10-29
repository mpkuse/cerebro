#include "RawFileIO.h"


void RawFileIO::write_image( string fname, const cv::Mat& img)
{
    __RawFileIO__write_image_debug_dm( std::cout << "Writing file: "<< fname << endl );
    cv::imwrite( (fname).c_str(), img );
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
    __RawFileIO__write_image_debug_dm( std::cout << "\033[1;32m" <<"Written to file: "<< filename  << "\033[0m\n" );
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
    __RawFileIO__write_image_debug_dm(std::cout << "\033[1;32m" <<"Written to file: "<< filename  << "\033[0m\n");
  }
  else
  {
    std::cout << "\033[1;31m" << "FAIL TO OPEN FILE for writing: "<< filename << "\033[0m\n";

  }

}
