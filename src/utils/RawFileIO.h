#pragma once


#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <queue>
#include <ostream>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;

#define __RawFileIO__write_image_debug_dm( msg ) msg;

class RawFileIO
{
public:
    static void write_image( string fname, const cv::Mat& img);
    static void write_string( string fname, const string& my_string);

    // templated static function canot only exist in header files.
    template <typename Derived>
    static void write_EigenMatrix(const string& filename, const MatrixBase<Derived>& a)
    {
      // string base = string("/home/mpkuse/Desktop/bundle_adj/dump/datamgr_mateigen_");
      std::ofstream file(filename);
      if( file.is_open() )
      {
        // file << a.format(CSVFormat) << endl;
        file << a << endl;
        __RawFileIO__write_image_debug_dm(std::cout << "\033[1;32m" <<"write_EigenMatrix: "<< filename  << "\033[0m\n";)
      }
      else
      {
        cout << "\033[1;31m" << "FAIL TO OPEN FILE for writing: "<< filename << "\033[0m\n";

      }
    }


    static void write_Matrix2d( const string& filename, const double * D, int nRows, int nCols );
    static void write_Matrix1d( const string& filename, const double * D, int n  );

};
