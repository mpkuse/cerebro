#include "MiscUtils.h"

string MiscUtils::type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

string MiscUtils::cvmat_info( const cv::Mat& mat )
{
    std::stringstream buffer;
    buffer << "shape=" << mat.rows << "," << mat.cols << "," << mat.channels() ;
    buffer << "\t" << "dtype=" << MiscUtils::type2str( mat.type() );
    return buffer.str();
}

string MiscUtils::imgmsg_info(const sensor_msgs::ImageConstPtr &img_msg)
{
    std::stringstream buffer;
    buffer << "---\n";
    buffer << "\theader:\n";
    buffer << "\t\tseq: " << img_msg->header.seq << endl;
    buffer << "\t\tstamp: " << img_msg->header.stamp << endl;
    buffer << "\t\tframe_id: " << img_msg->header.frame_id << endl;
    buffer << "\twidth:" << img_msg->width << endl;
    buffer << "\theight:" << img_msg->height << endl;
    buffer << "\tencoding:" << img_msg->encoding << endl;
    buffer << "\tis_bigendian:" << (int) img_msg->is_bigendian << endl;
    buffer << "\tstep:" << img_msg->step << endl;
    buffer << "\tdata:" << "[..." << img_msg->step * img_msg->height << " sized array...]" << endl;

    return buffer.str();

}

string MiscUtils::imgmsg_info(const sensor_msgs::Image& img_msg)
{
    std::stringstream buffer;
    buffer << "---\n";
    buffer << "\theader:\n";
    buffer << "\t\tseq: " << img_msg.header.seq << endl;
    buffer << "\t\tstamp: " << img_msg.header.stamp << endl;
    buffer << "\t\tframe_id: " << img_msg.header.frame_id << endl;
    buffer << "\twidth:" << img_msg.width << endl;
    buffer << "\theight:" << img_msg.height << endl;
    buffer << "\tencoding:" << img_msg.encoding << endl;
    buffer << "\tis_bigendian:" << (int) img_msg.is_bigendian << endl;
    buffer << "\tstep:" << img_msg.step << endl;
    buffer << "\tdata:" << "[..." << img_msg.step * img_msg.height << " sized array...]" << endl;

    return buffer.str();

}

cv::Mat MiscUtils::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

std::vector<std::string>
MiscUtils::split( std::string const& original, char separator )
{
    std::vector<std::string> results;
    std::string::const_iterator start = original.begin();
    std::string::const_iterator end = original.end();
    std::string::const_iterator next = std::find( start, end, separator );
    while ( next != end ) {
        results.push_back( std::string( start, next ) );
        start = next + 1;
        next = std::find( start, end, separator );
    }
    results.push_back( std::string( start, next ) );
    return results;
}

void MiscUtils::keypoint_2_eigen( const std::vector<cv::KeyPoint>& kp, MatrixXd& uv, bool make_homogeneous )
{
    assert( kp.size() > 0 && "MiscUtils::keypoint_2_eigen empty kp.");
    uv = MatrixXd::Constant( (make_homogeneous?3:2), kp.size(), 1.0 );
    for( int i=0; i<kp.size() ; i++ )
    {
        uv(0,i) = kp[i].pt.x;
        uv(1,i) = kp[i].pt.y;
    }
}

void MiscUtils::dmatch_2_eigen( const std::vector<cv::KeyPoint>& kp1, const std::vector<cv::KeyPoint>& kp2,
                            const std::vector<cv::DMatch> matches,
                            MatrixXd& M1, MatrixXd& M2,
                            bool make_homogeneous
                        )
{
    assert( matches.size() > 0 && "MiscUtils::dmatch_2_eigen DMatch cannot be empty" );
    assert( kp1.size() > 0 && kp2.size() > 0 && "MiscUtils::dmatch_2_eigen keypoints cannot be empty" );

    M1 = MatrixXd::Constant( (make_homogeneous?3:2), matches.size(), 1.0 );
    M2 = MatrixXd::Constant( (make_homogeneous?3:2), matches.size(), 1.0 );
    for( int i=0 ; i<matches.size() ; i++ ) {
        int queryIdx = matches[i].queryIdx; //kp1
        int trainIdx = matches[i].trainIdx; //kp2
        assert( queryIdx >=0 && queryIdx < kp1.size() );
        assert( trainIdx >=0 && trainIdx < kp2.size() );
        M1(0,i) = kp1[queryIdx].pt.x;
        M1(1,i) = kp1[queryIdx].pt.y;

        M2(0,i) = kp2[trainIdx].pt.x;
        M2(1,i) = kp2[trainIdx].pt.y;
    }
}



void MiscUtils::plot_point_sets( const cv::Mat& im, const MatrixXd& pts_set, cv::Mat& dst,
                                        const cv::Scalar& color, bool enable_keypoint_annotation, const string& msg )
{
  MatrixXf pts_set_float;
  pts_set_float = pts_set.cast<float>();

  cv::Mat pts_set_mat;
  cv::eigen2cv( pts_set_float, pts_set_mat );

  MiscUtils::plot_point_sets( im, pts_set_mat, dst, color, enable_keypoint_annotation, msg );
}

// in place manip
void MiscUtils::plot_point_sets( cv::Mat& im, const MatrixXd& pts_set,
                                        const cv::Scalar& color, bool enable_keypoint_annotation, const string& msg )
{
  MatrixXf pts_set_float;
  pts_set_float = pts_set.cast<float>();

  cv::Mat pts_set_mat;
  cv::eigen2cv( pts_set_float, pts_set_mat );

  MiscUtils::plot_point_sets( im, pts_set_mat, im, color, enable_keypoint_annotation, msg );
}

// TODO: call the next function instead of actually doing the work.
void MiscUtils::plot_point_sets( const cv::Mat& im, const cv::Mat& pts_set, cv::Mat& dst, const cv::Scalar& color, bool enable_keypoint_annotation, const string& msg )
{

  if( im.data == dst.data ) { // this is pointer comparison to know if this operation is inplace
    //   cout << "src and dst are same\n";
      // src and dst images are same, so dont copy. just ensure it is a 3 channel image.
      assert( im.rows > 0 && im.cols > 0 && "\n[MiscUtils::plot_point_sets]Image appears to be emoty. cannot plot.\n");
      assert( im.channels() == 3 && dst.channels() == 3 && "[MiscUtils::plot_point_sets]src and dst image are same physical image in memory. They need to be 3 channel." );
  }
  else {
    //   dst = cv::Mat( im.rows, im.cols, CV_8UC3 );
      if( im.channels() == 1 )
        cv::cvtColor( im, dst, cv::COLOR_GRAY2BGR );
      else
        im.copyTo(dst);
  }

  // cv::putText( dst, to_string(msg.length()), cv::Point(5,5), cv::FONT_HERSHEY_COMPLEX_SMALL, .95, cv::Scalar(0,255,255) );
  if( msg.length() > 0 ) {
    vector<std::string> msg_split;
    msg_split = MiscUtils::split( msg, ';' );
    for( int q=0 ; q<msg_split.size() ; q++ )
      cv::putText( dst, msg_split[q], cv::Point(5,20+20*q), cv::FONT_HERSHEY_COMPLEX_SMALL, .95, cv::Scalar(0,255,255) );
  }


  //pts_set is 2xN
  cv::Point2d pt;
  for( int i=0 ; i<pts_set.cols ; i++ )
  {
    pt = cv::Point2d(pts_set.at<float>(0,i),pts_set.at<float>(1,i) );
    cv::circle( dst, pt, 2, color, -1 );

    char to_s[20];
    sprintf( to_s, "%d", i);
    if( enable_keypoint_annotation ) {
        // cv::putText( dst, to_s, pt, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255,255,255) - color  );
        cv::putText( dst, to_s, pt, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, color  );
    }

  }
}


void MiscUtils::plot_point_sets( cv::Mat& im, const MatrixXd& pts_set,
                                        const cv::Scalar& color, const VectorXi& annotations, const string& msg )
{
    plot_point_sets( im, pts_set, im, color, annotations, msg );
}


void MiscUtils::plot_point_sets( cv::Mat& im, const MatrixXd& pts_set, cv::Mat& dst,
                                        const cv::Scalar& color, const VectorXi& annotations, const string& msg )

{
  // TODO consider addressof(a) == addressof(b)
  // dst = im.clone();


  assert( im.rows > 0 && im.cols > 0 && "\n[MiscUtils::plot_point_sets]Image appears to be emoty. cannot plot.\n");
  assert( pts_set.cols() > 0 && pts_set.cols() == annotations.rows() && "[MiscUtils::plot_point_sets] VectorXi annotation size must be equal to number of points. If you wish to use the default annotation ie. 0,1,...n use `true` instead. If you do not want annotation use `false`." );
  if( im.data == dst.data ) {
    //   cout << "src and dst are same\n";
      // src and dst images are same, so dont copy. just ensure it is a 3 channel image.
      assert( im.channels() == 3 && dst.channels() == 3 && "[MiscUtils::plot_point_sets]src and dst image are same physical image in memory. They need to be 3 channel." );
  }
  else {
    //   dst = cv::Mat( im.rows, im.cols, CV_8UC3 );
      if( im.channels() == 1 )
        cv::cvtColor( im, dst, cv::COLOR_GRAY2BGR );
      else
        im.copyTo(dst);
  }

  // cv::putText( dst, to_string(msg.length()), cv::Point(5,5), cv::FONT_HERSHEY_COMPLEX_SMALL, .95, cv::Scalar(0,255,255) );
  if( msg.length() > 0 ) {
    vector<std::string> msg_split;
    msg_split = MiscUtils::split( msg, ';' );
    for( int q=0 ; q<msg_split.size() ; q++ )
      cv::putText( dst, msg_split[q], cv::Point(5,20+20*q), cv::FONT_HERSHEY_COMPLEX_SMALL, .95, cv::Scalar(0,255,255) );
  }


  //pts_set is 2xN
  cv::Point2d pt;
  for( int i=0 ; i<pts_set.cols() ; i++ )
  {
    // pt = cv::Point2d(pts_set.at<float>(0,i),pts_set.at<float>(1,i) );
    pt = cv::Point2d( (float)pts_set(0,i), (float)pts_set(1,i) );
    cv::circle( dst, pt, 2, color, -1 );

    char to_s[20];
    sprintf( to_s, "%d", annotations(i) );
    cv::putText( dst, to_s, pt, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, color  );


  }
}


void MiscUtils::plot_point_sets( const cv::Mat& im, const MatrixXd& pts_set, cv::Mat& dst,
                                        vector<cv::Scalar>& color_annotations, float alpha, const string& msg )
{

      assert( im.rows > 0 && im.cols > 0 && "\n[MiscUtils::plot_point_sets]Image appears to be emoty. cannot plot.\n");
      assert( pts_set.cols() > 0 && pts_set.cols() == color_annotations.size() && "[MiscUtils::plot_point_sets] len of color vector must be equal to number of pts.\n" );
      if( im.data == dst.data ) {
        //   cout << "src and dst are same\n";
          // src and dst images are same, so dont copy. just ensure it is a 3 channel image.
          assert( im.channels() == 3 && dst.channels() == 3 && "[MiscUtils::plot_point_sets]src and dst image are same physical image in memory. They need to be 3 channel." );
      }
      else {
        //   dst = cv::Mat( im.rows, im.cols, CV_8UC3 );
          if( im.channels() == 1 )
            cv::cvtColor( im, dst, cv::COLOR_GRAY2BGR );
          else
            im.copyTo(dst);
      }

      // cv::putText( dst, to_string(msg.length()), cv::Point(5,5), cv::FONT_HERSHEY_COMPLEX_SMALL, .95, cv::Scalar(0,255,255) );
      if( msg.length() > 0 ) {
        vector<std::string> msg_split;
        msg_split = MiscUtils::split( msg, ';' );
        for( int q=0 ; q<msg_split.size() ; q++ )
          cv::putText( dst, msg_split[q], cv::Point(5,20+20*q), cv::FONT_HERSHEY_COMPLEX_SMALL, .95, cv::Scalar(0,255,255) );
      }


      //pts_set is 2xN
      cv::Point2d pt;
      int n_outside_image_domain = 0;
      for( int i=0 ; i<pts_set.cols() ; i++ )
      {
          if(  pts_set(0,i) < 0 || pts_set(0,i) > im.cols  ||  pts_set(1,i) < 0 || pts_set(1,i) > im.rows   ) { // avoid points which are outside
              n_outside_image_domain++;
              continue;
        }
        pt = cv::Point( (int)pts_set(0,i), (int)pts_set(1,i) );


        cv::Scalar _color = color_annotations[i];
        dst.at< cv::Vec3b >( pt )[0] = (uchar) ( (1.0-alpha)*(float)dst.at< cv::Vec3b >( pt )[0] + (alpha)*(float)_color[0] );
        dst.at< cv::Vec3b >( pt )[1] = (uchar) ( (1.0-alpha)*(float)dst.at< cv::Vec3b >( pt )[1] + (alpha)*(float)_color[1] );
        dst.at< cv::Vec3b >( pt )[2] = (uchar) ( (1.0-alpha)*(float)dst.at< cv::Vec3b >( pt )[2] + (alpha)*(float)_color[2] );

        // cv::Vec3d new_color( .5*_orgcolor[0]+.5*_color[0] + .5*_orgcolor[1]+.5*_color[1] + .5*_orgcolor[2]+.5*_color[2] )
      }

      if( float(n_outside_image_domain)/ float(pts_set.cols() ) > 0.2 ) // print only if u see too many outside the image
          cout << "[MiscUtils::plot_point_sets] with color at every point, found " << n_outside_image_domain << " outside the image of total points to plot=" << pts_set.cols() << "\n";
}



void MiscUtils::plot_point_pair( const cv::Mat& imA, const MatrixXd& ptsA, int idxA,
                      const cv::Mat& imB, const MatrixXd& ptsB, int idxB,
                    //   const VectorXd& mask,
                    cv::Mat& dst,
                    const cv::Scalar& color_marker,
                    const cv::Scalar& color_line,
                      bool annotate_pts,
                      /*const vector<string>& msg,*/
                      const string& msg
                    )
{
  // ptsA : ptsB : 2xN or 3xN

  assert( imA.rows == imB.rows && imA.rows > 0  );
  assert( imA.cols == imB.cols && imA.cols > 0  );
  assert( ptsA.cols() == ptsB.cols() && ptsA.cols() > 0 );
  // assert( mask.size() == ptsA.cols() );

  cv::Mat outImg_;
  cv::hconcat(imA, imB, outImg_);

    cv::Mat outImg;
    if( outImg_.channels() == 3 )
        outImg = outImg_;
    else
        cv::cvtColor( outImg_, outImg, CV_GRAY2BGR );




  // loop over all points
  int count = 0;
  for( int kl=0 ; kl<ptsA.cols() ; kl++ )
  {
    // if( mask(kl) == 0 )
    //   continue;

    count++;
    cv::Point2d A( ptsA(0,kl), ptsA(1,kl) );
    cv::Point2d B( ptsB(0,kl), ptsB(1,kl) );

    cv::circle( outImg, A, 2,color_marker, -1 );
    cv::circle( outImg, B+cv::Point2d(imA.cols,0), 2,color_marker, -1 );

    cv::line( outImg,  A, B+cv::Point2d(imA.cols,0), color_line );

    if( annotate_pts )
    {
      cv::putText( outImg, to_string(kl), A, cv::FONT_HERSHEY_SIMPLEX, 0.3, color_marker, 1 );
      cv::putText( outImg, to_string(kl), B+cv::Point2d(imA.cols,0), cv::FONT_HERSHEY_SIMPLEX, 0.3, color_marker, 1 );
    }
  }



  cv::Mat status = cv::Mat(150, outImg.cols, CV_8UC3, cv::Scalar(0,0,0) );
  cv::putText( status, to_string(idxA).c_str(), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 2 );
  cv::putText( status, to_string(idxB).c_str(), cv::Point(imA.cols+10,30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 2 );
  cv::putText( status, "marked # pts: "+to_string(count), cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 1.5 );


  // put msg in status image
  if( msg.length() > 0 ) { // ':' separated. Each will go in new line
      std::vector<std::string> msg_tokens = split(msg, ';');
      for( int h=0 ; h<msg_tokens.size() ; h++ )
          cv::putText( status, msg_tokens[h].c_str(), cv::Point(10,80+20*h), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 1.5 );
  }


  cv::vconcat( outImg, status, dst );

}

cv::Scalar MiscUtils::getFalseColor( float f )
{
    cv::Mat colormap_gray = cv::Mat::zeros( 1, 256, CV_8UC1 );
    cv::Mat colormap_color;
    for( int i=0 ; i<256; i++ ) colormap_gray.at<uchar>(0,i) = i;
    cv::applyColorMap(colormap_gray, colormap_color, cv::COLORMAP_JET	);
    if( f<0 ) {
        cv::Vec3b f_ = colormap_color.at<cv::Vec3b>(0,  (int)0 );
        cv::Scalar color_marker = cv::Scalar(f_[0],f_[1],f_[2]);
        return color_marker;
    }
    if( f>1. ) {
        cv::Vec3b f_ = colormap_color.at<cv::Vec3b>(0,  (int)255 );
        cv::Scalar color_marker = cv::Scalar(f_[0],f_[1],f_[2]);
        return color_marker;
    }

    int idx = (int) (f*255.);
    cv::Vec3b f_ = colormap_color.at<cv::Vec3b>(0,  (int)idx );
    cv::Scalar color_marker = cv::Scalar(f_[0],f_[1],f_[2]);
    return color_marker;
}

void MiscUtils::plot_point_pair( const cv::Mat& imA, const MatrixXd& ptsA, int idxA,
                      const cv::Mat& imB, const MatrixXd& ptsB, int idxB,
                      cv::Mat& dst,
                      short color_map_direction,
                      const string& msg
                     )
{
  // ptsA : ptsB : 2xN or 3xN
  assert( color_map_direction >= 0 && color_map_direction<=3 );
  assert( imA.rows == imB.rows && imA.rows > 0  );
  assert( imA.cols == imB.cols && imA.cols > 0  );
  assert( ptsA.cols() == ptsB.cols() && ptsA.cols() > 0 );
  // assert( mask.size() == ptsA.cols() );


  // make colormap
  cv::Mat colormap_gray = cv::Mat::zeros( 1, 256, CV_8UC1 );
  for( int i=0 ; i<256; i++ ) colormap_gray.at<uchar>(0,i) = i;
  cv::Mat colormap_color;
  cv::applyColorMap(colormap_gray, colormap_color, cv::COLORMAP_HSV);



  cv::Mat outImg_;
  cv::hconcat(imA, imB, outImg_);

  cv::Mat outImg;
  if( outImg_.channels() == 3 )
      outImg = outImg_;
  else
      cv::cvtColor( outImg_, outImg, CV_GRAY2BGR );





  // loop over all points
  int count = 0;
  for( int kl=0 ; kl<ptsA.cols() ; kl++ )
  {
    // if( mask(kl) == 0 )
    //   continue;

    count++;
    cv::Point2d A( ptsA(0,kl), ptsA(1,kl) );
    cv::Point2d B( ptsB(0,kl), ptsB(1,kl) );

    int coloridx;
    if( color_map_direction==0 ) coloridx = (int) (ptsA(0,kl)/imA.cols*256.); // horizontal-gradiant
    if( color_map_direction==1 ) coloridx = (int) (ptsA(1,kl)/imA.rows*256.); // vertical-gradiant
    if( color_map_direction==2 ) coloridx = (int) (   ( ptsA(0,kl) + ptsA(1,kl) ) / (imA.rows + imA.cols )*256.  ); // manhattan-gradiant
    if( color_map_direction==3 ) coloridx = (int) (   abs( ptsA(0,kl) - imA.rows/2. + ptsA(1,kl) - imA.cols/2. ) / (imA.rows/2. + imA.cols/2. )*256.  ); // image centered manhattan-gradiant
    if( coloridx<0 || coloridx>255 ) coloridx=0;
    cv::Vec3b f = colormap_color.at<cv::Vec3b>(0,  (int)coloridx );
    cv::Scalar color_marker = cv::Scalar(f[0],f[1],f[2]);

    cv::circle( outImg, A, 2,color_marker, -1 );
    cv::circle( outImg, B+cv::Point2d(imA.cols,0), 2,color_marker, -1 );

    /*
    cv::line( outImg,  A, B+cv::Point2d(imA.cols,0), color_line );

    if( annotate_pts )
    {
      cv::putText( outImg, to_string(kl), A, cv::FONT_HERSHEY_SIMPLEX, 0.3, color_marker, 1 );
      cv::putText( outImg, to_string(kl), B+cv::Point2d(imA.cols,0), cv::FONT_HERSHEY_SIMPLEX, 0.3, color_marker, 1 );
    }
    */
  }



  cv::Mat status = cv::Mat(150, outImg.cols, CV_8UC3, cv::Scalar(0,0,0) );
  cv::putText( status, to_string(idxA).c_str(), cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 2 );
  cv::putText( status, to_string(idxB).c_str(), cv::Point(imA.cols+10,30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 2 );
  cv::putText( status, "marked # pts: "+to_string(count), cv::Point(10,60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 1.5 );


  // put msg in status image
  if( msg.length() > 0 ) { // ':' separated. Each will go in new line
      std::vector<std::string> msg_tokens = split(msg, ';');
      for( int h=0 ; h<msg_tokens.size() ; h++ )
          cv::putText( status, msg_tokens[h].c_str(), cv::Point(10,80+20*h), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 1.5 );
  }


  cv::vconcat( outImg, status, dst );

}


// append a status image . ';' separated
void MiscUtils::append_status_image( cv::Mat& im, const string& msg, float txt_size, cv::Scalar bg_color, cv::Scalar txt_color, float line_thinkness )
{
    bool is_single_channel = (im.channels()==1)?true:false;
    txt_size = (txt_size<0.1 || txt_size>2)?0.4:txt_size;

    std::vector<std::string> msg_tokens = split(msg, ';');
    int status_im_height = 50+20*msg_tokens.size();

    cv::Mat status;
    if( is_single_channel )
        status = cv::Mat(status_im_height, im.cols, CV_8UC1, cv::Scalar(0,0,0) );
    else
        status = cv::Mat(status_im_height, im.cols, CV_8UC3, bg_color );


    for( int h=0 ; h<msg_tokens.size() ; h++ )
        cv::putText( status, msg_tokens[h].c_str(), cv::Point(10,20+20*h),
                cv::FONT_HERSHEY_SIMPLEX,
                txt_size, txt_color, line_thinkness );


    cv::vconcat( im, status, im );


}

bool MiscUtils::side_by_side( const cv::Mat& A, const cv::Mat& B, cv::Mat& dst )
{
    if( A.rows == B.rows && A.channels() == B.channels() ) {
        cv::hconcat(A, B, dst);
        return true;
    }
    else {
        dst = cv::Mat();
        return false;
    }
}

bool MiscUtils::vertical_side_by_side( const cv::Mat& A, const cv::Mat& B, cv::Mat& dst )
{
    if( A.cols == B.cols && A.channels() == B.channels() ) {
        cv::vconcat(A, B, dst);
        return true;
    }
    else {
        dst = cv::Mat();
        return false;
    }
}

double MiscUtils::Slope(int x0, int y0, int x1, int y1){
     return (double)(y1-y0)/(x1-x0);
}

void MiscUtils::draw_fullLine(cv::Mat& img, cv::Point2f a, cv::Point2f b, cv::Scalar color){
     double slope = MiscUtils::Slope(a.x, a.y, b.x, b.y);

     cv::Point2f p(0,0), q(img.cols,img.rows);

     p.y = -(a.x - p.x) * slope + a.y;
     q.y = -(b.x - q.x) * slope + b.y;

     cv::line(img,p,q,color,1,8,0);
}

// draw line on the image, given a line equation in homogeneous co-ordinates. l = (a,b,c) for ax+by+c = 0
void MiscUtils::draw_line( const Vector3d l, cv::Mat& im, cv::Scalar color )
{
    // C++: void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
    if( l(0) == 0 ) {
        // plot y = -c/b
        cv::Point2f a(0.0, -l(2)/l(1) );
        cv::Point2f a_(10.0, -l(2)/l(1) );
        MiscUtils::draw_fullLine( im, a, a_, color );
        return;
    }

    if( l(1) == 0 ) {
        // plot x = -c/a
        cv::Point2f b(-l(2)/l(0), 0.0 );
        cv::Point2f b_(-l(2)/l(0), 10.0 );
        MiscUtils::draw_fullLine( im, b, b_, color );
        return;
    }

    cv::Point2f a(0.0, -l(2)/l(1) );
    cv::Point2f b(-l(2)/l(0), 0.0 );
    // cout << a << "<--->" << b << endl;
    // cv::line( im, a, b, cv::Scalar(255,255,255) );
    MiscUtils::draw_fullLine( im, a, b, color );
}

// mark point on the image, pt is in homogeneous co-ordinate.
void MiscUtils::draw_point( const Vector3d pt, cv::Mat& im, cv::Scalar color  )
{
    // C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
    cv::circle( im, cv::Point2f( pt(0)/pt(2), pt(1)/pt(2) ), 2, color, -1   );

}

// mark point on image
void MiscUtils::draw_point( const Vector2d pt, cv::Mat& im, cv::Scalar color  )
{
    cv::circle( im, cv::Point2f( pt(0), pt(1) ), 2, color, -1   );
}
