#include "CameraGeometry.h"

MonoGeometry::MonoGeometry(camodocal::CameraPtr _camera)
{
    assert( _camera && "Abstract Camera is not set. You need to init camera before setting it to geometry clases" );
    if( !_camera ) {
        cout << "Abstract Camera is not set. You need to init camera before setting it to geometry clases\n";
        exit(10);
    }
    this->camera = _camera;

    // set K_new as default
    GeometryUtils::getK( this->camera, this->K_new );
    this->new_fx = K_new(0,0); // 375.;
    this->new_fy = K_new(1,1); //375.;
    this->new_cx = K_new(0,2); //376.;
    this->new_cy = K_new(1,2); //240.;

    // make rectification maps. Remember to remake the maps when set_K is called
    this->camera->initUndistortRectifyMap( map_x, map_y, new_fx, new_fy, cv::Size(0,0), new_cx, new_cy );
}


void MonoGeometry::set_K( Matrix3d K )
{
    std::lock_guard<std::mutex> lk(m);
    this->K_new = K;
    this->new_fx = K_new(0,0); // 375.;
    this->new_fy = K_new(1,1); //375.;
    this->new_cx = K_new(0,2); //376.;
    this->new_cy = K_new(1,2); //240.;

    // once a new K is set will have to recompute maps.
    this->camera->initUndistortRectifyMap( map_x, map_y,
                                    new_fx, new_fy, cv::Size(0,0),
                                    new_cx, new_cy );
}

void MonoGeometry::do_image_undistortion( const cv::Mat& im_raw, cv::Mat & im_undistorted )
{
    std::lock_guard<std::mutex> lk(m);
    cv::remap( im_raw, im_undistorted, map_x, map_y, CV_INTER_LINEAR );
}

//-------------------------------------------------------------------------------------//

StereoGeometry::StereoGeometry( camodocal::CameraPtr _left_camera,
                camodocal::CameraPtr _right_camera,
                Matrix4d __right_T_left )
{
    if( !_left_camera || !_right_camera )
    {
        cout << "ERROR : Abstract stereo Camera is not set. You need to init camera before setting it to geometry clases";
        exit(10);
    }
    assert( _left_camera && _right_camera  && "Abstract stereo Camera is not set. You need to init camera before setting it to geometry clases" );

    this->camera_left = _left_camera;
    this->camera_right = _right_camera;
    // this->right_T_left = Matrix4d( __right_T_left ); //stereo extrinsic. relative pose between two pairs.
    this->set_stereoextrinsic( __right_T_left  ); //stereo extrinsic. relative pose between two pairs.

    // set K_new from left_camera's intrinsic
    Matrix3d __K_new;
    GeometryUtils::getK( this->camera_left, __K_new );
    this->set_K( __K_new );


    left_geom = std::make_shared<MonoGeometry>( this->camera_left );
    right_geom = std::make_shared<MonoGeometry>( this->camera_right );
    left_geom->set_K( this->K_new );
    right_geom->set_K( this->K_new ); // it is really important that you have same K_new for each of the images in the stereo-pair, this is not a mistake.

    // stereo rectification maps
    // theory : http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO/tutorial.html
    // make_stereo_rectification_maps() // no need to call this again as it is already called when u either do set_K() or set_stereoextrinsic().

    // init SGBM
    // TODO Can put in options to the BM algorithm, look at opencv docs to set the options
    // like blocksize, numDisparities etc etc.
    bm = cv::StereoBM::create(64,21);
    sgbm = cv::StereoSGBM::create();

}

void StereoGeometry::print_blockmatcher_algo_info()
{
    cout << TermColor::RED();
    cout << "name = " << bm->getDefaultName() << endl;
    cout << "getNumDisparities=" << bm->getNumDisparities() << endl;
    cout << "getMinDisparity=" << bm->getMinDisparity() << endl;
    cout << "getBlockSize=" << bm->getBlockSize() << endl;


    cout << TermColor::RESET() << endl;
}



void StereoGeometry::set_stereoextrinsic( Matrix4d __right_T_left )
{
    std::lock_guard<std::mutex> lk(m_extrinsics);
    this->right_T_left = Matrix4d( __right_T_left );
    make_stereo_rectification_maps();
}

void StereoGeometry::set_stereoextrinsic(Vector4d quat_xyzw, Vector3d tr_xyz )
{
    std::lock_guard<std::mutex> lk(m_extrinsics);
    Matrix4d transform ;  //right_T_left
    PoseManipUtils::raw_xyzw_to_eigenmat( quat_xyzw, tr_xyz, transform );
    this->right_T_left = Matrix4d( transform );
    make_stereo_rectification_maps();
}

const Matrix4d& StereoGeometry::get_stereoextrinsic()
{
    std::lock_guard<std::mutex> lk(m_extrinsics);
    return this->right_T_left;
}

void StereoGeometry::fundamentalmatrix_from_stereoextrinsic( Matrix3d& F )
{
    Matrix3d Tx, _Rot;
    {
    std::lock_guard<std::mutex> lk2(m_extrinsics);
    PoseManipUtils::vec_to_cross_matrix( right_T_left.col(3).topRows(3), Tx );
    _Rot = right_T_left.topLeftCorner(3,3);
    }

    // The actual formula is : inverse(K_right)' * Tx * R * inverse(K_left)
    // but we use the same K_new for both. This is a subtle point here.
    F = this->get_K().transpose().inverse() * Tx * _Rot * this->get_K().inverse();  //< Fundamental Matrix
}


void StereoGeometry::draw_epipolarlines( cv::Mat& imleft_undistorted, cv::Mat& imright_undistorted )
{
    IOFormat numpyFmt(FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");

    // if image is 1 channel make it to 3 channel so that I can plot colored lines and points.
    cv::Mat imleft_undistorted_3chnl, imright_undistorted_3chnl;
    if( imleft_undistorted.channels() == 1 )
        cv::cvtColor(imleft_undistorted, imleft_undistorted_3chnl, CV_GRAY2BGR );
    else
        imleft_undistorted_3chnl = imleft_undistorted;

    if( imright_undistorted.channels() == 1 )
        cv::cvtColor(imright_undistorted, imright_undistorted_3chnl, CV_GRAY2BGR);
    else
        imright_undistorted_3chnl = imright_undistorted;

    imleft_undistorted = imleft_undistorted_3chnl;
    imright_undistorted = imright_undistorted_3chnl;


    // Fundamental Matrix
    Matrix3d F;
    this->fundamentalmatrix_from_stereoextrinsic( F );
    cout << "[StereoGeometry::draw_epipolarlines]F=" << F.format(numpyFmt) << endl;



    // tryout multiple points and get their corresponding epipolar line.
    for( int i=0 ; i<500; i+=20 ) {
    #if 1
    // take a sample point x on left image and find the corresponding line on the right image
    Vector3d x(1.5*i, i, 1.0);
    Vector3d ld = F * x;
    MiscUtils::draw_point( x, imleft_undistorted, cv::Scalar(255,0,0) );
    MiscUtils::draw_line( ld, imright_undistorted, cv::Scalar(255,0,0) );
    #endif

    #if 1
    // take a sample point on right image and find the corresponding line on the left image
    Vector3d xd(i, i, 1.0);
    Vector3d l = F.transpose() * xd;
    MiscUtils::draw_line( l, imleft_undistorted, cv::Scalar(0,0,255) );
    MiscUtils::draw_point( xd, imright_undistorted, cv::Scalar(0,0,255) );
    #endif
    }

}


void StereoGeometry::draw_srectified_epipolarlines( cv::Mat& imleft_srectified, cv::Mat& imright_srectified )
{
    IOFormat numpyFmt(FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");

    // if image is 1 channel make it to 3 channel so that I can plot colored lines and points.
    cv::Mat imleft_srectified_3chnl, imright_srectified_3chnl;
    if( imleft_srectified.channels() == 1 )
        cv::cvtColor(imleft_srectified, imleft_srectified_3chnl, CV_GRAY2BGR );
    else
        imleft_srectified_3chnl = imleft_srectified;

    if( imright_srectified.channels() == 1 )
        cv::cvtColor(imright_srectified, imright_srectified_3chnl, CV_GRAY2BGR);
    else
        imright_srectified_3chnl = imright_srectified;

    imleft_srectified = imleft_srectified_3chnl;
    imright_srectified = imright_srectified_3chnl;




    Matrix3d F = rm_fundamental_matrix; //< This was computed by make_stereo_rectification_maps.
    // this->fundamentalmatrix_from_stereoextrinsic( F );
    cout << "[StereoGeometry::draw_srectified_epipolarlines]F=" << F.format(numpyFmt) << endl;



    // tryout multiple points and get their corresponding epipolar line.
    for( int i=0 ; i<500; i+=20 ) {
    #if 1
    // take a sample point x on left image and find the corresponding line on the right image
    Vector3d x(1.5*i, i, 1.0);
    Vector3d ld = F * x;
    MiscUtils::draw_point( x, imleft_srectified, cv::Scalar(255,0,0) );
    MiscUtils::draw_line( ld, imright_srectified, cv::Scalar(255,0,0) );
    #endif

    #if 1
    // take a sample point on right image and find the corresponding line on the left image
    Vector3d xd(i, i, 1.0);
    Vector3d l = F.transpose() * xd;
    MiscUtils::draw_line( l, imleft_srectified, cv::Scalar(0,0,255) );
    MiscUtils::draw_point( xd, imright_srectified, cv::Scalar(0,0,255) );
    #endif
    }

}


void StereoGeometry::set_K( Matrix3d K )
{
    std::lock_guard<std::mutex> lk(m_intrinsics);

    this->K_new = K;
    this->new_fx = K_new(0,0); // 375.;
    this->new_fy = K_new(1,1); //375.;
    this->new_cx = K_new(0,2); //376.;
    this->new_cy = K_new(1,2); //240.;
    make_stereo_rectification_maps();
}

void StereoGeometry::set_K( float _fx, float _fy, float _cx, float _cy )
{
    std::lock_guard<std::mutex> lk(m_intrinsics);
    this->K_new << _fx, 0.0, _cx,
                    0.0, _fy, _cy,
                    0.0, 0.0, 1.0;

    this->new_fx = K_new(0,0); // 375.;
    this->new_fy = K_new(1,1); //375.;
    this->new_cx = K_new(0,2); //376.;
    this->new_cy = K_new(1,2); //240.;
    make_stereo_rectification_maps();
}

const Matrix3d& StereoGeometry::get_K()
{
    std::lock_guard<std::mutex> lk(m_intrinsics);
    return this->K_new;
}


// #define __StereoGeometry___make_stereo_rectification_maps(msg) msg;
#define __StereoGeometry___make_stereo_rectification_maps(msg) ;
void StereoGeometry::make_stereo_rectification_maps()
{
    cv::Size imsize = cv::Size( camera_left->imageWidth(), camera_left->imageHeight());

    IOFormat numpyFmt(FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");


    // Adopted from : http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO/node18.html
    __StereoGeometry___make_stereo_rectification_maps( cout << "[compute_stereo_rectification_transform]" << endl );





    Matrix3d right_R_left = this->right_T_left.topLeftCorner(3,3);
    Vector3d right_t_left = this->right_T_left.col(3).topRows(3);
    cv::Mat R, T;
    cv::eigen2cv( right_R_left, R );
    cv::eigen2cv( right_t_left, T );


    cv::Mat K_new_cvmat;
    cv::eigen2cv( this->K_new, K_new_cvmat );

    // TODO: If need be write getters for these matrices.
    // cv::Mat R1, R2;
    // cv::Mat P1, P2;
    // cv::Mat Q;

    cv::Mat D;
    // !! Main Call OpenCV !! //
    /// The below function does exactly this: http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO/node18.html
    // See OpenCVdoc: https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga617b1685d4059c6040827800e72ad2b6
    cv::stereoRectify( K_new_cvmat, D, K_new_cvmat, D, imsize, R, T,
                        rm_R1, rm_R2, rm_P1, rm_P2, rm_Q
                    );

    __StereoGeometry___make_stereo_rectification_maps(
    cout << "\tK_new=" << cv::format(K_new_cvmat, cv::Formatter::FMT_NUMPY)  << endl;
    cout << "\tR1=" << cv::format(rm_R1, cv::Formatter::FMT_NUMPY) << endl;
    cout << "\tR2=" << cv::format(rm_R2, cv::Formatter::FMT_NUMPY) << endl;
    cout << "\tP1=" << cv::format(rm_P1, cv::Formatter::FMT_NUMPY) << endl;
    cout << "\tP2=" << cv::format(rm_P2, cv::Formatter::FMT_NUMPY) << endl;
    cout << "\tQ=" << cv::format(rm_Q, cv::Formatter::FMT_NUMPY) << endl;
    )
    // cout << TermColor::RESET();

    // For horizontal stereo :
    // P2(0,3) := Tx * f, where f = P2(0,0) and Tx is the position of left camera as seen by the right camera. right_T_left.
    double Tx = rm_P2.at<double>(0,3) / rm_P2.at<double>(0,0);
    rm_shift = Vector3d( Tx, 0, 0);
    __StereoGeometry___make_stereo_rectification_maps(cout << "\tTx = " << Tx <<  " Tx is the position of left camera as seen by the right camera. right_T_left" << endl);
    Matrix3d TTx;
    PoseManipUtils::vec_to_cross_matrix( rm_shift, TTx );
    rm_fundamental_matrix = K_new.transpose().inverse() * TTx  * K_new.inverse();
    __StereoGeometry___make_stereo_rectification_maps(cout << "F_rect=" << rm_fundamental_matrix.format(numpyFmt) << endl);


    // For vertical stereo : TODO
    // P2(1,3) := Ty * f, where f = P2(0,0) and Tx is the horizontal shift between cameras
    // double Ty = P2(1,3) / P2(0,0);

    __StereoGeometry___make_stereo_rectification_maps(cout << "[/compute_stereo_rectification_transform]" << endl);



    // !! Make Stereo Rectification Maps !! //
    __StereoGeometry___make_stereo_rectification_maps(cout << "[make_rectification_maps]\n");
    cv::initUndistortRectifyMap( K_new_cvmat, D, rm_R1, rm_P1, imsize, CV_32FC1, map1_x, map1_y );
    cv::initUndistortRectifyMap( K_new_cvmat, D, rm_R2, rm_P2, imsize, CV_32FC1, map2_x, map2_y );
    __StereoGeometry___make_stereo_rectification_maps(
    cout << "\tmap1_x: " << MiscUtils::cvmat_info( map1_x ) << endl;
    cout << "\tmap1_y: " << MiscUtils::cvmat_info( map1_y ) << endl;
    cout << "\tmap2_x: " << MiscUtils::cvmat_info( map2_x ) << endl;
    cout << "\tmap2_y: " << MiscUtils::cvmat_info( map2_y ) << endl;
    cout << "[/make_rectification_maps]\n");

}

//-------------------------- Undistort Raw Image ----------------------------------------//

void StereoGeometry::do_image_undistortion( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                            cv::Mat& imleft_undistorted, cv::Mat& imright_undistorted
                        )
{
    std::lock_guard<std::mutex> lk(m_intrinsics);
    // cout << "imleft_raw " << MiscUtils::cvmat_info( imleft_raw ) << endl;
    // cout << "imright_raw " << MiscUtils::cvmat_info( imright_raw ) << endl;

    // this doesn't need a lock as this is not dependent on
    left_geom->do_image_undistortion( imleft_raw, imleft_undistorted );
    right_geom->do_image_undistortion( imright_raw, imright_undistorted );
}

//--------------------------SRectify (Stereo Rectify)----------------------------------------//


void StereoGeometry::do_stereo_rectification_of_undistorted_images(
    const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
    cv::Mat& imleft_srectified, cv::Mat& imright_srectified )
{
    cv::remap( imleft_undistorted, imleft_srectified, this->map1_x, this->map1_y, CV_INTER_LINEAR );
    cv::remap( imright_undistorted, imright_srectified, this->map2_x, this->map2_y, CV_INTER_LINEAR );
}



void StereoGeometry::do_stereo_rectification_of_raw_images(
        const cv::Mat imleft_raw, const cv::Mat imright_raw,
        cv::Mat& imleft_srectified, cv::Mat& imright_srectified )
{
    cv::Mat imleft_undistorted, imright_undistorted;

    // raw --> undistorted
    do_image_undistortion(imleft_raw, imright_raw,
        imleft_undistorted, imright_undistorted
        );

    // undistorted --> stereo rectify
    do_stereo_rectification_of_undistorted_images(imleft_undistorted, imright_undistorted,
        imleft_srectified, imright_srectified
    );
}


//-----------------------------------DISPARITY Computation--------------------------------------//
// stereo rectified --> disparity. Uses the stereo-block matching algorithm,
// ie. cv::StereoBM
// if you use this function, besure to call `do_stereo_rectification_of_undistorted_images`
// or `do_stereo_rectification_of_raw_images` on your images before calling this function
void StereoGeometry::do_stereoblockmatching_of_srectified_images(
    const cv::Mat& imleft_srectified, const cv::Mat& imright_srectified,
    cv::Mat& disparity
)
{
    // sgbm->compute( imleft_srectified, imright_srectified, disparity ); //disparity is CV_16UC1
    bm->compute( imleft_srectified, imright_srectified, disparity ); //disparity is CV_16UC1
    // cout << "[StereoGeometry::do_stereoblockmatching_of_srectified_images] disparity" << MiscUtils::cvmat_info( disparity ) << endl;
}

// raw--> disparity
// internally does:
//      step-1: raw-->undistorted
//      step-2: undistorted--> srectify (stereo rectify)
//      step-3: srectify --> disparity
void StereoGeometry::do_stereoblockmatching_of_raw_images(
    const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
    cv::Mat& disparity
)
{
    cv::Mat imleft_srectified, imright_srectified;
    do_stereo_rectification_of_raw_images( imleft_raw, imright_raw, imleft_srectified, imright_srectified );
    do_stereoblockmatching_of_srectified_images( imleft_srectified, imright_srectified, disparity );
}


// undistorted --> disparity
//      step-1: undistorted --> srectified
//      step-2 : srectified --> disparity
void StereoGeometry::do_stereoblockmatching_of_undistorted_images(
    const cv::Mat& imleft_undistorted, const cv::Mat& imright_undistorted,
    cv::Mat& disparity
)
{
    cv::Mat imleft_srectified, imright_srectified;
    do_stereo_rectification_of_undistorted_images( imleft_undistorted, imright_undistorted, imleft_srectified, imright_srectified );
    do_stereoblockmatching_of_srectified_images( imleft_srectified, imright_srectified, disparity );
}



//----------------------------- DISPARITY and point cloud -------------------------//
// # Inputs
//  disparity_raw : The disparity image
//  fill_eigen_matrix : _3dpts will be allocated and filled only if this flag is true. If this flag is false then eigen matrix is not filled in this function.
//  make_homogeneous : true will result in Eigen matrix being 4xN else will be 3xN. fill_eigen_matrix need to be true for this to matter.
// # Outputs
//  out3d : 3 channel image. X,Y,Z --> ch0, ch1, ch2. Will also contain invalid 3d points.
//  _3dpts : 4xN matrix containing only valid points. N < disparity_raw.shape[0]*disparity_raw.shape[1].
void StereoGeometry::disparity_to_3DPoints(const cv::Mat& disparity_raw,
    cv::Mat& out3D, MatrixXd& _3dpts,
    bool fill_eigen_matrix, bool make_homogeneous )
{
    // Adopted from : https://github.com/bkornel/Reproject-Image-To-3D
    // OpenCV3.3 version has a bug not sure about other version. It produces inf.

    // /\/\/\/\/\/\/\/\/\ ATTEMPT-1 /\/\/\/\/\/\/\/\/\
    //---------------- assemble all the data needed for computation
    const cv::Mat Q = this->get_Q();
    CV_Assert( !disparity_raw.empty() );

    cv::Mat disparity;
    if( disparity_raw.type() != CV_32FC1 )
        disparity_raw.convertTo( disparity, CV_32FC1 );
    else
        disparity = disparity_raw;

	CV_Assert(disparity.type() == CV_32FC1 );
	CV_Assert( Q.cols == 4 && Q.rows == 4);
    CV_Assert( Q.type() == CV_32F || Q.type() == CV_64F );

	// 3-channel matrix for containing the reprojected 3D world coordinates
	out3D = cv::Mat::zeros(disparity.size(), CV_32FC3);

	// Getting the interesting parameters from Q, everything else is zero or one
    float Q03,Q13,Q23,Q32,Q33;

    if( Q.type() == CV_32F ) {
    	Q03 = Q.at<float>(0, 3);
    	Q13 = Q.at<float>(1, 3);
    	Q23 = Q.at<float>(2, 3);
    	Q32 = Q.at<float>(3, 2);
    	Q33 = Q.at<float>(3, 3);
    } else {
        Q03 = (float) Q.at<double>(0, 3);
        Q13 = (float) Q.at<double>(1, 3);
        Q23 = (float) Q.at<double>(2, 3);
        Q32 = (float) Q.at<double>(3, 2);
        Q33 = (float) Q.at<double>(3, 3);
    }


    //--------------------- disparity to 3D points store as 3-channel image
	// Transforming a single-channel disparity map to a 3-channel image representing a 3D surface
	for (int i = 0; i < disparity.rows; i++)
	{
		const float* disp_ptr = disparity.ptr<float>(i);
		cv::Vec3f* out3D_ptr = out3D.ptr<cv::Vec3f>(i);

		for (int j = 0; j < disparity.cols; j++)
		{
			const float pw = 1.0f / (disp_ptr[j]/16. * Q32 + Q33 + 1E-6); // 1E-6 added to avoid inf.

            // ---------------------------------^^^^^
            // it is crucial that you convert the raw_disparity to
            // CV_32F and divide by 16. This is because raw_disparity from StereoBM() is in CV_16SC1
            // (16bit) and a disparity value := NumDisparities*16. so if you have NumDisparities as 64
            // the maximum value in disparity will be 1024. This is a bit weird. So be careful.

			cv::Vec3f& point = out3D_ptr[j];
			point[0] = (static_cast<float>(j)+Q03) * pw;
			point[1] = (static_cast<float>(i)+Q13) * pw;
			point[2] = Q23 * pw;
		}
	}



    // /\/\/\/\/\/\/\/\/\ ATTEMPT-2 /\/\/\/\/\/\/\/\/
    /*
    const cv::Mat Q = this->get_Q();
    cv::Mat Q_32; // opencv's reprojectImageTo3D() need Q to be in CV_32FC1
    Q.convertTo( Q_32, CV_32F );
    cv::Mat disparity_32f;
    disparity_raw.convertTo( disparity_32f, CV_32F );
    disparity_32f *= (1/16.);

    // if using opencv's reprojectImageTo3D() it is crucial that you convert the raw_disparity to
    // CV_32F and divide by 16. This is because raw_disparity from StereoBM() is in CV_16SC1
    // (16bit) and a disparity value := NumDisparities*16. so if you have NumDisparities as 64
    // the maximum value in disparity will be 1024. This is a bit weird. So be careful.

    cout << "Q cvinfo" << MiscUtils::cvmat_info( Q ) << endl;
    cout << "Q_32 cvinfo" << MiscUtils::cvmat_info( Q_32 ) << endl;
    cout << "Q" << Q << endl;
    cout << (  (Q_32.size() == cv::Size(4,4))?"true":"false"  )<< endl;
    cv::reprojectImageTo3D( disparity_32f, out3D, Q_32, true );
    cv::Mat disparity = disparity_raw;
    // END /\/\/\/\/\/\/\/\/\ ATTEMPT-2 /\/\/\/\/\/\/\/\/
    */

    if( fill_eigen_matrix == false )
        return ;


    //--------------------------- Eigen 3D points as 3xN or 4xN
    // TODO: Make this more efficient. If I enable 4xN matrix it takes 8ms/image if I disable it takes 3ms/image.
    // Step-1: loop over out3D to know how how many were valid points. remove inf points and remove behind points (points where disparity was invalid)
    int N = 0;
    for (int i = 0; i < disparity.rows; i++)
	{
		const float* disp_ptr = disparity.ptr<float>(i);
		cv::Vec3f* out3D_ptr = out3D.ptr<cv::Vec3f>(i);

		for (int j = 0; j < disparity.cols; j++)
		{
            cv::Vec3f& point = out3D_ptr[j];
            if( point[2] < 500. && point[2] > 0.0001 ) {
                N++;
            }
        }
    }

    // Step-2: Allocate Memory
    if( make_homogeneous )
    {
        _3dpts = MatrixXd::Zero( 4, N );
        // Step-3: Copy values into Eigen::Matrix of shape 4xN.
        int n=0;
        for (int i = 0; i < disparity.rows; i++)
        {
            const float* disp_ptr = disparity.ptr<float>(i);
            cv::Vec3f* out3D_ptr = out3D.ptr<cv::Vec3f>(i);

            for (int j = 0; j < disparity.cols; j++)
            {
                cv::Vec3f& point = out3D_ptr[j];
                if( point[2] < 500. && point[2] > 0.0001 ) {
                    _3dpts(0,n) = (double)point[0];
                    _3dpts(1,n) = (double)point[1];
                    _3dpts(2,n) = (double)point[2];
                    _3dpts(3,n) = (double)1.0;
                    n++;
                }
            }
        }
    }
    else
    {
        _3dpts = MatrixXd::Zero( 3, N );
        // Step-3: Copy values into Eigen::Matrix of shape 4xN.
        int n=0;
        for (int i = 0; i < disparity.rows; i++)
        {
            const float* disp_ptr = disparity.ptr<float>(i);
            cv::Vec3f* out3D_ptr = out3D.ptr<cv::Vec3f>(i);

            for (int j = 0; j < disparity.cols; j++)
            {
                cv::Vec3f& point = out3D_ptr[j];
                if( point[2] < 500. && point[2] > 0.0001 ) {
                    _3dpts(0,n) = (double)point[0];
                    _3dpts(1,n) = (double)point[1];
                    _3dpts(2,n) = (double)point[2];
                    n++;
                }
            }
        }
    }



}




// _3dpts : 4xN
//      1. raw --> srectified
//      2. srectified --> disparity_raw
//      3. disparity_raw --> 3d points
// return only 3d points
void StereoGeometry::get3dpoints_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                            MatrixXd& _3dpts    )
{
    cv::Mat disp_raw;
    this->do_stereoblockmatching_of_raw_images( imleft_raw, imright_raw, disp_raw );


    const cv::Mat Q = this->get_Q();
    cv::Mat _3dImage;
    this->disparity_to_3DPoints( disp_raw, _3dImage, _3dpts, true, true );

    // cout << "notimplemented\n";
    // exit(11);
}


// returns both 3dpoints and 3dimage
void StereoGeometry::get3dpoints_and_3dmap_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                            MatrixXd& _3dpts, cv::Mat& _3dImage     )
{
    cv::Mat disp_raw;
    this->do_stereoblockmatching_of_raw_images( imleft_raw, imright_raw, disp_raw );


    const cv::Mat Q = this->get_Q();
    this->disparity_to_3DPoints( disp_raw, _3dImage, _3dpts, true, true );

}

// returns both 3dpoints and 3dimage along with srectified image pair
void StereoGeometry::get3dpoints_and_3dmap_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                            MatrixXd& _3dpts, cv::Mat& _3dImage,
                        cv::Mat& imleft_srectified, cv::Mat& imright_srectified     )
{
    // raw --> srectified
    this->do_stereo_rectification_of_raw_images(  imleft_raw, imright_raw, imleft_srectified, imright_srectified );

    // srectified --> disp
    cv::Mat disp_raw;
    this->do_stereoblockmatching_of_srectified_images( imleft_srectified, imright_srectified, disp_raw );


    const cv::Mat Q = this->get_Q();
    this->disparity_to_3DPoints( disp_raw, _3dImage, _3dpts, true, true );

}


void StereoGeometry::get3dpoints_and_disparity_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                            MatrixXd& _3dpts, cv::Mat& disparity_for_visualization    )
{
    cv::Mat disp_raw;
    this->do_stereoblockmatching_of_raw_images( imleft_raw, imright_raw, disp_raw );


    cv::Mat _3dImage;
    this->disparity_to_3DPoints( disp_raw, _3dImage, _3dpts, true, true );

    cv::Mat disparity_for_visualization_gray;
    cv::normalize(disp_raw, disparity_for_visualization_gray, 0, 255, CV_MINMAX, CV_8U); //< disp8 used just for visualization
    cv::applyColorMap(disparity_for_visualization_gray, disparity_for_visualization, cv::COLORMAP_HOT);



    // cout << "notimplemented\n";
    // exit(11);
}



void StereoGeometry::get_srectifiedim_and_3dpoints_and_disparity_from_raw_images(
                    const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                    cv::Mat& imleft_srectified, cv::Mat& imright_srectified,
                    MatrixXd& _3dpts, cv::Mat& disparity_for_visualization    )
{
    // raw --> srectified
    this->do_stereo_rectification_of_raw_images( imleft_raw, imright_raw, imleft_srectified,  imright_srectified );

    // block matching
    cv::Mat disp_raw;
    this->do_stereoblockmatching_of_srectified_images( imleft_srectified, imright_srectified, disp_raw );


    // disparity to 3D points
    cv::Mat _3dImage;
    this->disparity_to_3DPoints( disp_raw, _3dImage, _3dpts, true, true );

    cv::Mat disparity_for_visualization_gray;
    cv::normalize(disp_raw, disparity_for_visualization_gray, 0, 255, CV_MINMAX, CV_8U); //< disp8 used just for visualization
    cv::applyColorMap(disparity_for_visualization_gray, disparity_for_visualization, cv::COLORMAP_HOT);




}

void StereoGeometry::get_srectifiedim_and_3dpoints_and_3dmap_and_disparity_from_raw_images(
                    const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                    cv::Mat& imleft_srectified, cv::Mat& imright_srectified,
                    MatrixXd& _3dpts, cv::Mat& _3dImage, cv::Mat& disparity_for_visualization    )
{
    // raw --> srectified
    this->do_stereo_rectification_of_raw_images( imleft_raw, imright_raw, imleft_srectified,  imright_srectified );

    // block matching
    cv::Mat disp_raw;
    this->do_stereoblockmatching_of_srectified_images( imleft_srectified, imright_srectified, disp_raw );


    // disparity to 3D points
    this->disparity_to_3DPoints( disp_raw, _3dImage, _3dpts, true, true );

    cv::Mat disparity_for_visualization_gray;
    cv::normalize(disp_raw, disparity_for_visualization_gray, 0, 255, CV_MINMAX, CV_8U); //< disp8 used just for visualization
    cv::applyColorMap(disparity_for_visualization_gray, disparity_for_visualization, cv::COLORMAP_HOT);




}


// _3dImage: 3d points as 3 channel image. 1st channel is X, 2nd channel is Y and 3rd channel is Z.
// Also note that these co-ordinates and imleft_raw co-ordinate do not correspond. They correspond to
// the srectified images. Incase you want to use the color info with these 3d points, this
// will lead to wrong. You should compute the srectified images for that.
//      1. raw --> srectified
//      2. srectified --> disparity_raw
//      3. disparity_raw --> 3d points
void StereoGeometry::get3dmap_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                            cv::Mat& _3dImage )
{
    cv::Mat disp_raw;
    this->do_stereoblockmatching_of_raw_images( imleft_raw, imright_raw, disp_raw );


    const cv::Mat Q = this->get_Q();
    MatrixXd _3dpts;
    this->disparity_to_3DPoints( disp_raw, _3dImage, _3dpts, false, true );

    // cout << "notimplemented\n";
    // exit(11);
}


// e_3dImageX, e_3dImageY, e_3dImageZ: cv::split( _3dImage ). same size as imleft_raw.shape.
// Also note that these co-ordinates and imleft_raw co-ordinate do not correspond. They correspond to
// the srectified images. Incase you want to use the color info with these 3d points, this
// will lead to wrong. You should compute the srectified images for that.
//      1. raw --> srectified
//      2. srectified --> disparity_raw
//      3. disparity_raw --> 3d points
void StereoGeometry::get3dmap_from_raw_images( const cv::Mat& imleft_raw, const cv::Mat& imright_raw,
                            MatrixXd& e_3dImageX, MatrixXd& e_3dImageY, MatrixXd& e_3dImageZ  )
{
    // TODO : perhaps return vector<MatrixXd>
    cout << "this is somewhat fragile. I am making a 3 array of matrix and returning it. This is bad. this need to be tested for correctness after this can be used safely.";
    exit(11);

    cv::Mat disp_raw;
    this->do_stereoblockmatching_of_raw_images( imleft_raw, imright_raw, disp_raw );


    const cv::Mat Q = this->get_Q();
    cv::Mat _3dImage;
    MatrixXd _3dpts;
    this->disparity_to_3DPoints( disp_raw, _3dImage, _3dpts, false, true );

    cv::Mat _3dImage_XYZ[3];   //destination array
    cv::split(_3dImage,_3dImage_XYZ);//split source
    cv::cv2eigen( _3dImage_XYZ[0], e_3dImageX );
    cv::cv2eigen( _3dImage_XYZ[1], e_3dImageY );
    cv::cv2eigen( _3dImage_XYZ[2], e_3dImageZ );
}




//-------------------------------------------------------------------------------------//
void GeometryUtils::make_K( float new_fx, float new_fy, float new_cx, float new_cy, Matrix3d& K )
{
    K << new_fx, 0, new_cx,
              0, new_fy, new_cy,
              0, 0, 1;
}





void GeometryUtils::getK( camodocal::CameraPtr m_cam, Matrix3d& K )
{
    Matrix3d K_rect;
    vector<double> m_camera_params;
    m_cam->writeParameters( m_camera_params ); // retrive camera params from Abstract Camera.
    // camodocal::CataCamera::Parameters p();
    // cout << "size=" << m_camera_params.size() << " ::>\n" ;
    // for( int i=0 ; i<m_camera_params.size() ; i++ ) cout << "\t" << i << " " << m_camera_params[i] << endl;

    switch( m_cam->modelType() )
    {
        case camodocal::Camera::ModelType::MEI:
            K_rect << m_camera_params[5], 0, m_camera_params[7],
                      0, m_camera_params[6], m_camera_params[8],
                      0, 0, 1;
            break;
        case camodocal::Camera::ModelType::PINHOLE:
            K_rect << m_camera_params[4], 0, m_camera_params[6],
                      0, m_camera_params[5], m_camera_params[7],
                      0, 0, 1;
            break;
        case camodocal::Camera::ModelType::KANNALA_BRANDT:
            K_rect << m_camera_params[4], 0, m_camera_params[6],
                      0, m_camera_params[5], m_camera_params[7],
                      0, 0, 1;
            break;
            default:
            // TODO: Implement for other models. Look at initUndistortRectifyMap for each of the abstract class.
            cout << "[getK] Wrong\nQuit....";
            exit(10);

    }
    K = Matrix3d(K_rect);
}


// given a point cloud as 3xN or 4xN matrix, gets colors for each based on the depth
void GeometryUtils::depthColors( const MatrixXd& ptcld, vector<cv::Scalar>& out_colors, double min, double max )
{
    assert( ( ptcld.rows() == 3 || ptcld.rows() == 4 ) && "ptcld need to be either a 3xN or a 4xN matrix" );
    assert( ptcld.cols() > 0 );

    cv::Mat colormap_gray = cv::Mat::zeros( 1, 256, CV_8UC1 );
    for( int i=0 ; i<256; i++ ) colormap_gray.at<uchar>(0,i) = i;
    cv::Mat colormap_color;
    cv::applyColorMap(colormap_gray, colormap_color, cv::COLORMAP_HOT);

    if( min < 0 )
        min = ptcld.row(2).minCoeff();
    if( max < 0 )
        max = ptcld.row(2).maxCoeff();
    double mean = ptcld.row(2).mean();
    assert( max > min );

    out_colors.clear();
    for( int k=0 ; k<ptcld.cols() ; k++ ) {
        int ixx = 256.* ( ptcld(2, k ) - min ) / (max - min);

        if( ixx <= 0 ) ixx = 0;
        if( ixx > 255 ) ixx = 255;

        cv::Vec3b f = colormap_color.at<cv::Vec3b>(0,  (int)ixx );
        out_colors.push_back( cv::Scalar(f[0], f[1], f[2]) );
    }

    // cout << "min = " << min << "   max=" << max << "    mean=" << mean << endl;

}


// given a point cloud as 3xN or 4xN matrix, gets colors for each based on the depth.
// return in out_colors as 3xN
void GeometryUtils::depthColors( const MatrixXd& ptcld, MatrixXd& out_colors, double min, double max )
{
    assert( ( ptcld.rows() == 3 || ptcld.rows() == 4 ) && "ptcld need to be either a 3xN or a 4xN matrix" );
    assert( ptcld.cols() > 0 );

    cv::Mat colormap_gray = cv::Mat::zeros( 1, 256, CV_8UC1 );
    for( int i=0 ; i<256; i++ ) colormap_gray.at<uchar>(0,i) = i;
    cv::Mat colormap_color;
    cv::applyColorMap(colormap_gray, colormap_color, cv::COLORMAP_HOT);

    if( min < 0 )
        min = ptcld.row(2).minCoeff();
    if( max < 0 )
        max = ptcld.row(2).maxCoeff();
    double mean = ptcld.row(2).mean();
    assert( max > min );

    out_colors = MatrixXd( 3,ptcld.cols() );
    for( int k=0 ; k<ptcld.cols() ; k++ ) {
        int ixx = 256.* ( ptcld(2, k ) - min ) / (max - min);

        if( ixx <= 0 ) ixx = 0;
        if( ixx > 255 ) ixx = 255;

        cv::Vec3b f = colormap_color.at<cv::Vec3b>(0,  (int)ixx );

        // out_colors.push_back( cv::Scalar(f[0], f[1], f[2]) );
        out_colors( 0, k ) = float(f[2]) / 255.;
        out_colors( 1, k ) = float(f[1]) / 255.;
        out_colors( 2, k ) = float(f[0]) / 255.;
    }

    // cout << "min = " << min << "   max=" << max << "    mean=" << mean << endl;

}


void GeometryUtils::idealProjection( const Matrix3d& K, const MatrixXd& c_X, MatrixXd& uv  )
{
    assert( c_X.rows() == 4 && "c_X need to be expressed in homogeneous co-ordinates\n" );
    assert( c_X.cols() > 0 );

    // a) c_X = c_X / c_X.row(2). ==> Z divide
    // b) perspective_proj = c_X.topRows(3)
    MatrixXd uv_normalized = MatrixXd::Constant( 3, c_X.cols(), 1.0 );
    uv_normalized.row(0) = c_X.row(0).array() / c_X.row(2).array();
    uv_normalized.row(1) = c_X.row(1).array() / c_X.row(2).array();


    // c) uv = K * c_X
    uv = K * uv_normalized;
}
