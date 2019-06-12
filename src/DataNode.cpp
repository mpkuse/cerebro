#include "DataNode.h"


void DataNode::setImageFromMsg( const sensor_msgs::ImageConstPtr msg )
{
    std::lock_guard<std::mutex> lk(m);

    auto tmp = cv_bridge::toCvShare(msg, "bgr8" )->image;

    // this->image = cv_bridge::toCvShare(msg, "bgr8" )->image;
    // this->image = (cv_bridge::toCvCopy(msg, "bgr8" )->image).clone();
    // this->image = (cv_bridge::toCvShare(msg, "bgr8" )->image).clone();

    this->image = tmp.clone();
    tmp.release();


    //
    // Note: (on Opencv and cv_bridge, memory)
    // I was observing a massive memory leak when not using clone().
    // See [this link](https://answers.opencv.org/question/14285/how-to-free-memory-through-cvmat/)
    // So, the point is, if using cv::Mat, it is important to understand who owns the memory.
    // cv::Mat::clone() does a deep copy. it is important to .clone here for
    // the deallocation to work correctly.


    assert( image.rows > 0 && image.cols > 0 && !image.empty() && "In DataNode::setImageFromMsg image that is being set from sensor_msgs::Image is invalid.");


    this->t_image = msg->header.stamp;
    this->m_image = true;
}

void DataNode::setImageFromMsg( const sensor_msgs::ImageConstPtr msg, short cam_id )
{
    std::lock_guard<std::mutex> lk(m);

    // cv::Mat __image = cv_bridge::toCvShare(msg, "bgr8" )->image;
    cv::Mat __image = (cv_bridge::toCvCopy(msg, "bgr8" )->image).clone();
    this->all_images[cam_id] = __image;

    //
    // Note: (on Opencv and cv_bridge, memory)
    // I was observing a massive memory leak when not using clone().
    // See [this link](https://answers.opencv.org/question/14285/how-to-free-memory-through-cvmat/)
    // So, the point is, if using cv::Mat, it is important to understand who owns the memory.
    // cv::Mat::clone() does a deep copy. it is important to .clone here for
    // the deallocation to work correctly.

    assert( this->all_images.at(cam_id).rows > 0 && this->all_images.at(cam_id).cols > 0 && ! this->all_images.at(cam_id).empty() && "In DataNode::setImageFromMsg with cam_id image that is being set from sensor_msgs::Image is invalid.");


    this->t_all_images[cam_id] = msg->header.stamp;

    __image.release();

}

void DataNode::print_image_cvinfo()
{
    std::lock_guard<std::mutex> lk(m);
    cout << "[DataNode::print_image_cvinfo]\n";

    if( m_image ) {
        cout << "[DataNode::print_image_cvinfo] main image: " << MiscUtils::cvmat_info( image ) << endl;
        cout << TermColor::iBLUE() << "[DataNode::print_image_cvinfo] refcount=" << image.u->refcount << TermColor::RESET() << endl;
    }

    for( auto it=all_images.begin() ; it!=all_images.end() ; it++ ) {
        cout << TermColor::iBLUE() << "[DataNode::print_image_cvinfo] image#" << it->first << " refcount=" << it->second.u->refcount << TermColor::RESET() << endl;
    }

    cout << "[DataNode::print_image_cvinfo] Done\n";
}

// #define __DATANODE__deallocate_all_images( msg ) msg;
#define __DATANODE__deallocate_all_images( msg ) ;
void DataNode::deallocate_all_images()
{
    __DATANODE__deallocate_all_images(
    cout << "[DataNode::deallocate_all_images] getT= " << getT() << " getT_image()="<< getT_image() << "\n";
    )
    std::lock_guard<std::mutex> lk(m);

    // deallocate main image
    if( m_image ) {
        image.release();
        image.deallocate();
        t_image = ros::Time();
        m_image = false;
    }
    // cout << "[DataNode::deallocate_all_images] After Deallocate image.u->refcount=" << image.u->refcount << endl;

    // deallocate other images with cam_id
    for( auto it=all_images.begin() ; it!=all_images.end() ; it++ ) {
        it->second.release();
        it->second.deallocate();
        // cout << "[DataNode::deallocate_all_images] image# " << it->first << ", After Deallocate it->second.u->refcount=" << image.u->refcount << endl;
    }
    all_images.clear();
    t_all_images.clear();
}

void DataNode::setPoseFromMsg( const nav_msgs::OdometryConstPtr msg )
{
    std::lock_guard<std::mutex> lk(m);

    PoseManipUtils::geometry_msgs_Pose_to_eigenmat( msg->pose.pose, wTc );

    // vector<double> _q = msg->pose.covariance ;
    wTc_covariance = MatrixXd::Zero(6,6);
    for( int i=0 ; i<36 ; i++ )
        wTc_covariance(int(i/6), i%6) = msg->pose.covariance[i];

    t_wTc = msg->header.stamp;
    m_wTc = true;
}


void DataNode::setPointCloudFromMsg( const sensor_msgs::PointCloudConstPtr msg )
{
    std::lock_guard<std::mutex> lk(m);

    int n_pts = msg->points.size() ;
    if( n_pts == 0 ) {
        // cout << TermColor::RED() << "[DataNode::setPointCloudFromMsg]npts=0" << TermColor::RESET() << endl;
        t_ptcld = msg->header.stamp;
        m_ptcld = true;
        m_ptcld_zero_pts = true;
        return ;
    }
    assert( n_pts > 0 && "[DataNode::setPointCloudFromMsg] The input pointcloud msg contains zero 3d points, ie. msg->points\n");

    ptcld = MatrixXd::Zero( 4, n_pts );
    for( int i=0 ; i<n_pts ; i++ ) {
        ptcld( 0, i ) = msg->points[i].x;
        ptcld( 1, i ) = msg->points[i].y;
        ptcld( 2, i ) = msg->points[i].z;
        ptcld( 3, i ) = 1.0;
    }

    t_ptcld = msg->header.stamp;
    m_ptcld = true;
}


void DataNode::setUnVnFromMsg( const sensor_msgs::PointCloudConstPtr msg )
{
    std::lock_guard<std::mutex> lk(m);

    int n_pts = msg->channels.size() ;
    if( n_pts == 0 ) {
        // cout << TermColor::RED() << "[DataNode::setUnVnFromMsg]npts=0" << TermColor::RESET() << endl;
        t_unvn = msg->header.stamp;
        m_unvn = true;
        m_unvn_zero_pts = true;
        return ;
    }
    assert( n_pts > 0 && "[DataNode::setUnVnFromMsg] The input pointcloud msg contains zero unvn points, ie. msg->channels\n");

    unvn = MatrixXd::Zero( 3, n_pts );
    for( int i=0 ; i<n_pts ; i++ ) {
        unvn( 0, i ) = msg->channels[i].values[0];
        unvn( 1, i ) = msg->channels[i].values[1];
        unvn( 2, i ) = 1.0;
    }

    t_unvn = msg->header.stamp;
    m_unvn = true;
}


void DataNode::setUVFromMsg( const sensor_msgs::PointCloudConstPtr msg )
{
    std::lock_guard<std::mutex> lk(m);

    int n_pts = msg->channels.size() ;
    if( n_pts == 0 ) {
        // cout << TermColor::RED() << "[DataNode::setUVFromMsg]npts=0" << TermColor::RESET() << endl;
        t_uv = msg->header.stamp;
        m_uv = true;
        m_uv_zero_pts = true;
        return ;
    }
    assert( n_pts > 0 && "[DataNode::setUVFromMsg] The input pointcloud msg contains zero uv points, ie. msg->channels\n");

    uv = MatrixXd::Zero( 3, n_pts );
    for( int i=0 ; i<n_pts ; i++ ) {
        uv( 0, i ) = msg->channels[i].values[2];
        uv( 1, i ) = msg->channels[i].values[3];
        uv( 2, i ) = 1.0;
    }

    t_uv = msg->header.stamp;
    m_uv = true;
}

void DataNode::setTrackedFeatIdsFromMsg( const sensor_msgs::PointCloudConstPtr msg )
{
    std::lock_guard<std::mutex> lk(m);

    int n_pts = msg->channels.size() ;
    if( n_pts == 0 ) {
        // cout << TermColor::RED() << "[DataNode::setTrackedFeatIdsFromMsg]npts=0" << TermColor::RESET() << endl;
        t_tracked_feat_ids = msg->header.stamp;
        m_tracked_feat_ids = true;
        m_tracked_feat_ids_zero_pts = true;
        return ;
    }
    assert( n_pts > 0 && "[DataNode::setTrackedFeatIdsFromMsg] The input pointcloud msg contains zero tracked feature point ids, ie. msg->channels\n");

    tracked_feat_ids = VectorXi::Zero( n_pts );
    for( int i=0 ; i<n_pts ; i++ ) {
        tracked_feat_ids( i ) = int( msg->channels[i].values[4] );
    }

    t_tracked_feat_ids = msg->header.stamp;
    m_tracked_feat_ids = true;
}

void DataNode::setNumberOfSuccessfullyTrackedFeatures( int n )
{
    std::lock_guard<std::mutex> lk(m);
    assert( n >= 0 && "[DataNode::setNumberOfSuccessfullyTrackedFeatures] you cannot set a negative value");
    numberOfSuccessfullyTrackedFeatures = n;
}

int DataNode::getNumberOfSuccessfullyTrackedFeatures() const
{
    std::lock_guard<std::mutex> lk(m);
    return numberOfSuccessfullyTrackedFeatures;
}



const cv::Mat& DataNode::getImage() const {
    std::lock_guard<std::mutex> lk(m);
    assert( isImageAvailable() && "[DataNode::getImage] you requested the image before setting it");
    return this->image;
}


const cv::Mat& DataNode::getImage(short cam_id) const {
    std::lock_guard<std::mutex> lk(m);
    assert( isImageAvailable(cam_id) && "[DataNode::getImage with cam_id] you requested the image before setting it");
    // return this->all_images[cam_id];
    return this->all_images.at(cam_id);
}

const Matrix4d& DataNode::getPose() const{
    std::lock_guard<std::mutex> lk(m);
    assert( m_wTc && "[DataNode::getPose] you requested the pose before setting it");
    #if 0
    if( !m_wTc ) { cout << "ASSERT ERROR " << "[DataNode::getPose]  you requested the pose before setting it\n"; }
    #endif
    return wTc;
}


const MatrixXd& DataNode::getPoseCovariance() const {
    std::lock_guard<std::mutex> lk(m);
    assert( m_wTc && "[DataNode::getPoseCovariance] you requested the pose-cov before setting it");

    return wTc_covariance;
}

const MatrixXd& DataNode::getPointCloud() const {
    std::lock_guard<std::mutex> lk(m);
    assert( m_ptcld && "[DataNode::getPointCloud] you requested the pointcloud before setting it");
    return ptcld;
}

const MatrixXd& DataNode::getUnVn() const {
     // returns a 3xN matrix
     std::lock_guard<std::mutex> lk(m);
     assert( m_unvn && "[DataNode::getUnVn] you requested UnVn before setting it.\n" );
     return unvn;
 }

const MatrixXd& DataNode::getUV() const {
     // returns a 3xN matrix
     std::lock_guard<std::mutex> lk(m);
     assert( m_unvn && "[DataNode::getUV] you requested UnVn before setting it.\n" );
     return uv;
 }

const VectorXi& DataNode::getFeatIds() const  {
     // return a N-vector
     std::lock_guard<std::mutex> lk(m);
     assert( m_unvn && "[DataNode::getFeatIds] you requested UnVn before setting it.\n" );
     return tracked_feat_ids;
 }

 int DataNode::nPts() const {
     std::lock_guard<std::mutex> lk(m);
     #if 0
     if( m_tracked_feat_ids == false || m_tracked_feat_ids_zero_pts == true )
        return 0;
     return tracked_feat_ids.size();
     #endif

 }

 const ros::Time DataNode::getT() const {
     std::lock_guard<std::mutex> lk(m);
     return stamp;
 }
 const ros::Time DataNode::getT_image() const {
     std::lock_guard<std::mutex> lk(m);
     assert( isImageAvailable() );
     return t_image;
 }
 const ros::Time DataNode::getT_image(short cam_id) const {
     std::lock_guard<std::mutex> lk(m);
     assert( isImageAvailable(cam_id) );
    //  return t_all_images[cam_id];
     return t_all_images.at(cam_id);
 }



 const ros::Time DataNode::getT_pose() const {
     std::lock_guard<std::mutex> lk(m);
     return t_wTc;
 }
 const ros::Time DataNode::getT_ptcld() const {
     std::lock_guard<std::mutex> lk(m);
     return t_ptcld;
 }
 const ros::Time DataNode::getT_unvn() const {
     std::lock_guard<std::mutex> lk(m);
     return t_unvn;
 }
 const ros::Time DataNode::getT_uv() const {
     std::lock_guard<std::mutex> lk(m);
     return t_uv;
 }

void DataNode::prettyPrint()
{
    if( this->isKeyFrame() )
        cout << "\033[1;32m";
    else
        cout << "\033[1;31m";;

    cout << "\t---DataNode";
    cout << "\tt="<< this->getT() ;
    cout << "\tkeyFrame=" << (this->isKeyFrame()?"YES":"NO") << endl ;


    cout << "\t\twholeImageDesc: ";
    if( this->isWholeImageDescriptorAvailable() ) {
        cout << " available and of size: " << this->getWholeImageDescriptor().size() << endl;
    }
    else { cout << "Not Available\n" ;}


    if( isImageAvailable() ) {
        cv::Mat _im = this->getImage();
        cout << "\t\tImage: " << _im.rows << "x" << _im.cols << "x" << _im.channels()
             << "  " << MiscUtils::type2str( _im.type() ) << "\tt=" << t_image << endl;

    }
    else { cout << "\tImage: N/A\n"; }

    if( isPoseAvailable() ) {
        Matrix4d _pose = this->getPose();
        cout << "\t\tPose : " << PoseManipUtils::prettyprintMatrix4d( _pose ) << "\tt=" << t_wTc << endl;
    }
    else { cout << "\t\tPose: N/A\n"; }

    if( isPtCldAvailable() ) {
        MatrixXd _ptcld = this->getPointCloud();
        cout << "\t\tPointCloud : " << _ptcld.rows() << "x" << _ptcld.cols()  << "\tt=" << t_ptcld  << endl;
    }
    else { cout << "\t\tPointCloud : N/A\n"; }

    if( isUnVnAvailable() ) {
        MatrixXd _unvn = this->getUnVn();
        cout << "\t\tUnVn : " << _unvn.rows() << "x" << _unvn.cols() << "\tt=" << t_unvn << endl;
    }
    else { cout << "\t\tUnVn : N/A\n"; }

    if( isUVAvailable() ) {
        MatrixXd _uv = this->getUV();
        cout << "\t\tUV : " << _uv.rows() << "x" << _uv.cols() << "\tt=" << t_uv  << endl;
    }
    else { cout << "\t\tUV : N/A\n"; }

    if( isFeatIdsAvailable() ) {
        VectorXi _ids = this->getFeatIds();
        cout << "\t\tFeatIds : " << _ids.rows() << "x" << _ids.cols() << "\tt=" << t_tracked_feat_ids << endl;
    }
    else { cout << "\t\tFeatIds : N/A\n"; }
    cout << "\033[0m\n";
}



void DataNode::setWholeImageDescriptor( VectorXd vec )
{
    std::lock_guard<std::mutex> lk(m);

    assert( !m_img_desc && "[DataNode::setWholeImageDescriptor] You are trying to set image desc. The descriptor is already set and this action might overwrite the exisiting descriptor. If you think this is not an error feel free to comment this assert\n" );
    img_desc = vec;
    m_img_desc = true;
}

// const VectorXd& DataNode::getWholeImageDescriptor()
const VectorXd DataNode::getWholeImageDescriptor() const
{
    std::lock_guard<std::mutex> lk(m);
    assert( m_img_desc && "[DataNode::getWholeImageDescriptor] You are trying to get the image descriptor before setting it." );

    return img_desc;

}
