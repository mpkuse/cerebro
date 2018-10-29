#include "DataNode.h"


void DataNode::setImageFromMsg( const sensor_msgs::ImageConstPtr msg )
{
    std::lock_guard<std::mutex> lk(m);

    // image = cv_bridge::toCvShare(msg, "bgr8" )->image
    image = cv_bridge::toCvCopy(msg, "bgr8" )->image;

    // TODO: Make sure the image is valid
    assert( image.rows > 0 && image.cols > 0 && !image.empty() && "In DataNode::setImageFromMsg image that is being set from sensor_msgs::Image is invalid.");


    t_image = msg->header.stamp;
    m_image = true;
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
    assert( n_pts > 0 && "[DataNode::setTrackedFeatIdsFromMsg] The input pointcloud msg contains zero tracked feature point ids, ie. msg->channels\n");

    tracked_feat_ids = VectorXi::Zero( n_pts );
    for( int i=0 ; i<n_pts ; i++ ) {
        tracked_feat_ids( i ) = int( msg->channels[i].values[4] );
    }

    t_tracked_feat_ids = msg->header.stamp;
    m_tracked_feat_ids = true;
}



const cv::Mat& DataNode::getImage() {
    std::lock_guard<std::mutex> lk(m);
    assert( m_image && "[DataNode::getImage] you requested the image before setting it");
    return image;
}


const Matrix4d& DataNode::getPose() {
    std::lock_guard<std::mutex> lk(m);
    assert( m_wTc && "[DataNode::getPose] you requested the pose before setting it");
    return wTc;
}


const MatrixXd& DataNode::getPoseCovariance() {
    std::lock_guard<std::mutex> lk(m);
    assert( m_wTc && "[DataNode::getPoseCovariance] you requested the pose-cov before setting it");
    return wTc_covariance;
}

const MatrixXd& DataNode::getPointCloud() {
    std::lock_guard<std::mutex> lk(m);
    assert( m_ptcld && "[DataNode::getPointCloud] you requested the pointcloud before setting it");
    return ptcld;
}

const MatrixXd& DataNode::getUnVn() {
     // returns a 3xN matrix
     std::lock_guard<std::mutex> lk(m);
     assert( m_unvn && "[DataNode::getUnVn] you requested UnVn before setting it.\n" );
     return unvn;
 }

const MatrixXd& DataNode::getUV() {
     // returns a 3xN matrix
     std::lock_guard<std::mutex> lk(m);
     assert( m_unvn && "[DataNode::getUV] you requested UnVn before setting it.\n" );
     return uv;
 }

const VectorXi& DataNode::getFeatIds() {
     // return a N-vector
     std::lock_guard<std::mutex> lk(m);
     assert( m_unvn && "[DataNode::getFeatIds] you requested UnVn before setting it.\n" );
     return tracked_feat_ids;
 }

 int DataNode::nPts() {
     std::lock_guard<std::mutex> lk(m);
     return tracked_feat_ids.size();
 }

 const ros::Time DataNode::getT() {
     std::lock_guard<std::mutex> lk(m);
     return stamp;
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
