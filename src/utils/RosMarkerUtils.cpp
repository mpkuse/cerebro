#include "RosMarkerUtils.h"

// cam_size = 1: means basic size. 1.5 will make it 50% bigger.
void RosMarkerUtils::init_camera_marker( visualization_msgs::Marker& marker, float cam_size )
{
     marker.header.frame_id = "world";
     marker.header.stamp = ros::Time::now();
     marker.action = visualization_msgs::Marker::ADD;
     marker.color.a = .7; // Don't forget to set the alpha!
     marker.type = visualization_msgs::Marker::LINE_LIST;
    //  marker.id = i;
    //  marker.ns = "camerapose_visual";

     marker.scale.x = 0.003; //width of line-segments
     float __vcam_width = 0.07*cam_size;
     float __vcam_height = 0.04*cam_size;
     float __z = 0.1*cam_size;




     marker.points.clear();
     geometry_msgs::Point pt;
     pt.x = 0; pt.y=0; pt.z=0;
     marker.points.push_back( pt );
     pt.x = __vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = 0; pt.y=0; pt.z=0;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = 0; pt.y=0; pt.z=0;
     marker.points.push_back( pt );
     pt.x = __vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = 0; pt.y=0; pt.z=0;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );

     pt.x = __vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = -__vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = __vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = __vcam_width; pt.y=-__vcam_height; pt.z=__z;
     marker.points.push_back( pt );
     pt.x = __vcam_width; pt.y=__vcam_height; pt.z=__z;
     marker.points.push_back( pt );


     // TOSET
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    // marker.id = i;
    // marker.ns = "camerapose_visual";
    marker.color.r = 0.2;marker.color.b = 0.;marker.color.g = 0.;
}

void RosMarkerUtils::setpose_to_marker( const Matrix4d& w_T_c, visualization_msgs::Marker& marker )
{
    Quaterniond quat( w_T_c.topLeftCorner<3,3>() );
    marker.pose.position.x = w_T_c(0,3);
    marker.pose.position.y = w_T_c(1,3);
    marker.pose.position.z = w_T_c(2,3);
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
}

void RosMarkerUtils::setposition_to_marker( const Vector3d& w_t_c, visualization_msgs::Marker& marker )
{
    marker.pose.position.x = w_t_c(0);
    marker.pose.position.y = w_t_c(1);
    marker.pose.position.z = w_t_c(2);
}

void RosMarkerUtils::setposition_to_marker( const Vector4d& w_t_c, visualization_msgs::Marker& marker )
{
    marker.pose.position.x = w_t_c(0) / w_t_c(3); ;
    marker.pose.position.y = w_t_c(1) / w_t_c(3); ;
    marker.pose.position.z = w_t_c(2) / w_t_c(3); ;
}

void RosMarkerUtils::setposition_to_marker( float x, float y, float z, visualization_msgs::Marker& marker )
{
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
}

void RosMarkerUtils::setcolor_to_marker( float r, float g, float b, visualization_msgs::Marker& marker  )
{
    assert( r>=0. && r<=1.0 && g>=0. && g<=1.0 && b>=0 && b<=1.0 );
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
}

void RosMarkerUtils::setcolor_to_marker( float r, float g, float b, float a, visualization_msgs::Marker& marker  )
{
    assert( r>=0. && r<=1.0 && g>=0. && g<=1.0 && b>=0 && b<=1.0 && a>0. && a<=1.0);
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
}

void RosMarkerUtils::init_text_marker( visualization_msgs::Marker &marker )
{
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = .8; // Don't forget to set the alpha!
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.scale.x = 1.; //not in use
    marker.scale.y = 1.; //not in use
    marker.scale.z = 1.;

    //// Done . no need to edit firther
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    // marker.id = i;
    // marker.ns = "camerapose_visual";
    marker.color.r = 0.2;marker.color.b = 0.;marker.color.g = 0.;

}

void RosMarkerUtils::init_line_strip_marker( visualization_msgs::Marker &marker )
{
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = .8; // Don't forget to set the alpha!
    marker.type = visualization_msgs::Marker::LINE_STRIP;

    marker.scale.x = 0.02;

    marker.points.clear();

    //// Done . no need to edit firther
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    // marker.id = i;
    // marker.ns = "camerapose_visual";
    marker.color.r = 0.2;marker.color.b = 0.;marker.color.g = 0.;

}

void RosMarkerUtils::init_line_marker( visualization_msgs::Marker &marker )
{
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = .8; // Don't forget to set the alpha!
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.scale.x = 0.02;

    marker.points.clear();

    //// Done . no need to edit firther
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    // marker.id = i;
    // marker.ns = "camerapose_visual";
    marker.color.r = 0.2;marker.color.b = 0.;marker.color.g = 0.;

}

void RosMarkerUtils::init_line_marker( visualization_msgs::Marker &marker, const Vector3d& p1, const Vector3d& p2 )
{
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = .8; // Don't forget to set the alpha!
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.scale.x = 0.02;

    marker.points.clear();
    geometry_msgs::Point pt;
    pt.x = p1(0);
    pt.y = p1(1);
    pt.z = p1(2);
    marker.points.push_back( pt );
    pt.x = p2(0);
    pt.y = p2(1);
    pt.z = p2(2);
    marker.points.push_back( pt );

    //// Done . no need to edit firther
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    // marker.id = i;
    // marker.ns = "camerapose_visual";
    marker.color.r = 0.2;marker.color.b = 0.;marker.color.g = 0.;

}


void RosMarkerUtils::init_points_marker( visualization_msgs::Marker &marker )
{
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = .8; // Don't forget to set the alpha!
    marker.type = visualization_msgs::Marker::POINTS;

    marker.scale.x = 0.04;
    marker.scale.y = 0.04;

    marker.points.clear();

    //// Done . no need to edit firther
    marker.pose.position.x = 0.;
    marker.pose.position.y = 0.;
    marker.pose.position.z = 0.;
    marker.pose.orientation.x = 0.;
    marker.pose.orientation.y = 0.;
    marker.pose.orientation.z = 0.;
    marker.pose.orientation.w = 1.;
    // marker.id = i;
    // marker.ns = "camerapose_visual";
    marker.color.r = 0.2;marker.color.b = 0.;marker.color.g = 0.;

}




void RosMarkerUtils::add_point_to_marker( float x, float y, float z, visualization_msgs::Marker& marker, bool clear_prev_points )
{
    if( clear_prev_points )
        marker.points.clear();

    geometry_msgs::Point pt;
    pt.x = x; pt.y = y; pt.z = z;
    marker.points.push_back( pt );
}

void RosMarkerUtils::add_point_to_marker( const Vector3d& X, visualization_msgs::Marker& marker, bool clear_prev_points )
{
    if( clear_prev_points )
        marker.points.clear();

    geometry_msgs::Point pt;
    pt.x = X(0); pt.y = X(1); pt.z = X(2);
    marker.points.push_back( pt );
}

void RosMarkerUtils::add_point_to_marker( const Vector4d& X, visualization_msgs::Marker& marker, bool clear_prev_points )
{
    if( clear_prev_points )
        marker.points.clear();

    geometry_msgs::Point pt;
    assert( abs(X(3)) > 1E-5 );
    pt.x = X(0)/X(3); pt.y = X(1)/X(3); pt.z = X(2)/X(3);
    marker.points.push_back( pt );
}

void RosMarkerUtils::add_points_to_marker( const MatrixXd& X, visualization_msgs::Marker& marker, bool clear_prev_points ) //X : 3xN or 4xN.
{
    assert( (X.rows() == 3 || X.rows() == 4) && "[RosMarkerUtils::add_points_to_marker] X need to of size 3xN or 4xN\n" );
    geometry_msgs::Point pt;

    if( clear_prev_points )
        marker.points.clear();


    for( int i=0 ; i<X.cols() ; i++ ) {
        if( X.rows() == 3 ) {
            pt.x = X(0,i); pt.y = X(1,i); pt.z = X(2,i);
        }
        else {
            pt.x = X(0,i) / X(3,i); pt.y = X(1,i) / X(3,i); pt.z = X(2,i) / X(3,i);
        }
        marker.points.push_back( pt );
    }
}
