#include <ros/ros.h>
#include <cerebro/WholeImageDescriptorCompute.h>
#include <iostream>

#include <sensor_msgs/Image.h>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>


int main( int argc, char ** argv )
{
    // Ros INIT and Make connection to Server
    ros::init( argc, argv, "unittest_rosservice_client_cpp" );
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<cerebro::WholeImageDescriptorCompute>( "whole_image_descriptor_compute" );
    client.waitForExistence();
    if( !client.exists() ) {
        ROS_ERROR( "Connection to server NOT successful" );
        return 0;
    }
    else std::cout << "Connection to server established\n";

    // Make service message
    cerebro::WholeImageDescriptorCompute srv; //service message
    cv::Mat im = cv::imread( "/app/lena.jpg") ;
    cv::Mat im_resized;
    cv::resize( im, im_resized, cv::Size(640, 512) );
    cv::imshow( "lena", im );
    cv::waitKey(0);

    // call
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im_resized).toImageMsg();
    srv.request.ima = *image_msg;
    srv.request.a = 678;
    if( client.call( srv ) ) {
        ROS_INFO( "Received response from server" );
        std::cout << "desc.size=" << srv.response.desc.size() << std::endl;
    }
    else {
        ROS_ERROR( "Failed to call ros service" );
    }

    return 0;

}
