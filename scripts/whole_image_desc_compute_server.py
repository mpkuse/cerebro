#!/usr/bin/env python

from cerebro.srv import *
import rospy

def handle_req( req ):
    print 'Handle request '
    # return WholeImageDescriptorCompute( )
rospy.init_node( 'whole_image_descriptor_compute_server' )
s = rospy.Service( 'whole_image_descriptor_compute', WholeImageDescriptorCompute, handle_req  )
print 'whole_image_descriptor_compute_server is running'
rospy.spin()
