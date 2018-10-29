#!/usr/bin/env python

from cerebro.srv import *
import rospy

def handle_req( req ):
    print 'Handle request '
    print req
    u = {}
    u['desc'] = [1.1, 2,3, 4.4 ]

    res = WholeImageDescriptorComputeResponse()
    res.desc = [11.1, 32., 1.]
    return res


    results = WholeImageDescriptorCompute( )
    results.desc = [1.1, 2,3, 4.4 ]
    return results


rospy.init_node( 'whole_image_descriptor_compute_server' )
s = rospy.Service( 'whole_image_descriptor_compute', WholeImageDescriptorCompute, handle_req  )
print 'whole_image_descriptor_compute_server is running'
rospy.spin()
