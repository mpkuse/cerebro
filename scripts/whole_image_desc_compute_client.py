#!/usr/bin/env python

from cerebro.srv import *
import rospy

rospy.wait_for_service( 'whole_image_descriptor_compute' )
try:
    res = rospy.ServiceProxy( 'whole_image_descriptor_compute', WholeImageDescriptorCompute )

    u = res( 23  )
    print 'received: ', u
except rospy.ServiceException, e:
    print 'failed', e
