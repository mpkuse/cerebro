#!/usr/bin/env python

import numpy as np
import code
import cv2
import time

import json

# Needed to make ros-service call.
import rospy
from cv_bridge import CvBridge, CvBridgeError
from cerebro.srv import *


# import keras
# class SampleGPUComputer:
#     def __init__(self):
#         self.model = keras.applications.vgg16.VGG16( weights=None)
#         self.model.summary()
#
#     def compute( self, im ):
#         im = cv2.resize( im, (224,224) )
#         preds = self.model.predict( np.expand_dims(im,0) )
#         return preds

if __name__ == "__main__":
    BASE = '/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/'


    #
    # Open Log File
    #
    LOG_FILE_NAME = BASE+'/log.json'
    print 'Open file: ', LOG_FILE_NAME
    with open(LOG_FILE_NAME) as data_file:
        data = json.load(data_file)


    #
    # Init Keras Model - NetVLAD / Enable Service
    #
    # gpu_s = SampleGPUComputer()
    rospy.wait_for_service( 'whole_image_descriptor_compute' )
    try:
        service_proxy = rospy.ServiceProxy( 'whole_image_descriptor_compute', WholeImageDescriptorCompute )
    except rospy.ServiceException, e:
        print 'failed', e

    #
    # Loops over all images and precomputes their netvlad vector
    #
    netvlad_desc = []
    netvlad_at_i = []
    for i in range( len(data['DataNodes']) ):
        a = data['DataNodes'][i]['isKeyFrame']
        b = data['DataNodes'][i]['isImageAvailable']
        c = data['DataNodes'][i]['isPoseAvailable']
        d = data['DataNodes'][i]['isPtCldAvailable']


        if not ( a==1 and b==1 and c==1 and d==1 ): #only process keyframes which have pose and ptcld info
            continue

        im = cv2.imread( BASE+'%d.jpg' %(i) )

        start_time = time.time()
        print '---', i , '\n'
        image_msg = CvBridge().cv2_to_imgmsg( im )
        rcvd_ = service_proxy( image_msg, 24 )
        netvlad_desc.append( rcvd_.desc )
        netvlad_at_i.append( i )
        print 'Done in %4.4fms' %( 1000. * (time.time() - start_time ) )

        cv2.imshow( 'im', im )
        key = cv2.waitKey(10)
        if key == ord( 'q' ):
            break
    netvlad_desc = np.array( netvlad_desc )
    D = netvlad_desc

    #
    # Find candidate matches for each i amongst (0 to i-1). Accept candidate
    #   only if i-1, i-2 and i go to a similar neighbourhood.
    #       T = [ {a<-->b, score}, {a<-->b, score},...  ] #list of loop candidates with their scores
    #
    T = []
    for i in range( netvlad_desc.shape[0] ):
            if i < 50: #don't lookup for first few frames
                continue

            DOT = np.dot( D[0:i-45,:], D[i,:] ) # compare D_live[i] will all the memory
            score  = np.max(DOT)
            argmax = np.argmax( DOT )
            print 'Nearest neighobour of %d of live in db is %d (dotprodt = %4.4f)' %( netvlad_at_i[i], netvlad_at_i[argmax], score )

            T.append( (i, argmax, score) )



    #
    # Similar Visualization to prev loop tool
    #
