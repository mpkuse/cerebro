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


# publishing
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# returns selected loop candidates by locality heuristic and the threshold.
def filter_candidates( T, TH=0.92, locality=8 ):
    S = []
    for i in range( 0,len(T)-3, 3 ):
        p0 = int(T[i][0])
        p1 = int(T[i][1])
        score = T[i][2]

        if score > TH and abs(T[i+1][1] - p1) < locality and abs(T[i+2][1] - p1) < locality: # and T[i+1][2] > TH and T[i+2][2] > TH:
            S.append( (p0, p1, score) )
    return S



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
    # BASE = '/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/'
    # BASE = '/Bulk_Data/_tmp_cerebro/bb4_loopy_drone_fly_area/'
    # BASE = '/Bulk_Data/_tmp_cerebro/bb4_long_lab_traj/'
    BASE = '/Bulk_Data/_tmp_cerebro/bb4_floor2_cyt/'

    # BASE = '/Bulk_Data/_tmp_cerebro/euroc_MH_01_easy/'
    BASE = '/Bulk_Data/_tmp_cerebro/euroc_MH_02_easy/'


    #
    # Open Log File
    #
    LOG_FILE_NAME = BASE+'/log.json'
    print 'Open file: ', LOG_FILE_NAME
    with open(LOG_FILE_NAME) as data_file:
        data = json.load(data_file)

    # Collect all w_T_c, ie. VIO poses for plotting
    VIO__w_t_i = []
    w_t_c = np.array( [0,0,0] )
    print 'Loading VIO__w_t_i'
    for i in range( len(data['DataNodes']) ):
        if data['DataNodes'][i]['isPoseAvailable'] == 0:
            # w_t_c = np.array( [0,0,0] )
            pass # use the previously known pose
        else:
            w_T_c = np.array( data['DataNodes'][i]['w_T_c']['data']).reshape( (4,4) )
            w_t_c = w_T_c[0:3,3]

        VIO__w_t_i.append( w_t_c )



    #
    # ROS Node
    #
    rospy.init_node('talker', anonymous=True)
    rospy.myargv(argv=sys.argv)
    print 'Publish Topic : Marker::chatter, Image::frames, Image::loopcandidates'
    pub = rospy.Publisher('chatter', Marker, queue_size=100)
    pub_frames = rospy.Publisher('frames', Image, queue_size=50)
    pub_loopcandidates = rospy.Publisher('loopcandidates', Image, queue_size=50)



    #
    # Loops over all images and precomputes their netvlad vector
    #
    if False: #making this to false will load npz files which contain the pre-computes descriptors.
        #
        # Init Keras Model - NetVLAD / Enable Service
        #
        # gpu_s = SampleGPUComputer()
        print 'waiting for ros-service : whole_image_descriptor_compute'
        rospy.wait_for_service( 'whole_image_descriptor_compute' )
        try:
            service_proxy = rospy.ServiceProxy( 'whole_image_descriptor_compute', WholeImageDescriptorCompute )
        except rospy.ServiceException, e:
            print 'failed', e
        print 'connected to ros-service'

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
        netvlad_desc = np.array( netvlad_desc ) # N x 4096. 4096 is the length of netvlad vector.

        fname = BASE+'/file.npz'
        print 'Save `netvlad_desc` and `netvlad_at_i` in ', fname
        np.savez_compressed(BASE+'/file.npz', netvlad_desc=netvlad_desc, netvlad_at_i=netvlad_at_i)
    else:
        fname = BASE+'/file.npz'
        print 'Load ', fname
        loaded = np.load(fname)
        netvlad_desc = loaded['netvlad_desc']
        netvlad_at_i = loaded['netvlad_at_i']
        print 'netvlad_desc.shape=', netvlad_desc.shape , '\tnetvlad_at_i.shape', netvlad_at_i.shape



    #
    # Find candidate matches for each i amongst (0 to i-1). Accept candidate
    #   only if i-1, i-2 and i go to a similar neighbourhood.
    #       T = [ {a<-->b, score}, {a<-->b, score},...  ] #list of raw loop candidates with their scores
    #
    D = netvlad_desc
    T = []
    for i in range( netvlad_desc.shape[0] ):
            if i < 50: #don't lookup for first few frames
                continue

            DOT = np.dot( D[0:i-45,:], D[i,:] ) # compare D_live[i] will all the memory
            score  = np.max(DOT)
            argmax = np.argmax( DOT )
            #print 'Nearest neighobour of %d of live in db is %d (dotprodt = %4.4f)' %( netvlad_at_i[i], netvlad_at_i[argmax], score )

            T.append( (netvlad_at_i[i], netvlad_at_i[argmax], score) )

    S = filter_candidates( T, TH=0., locality=8 )





    #
    # Publish VIO__w_t_i and loop candidates on trajectory
    #
    from unit_tools import publish_marker, imshow_loopcandidates, play_trajectory_with_loopcandidates


    #
    # while not rospy.is_shutdown():
    #     publish_marker( pub, VIO__w_t_i, T, TH=0.92 )
    #


    rate = rospy.Rate(10) # 10hz
    TH=0.92
    TH_step = 0.005
    list_of_scores = [ g[2] for g in S ]
    while not rospy.is_shutdown():
        S = filter_candidates( T, TH=TH, locality=8 )
        publish_marker( pub, VIO__w_t_i, S, TH=TH )
        print '---\n(higher score ==> higher confidence of match)'
        print 'list_of_scores: min=', min(list_of_scores), 'max=',max( list_of_scores)
        print 'press\n\
                <a> to increment threshold by %4.6f.<z> to decrement.\n\
                <s> to view current loop-candidates and write list as csv file.\n\
                <p> play like a video\n\
                <c> compare current candidates with manual annotations (not in use)\n\
                <q> to quit.' %(TH_step)

        IM = np.zeros( (120,450)).astype('uint8')
        cv2.putText(IM,'DATASET=%s' %( BASE.split('/')[-2] ), (10,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
        cv2.putText(IM,'nAccepted=%d' %( len(S) ), (10,35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
        cv2.putText(IM,'TH=%4.6f' %(TH), (10,65), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
        cv2.putText(IM,'max=%4.6f' %(max(list_of_scores)), (10,85), cv2.FONT_HERSHEY_SIMPLEX, .5, 255)
        cv2.putText(IM,'min=%4.6f' %(min(list_of_scores)), (10,105), cv2.FONT_HERSHEY_SIMPLEX, .5, 255)

        cv2.imshow( 'im', IM )
        key = cv2.waitKey(10)
        if key == ord( 'a' ):
            TH += TH_step
        if key == ord( 'z' ):
            TH -= TH_step
        if key == ord( 'q' ):
            break
        if key == ord( 's' ):
            imshow_loopcandidates( S, BASE=BASE, VIO__w_t_i=VIO__w_t_i, pub=pub )

        if key == ord( 'p' ):
            play_trajectory_with_loopcandidates( VIO__w_t_i, S, BASE=BASE, pub=pub, pub_frames=pub_frames, pub_loopcandidates=pub_loopcandidates )
        # if key == ord( 'c' ):
            # compare_with_manual( T, dump_file_ptr=pr_file_ptr )
        # if key == ord( 'v'):
            # for g in np.linspace( list_of_scores[0], list_of_scores[-1], 50 ):
                # TH = g
                # compare_with_manual( T, dump_file_ptr=pr_file_ptr )
        rate.sleep()


    # Write the current candidates to file:
    print 'Open File loop candidates: ', BASE+'loop_candidates.txt'
    with open(BASE+"loop_candidates.txt", "w") as text_file:
        text_file.write( '#curr,prev,score\n#TH=%f\n' %(TH))
        for s in S:
            text_file.write( '%d, %d, %f\n' %(s[0], s[1], s[2]) )

    quit()
