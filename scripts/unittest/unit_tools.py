
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import code
import sys



def make_marker():
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_namespace"
    marker.id = 0

    marker.type = marker.SPHERE
    marker.action = marker.ADD

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.5
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker

def make_line_marker(w_t_p0, w_t_p1):
    marker = make_marker()
    marker.ns = 'hlt_loop_candidates_raw'#+offsettxt
    marker.type = marker.LINE_LIST
    marker.id = 0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.scale.x = 0.17
    marker.points = []
    marker.points.append( Point( w_t_p0[0], w_t_p0[1], w_t_p0[2] ) )
    marker.points.append( Point( w_t_p1[0], w_t_p1[1], w_t_p1[2] ) )
    return marker

# T : info on each pair like:  [  (i,j, score) , ... ]
# VIO__w_t_i: position of each node.
# max_n (optional): If this is None means publish all nodes and all candidates. If this is a number will publish upto that nodes. In this case will return image of last pair
def publish_marker( pub, VIO__w_t_i, T, TH=0.92, max_n=None, BASE=None  ):

    if max_n is None:
        up_to = len(VIO__w_t_i)
    else:
        up_to = max_n

    # Publish VIO
    m = make_marker()
    m.ns = 'vio'#+offsettxt
    m.type = m.LINE_STRIP
    m.scale.x = 0.05
    print 'len(VIO__w_t_i) ', len(VIO__w_t_i), '   len(T) ', len(T), 'max_n ', str(up_to)
    for i, w_t_i in enumerate(VIO__w_t_i[:up_to] ):
        m.points.append( Point( w_t_i[0], w_t_i[1], w_t_i[2] ) )  #plot nodes at x,y,z
        # m.points.append( Point( w_t_i[0], w_t_i[2], i ) )           #plot nodes at x,z,frame#
    pub.publish( m )


    if True:
        # Publish VIO text
        m = make_marker()
        m.ns = 'vio_text'#+offsettxt
        m.type = m.TEXT_VIEW_FACING
        m.scale.z = 0.07
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        for i, w_t_i in enumerate(VIO__w_t_i[:up_to]):
            m.text = str(i)
            m.id = i
            m.pose.position.x =  w_t_i[0]
            m.pose.position.y =  w_t_i[1]
            m.pose.position.z =  w_t_i[2]
            pub.publish( m )


    # Plot loop candidates
    c = 0
    m = make_marker()
    m.ns = 'loop_candidates_raw'#+offsettxt
    m.type = m.LINE_LIST
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.scale.x = 0.07
    selected_loop_candidates = []
    for i in range( len(T) ):
        p0 = int(T[i][0])
        p1 = int(T[i][1])
        score = T[i][2]

        # if score > TH and abs(T[i+1][1] - p1) < 8 and abs(T[i+2][1] - p1) < 8: # and T[i+1][2] > TH and T[i+2][2] > TH:
        if max_n is not None and (p0 >= up_to or p1 >= up_to):
            continue

        if max_n is None:
            print '%d<--%4.2f-->%d' %( p0, score, p1 ), '\tAccept'

        w_t_p0 = VIO__w_t_i[ p0 ]
        w_t_p1 = VIO__w_t_i[ p1 ]
        m.points.append( Point( w_t_p0[0], w_t_p0[1], w_t_p0[2] ) )
        m.points.append( Point( w_t_p1[0], w_t_p1[1], w_t_p1[2] ) )

        # m.points.append( Point( w_t_p0[0], w_t_p0[2], p0) )
        # m.points.append( Point( w_t_p1[0], w_t_p1[2], p1 ) )

        selected_loop_candidates.append( (p0,p1) )

        c = c+1

    pub.publish( m )
    print 'publish n_loop_candidates: ', c, 'TH=', TH


    # Display Images of the Last Candidate
    if (max_n is not None) and len(selected_loop_candidates) > 0:
        assert( BASE is not None )
        p0 = selected_loop_candidates[ -1 ][0]
        p1 = selected_loop_candidates[ -1 ][1]

        im0 = cv2.imread( BASE+'/%d.jpg' %(p0) )
        im1 = cv2.imread( BASE+'/%d.jpg' %(p1) )
        blank_space = np.ones( (im0.shape[0], 50, 3) )*255

        im = np.concatenate( (im0, blank_space.astype('uint8'), im1,), axis=1 )
        status = np.zeros( (120,im.shape[1],3)).astype('uint8')
        cv2.putText(status, '%d<   (%4.6f)   >%d' %( p0, score, p1 ), (10,65), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2 )
        im = np.concatenate( (im,status), axis=0 )
        # cv2.imshow( 'candidate', im  )
        # cv2.waitKey(10)
        return im
    else:
        return None




# uses the global variable TH and KITTI_BASE.
# T : info on each pair like:  [  (i,j, score) , ... ]
# VIO__w_t_i: position of each node. list of 3d pts.
#   Show call the candidates in T. Hopefully they are filtered already, using the call to `filter_candidates`
def imshow_loopcandidates( T, BASE=None, VIO__w_t_i=None, pub=None ):
    assert( BASE is not None )

    # for i in range( 0,len(T)-3, 3 ):
    for i in range( 0, len(T) ):
        p0 = int(T[i][0])
        p1 = int(T[i][1])
        score = T[i][2]
        print '[%d of %d]' %(i, len(T)), '%d<--(%4.2f)-->%d' %( p0, score, p1 ), '\tAccept'

        im0 = cv2.imread( BASE+'/%d.jpg' %(p0) )
        im1 = cv2.imread( BASE+'/%d.jpg' %(p1) )
        blank_space = np.ones( (im0.shape[0], 50, 3) )*255

        im = np.concatenate( (im0, blank_space.astype('uint8'), im1,), axis=1 )
        status = np.zeros( (120,im.shape[1],3)).astype('uint8')
        cv2.putText(status, '%d<   (%4.6f)   >%d' %( p0, score, p1 ), (10,65), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255) )
        im = np.concatenate( (im,status), axis=0 )

        # TODO Try Feature Matching and score computation on these 2 images

        # Publish marker to highlight this pair on trajectory
        if VIO__w_t_i is not None and pub is not None:
            marker = make_line_marker( VIO__w_t_i[p0], VIO__w_t_i[p1] )
            pub.publish( marker )


        cv2.imshow( 'im', im )
        print 'Press <space> for next pair, <q> to exit this loop. <w> to save this image to BASE (%s)' %(BASE)
        key = cv2.waitKey(0)
        if key == ord('q'):
            break
        if key == ord('w'):
            print 'Writing Image: ', BASE+'/loopcandidate_%d_%d.jpg' %(p0,p1)
            cv2.imwrite( BASE+'/loopcandidate_%d_%d.jpg' %(p0,p1), im )

    cv2.destroyWindow('im')



# Play. Increment 1 note at a time. 20Hz loop
# w_t_all: list of 4x4 matrix. Each 4x4 matrix is the pose.
# T      : loop candidates
def play_trajectory_with_loopcandidates( VIO__w_t_i, T, BASE=None, pub=None, pub_frames=None, pub_loopcandidates=None ):
    assert( BASE is not None and pub is not None and pub_frames is not None and pub_loopcandidates is not None )

    print 'play_trajectory_with_loopcandidates'

    m = make_marker()
    m.ns = 'vio'#+offsettxt
    m.type = m.LINE_STRIP
    m.scale.x = 0.05
    m.points = []
    m.id = 0


    rate = rospy.Rate(200)
    N = len(VIO__w_t_i)
    n = -1
    last_candidate_indx = 0
    while not rospy.is_shutdown():
        n += 1
        if n >= N:
            break
        print 'Play %d of %d' %(n,N)

        im_frame = cv2.imread( BASE+'/%d.jpg' %(n) )
        # cv2.imshow( 'frame-', im_frame )
        image_msg = CvBridge().cv2_to_imgmsg( im_frame )
        pub_frames.publish( image_msg )

        # im_loopcandidate = publish_marker( pub, VIO__w_t_i, T, max_n=n, BASE=BASE )
        m.points.append( Point( VIO__w_t_i[n][0], VIO__w_t_i[n][1], VIO__w_t_i[n][2] ) )
        pub.publish( m )


        for candidate_i  in range( last_candidate_indx, len(T) ):
            candidate = T[ candidate_i ]
            if candidate[0] > n:
                last_candidate_indx = candidate_i
                break
            p0 = int(candidate[0])
            p1 = int(candidate[1])
            score = candidate[2]

            # Make and publish Line Marker
            candidate_marker = make_line_marker( VIO__w_t_i[p0], VIO__w_t_i[p1] )
            candidate_marker.ns = 'loop_candidates_raw'
            candidate_marker.color.r = 1.0
            candidate_marker.color.g = 0.0
            candidate_marker.color.b = 0.0
            candidate_marker.scale.x = 0.07
            candidate_marker.id = candidate_i
            pub.publish(candidate_marker)


            # Load Those 2 image pair and publish
            im0 = cv2.imread( BASE+'/%d.jpg' %(p0) )
            im1 = cv2.imread( BASE+'/%d.jpg' %(p1) )
            blank_space = np.ones( (im0.shape[0], 50, 3) )*255

            im = np.concatenate( (im0, blank_space.astype('uint8'), im1,), axis=1 )
            status = np.zeros( (120,im.shape[1],3)).astype('uint8')
            cv2.putText(status, '%d<   (%4.6f)   >%d' %( p0, score, p1 ), (10,65), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2 )
            im_loopcandidate = np.concatenate( (im,status), axis=0 )

            image_loopcandidate_msg = CvBridge().cv2_to_imgmsg( im_loopcandidate )
            pub_loopcandidates.publish( image_loopcandidate_msg )


        # if im_loopcandidate is not None:
            # image_loopcandidate_msg = CvBridge().cv2_to_imgmsg( im_loopcandidate )
            # pub_loopcandidates.publish( image_loopcandidate_msg )

        cv2.waitKey(1)
        rate.sleep()
