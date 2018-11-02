#!/usr/bin/env python

import numpy as np
import code
import cv2
import time

import json

import torch
# Stub to warn about opencv version.
if int(cv2.__version__[0]) < 3: # pragma: no cover
  print('Warning: OpenCV 3 is not installed')

from demo_superpoint import *

class KSuperPointExp:
    def __init__(self):
        #------------------------------------------------------------------------
        # Setup Neural Net
        weights_path='superpoint_v1.pth'
        nms_dist=4
        conf_thresh=0.015
        nn_thresh=0.7
        cuda=True
        print('==> Loading pre-trained network.')
        # This class runs the SuperPoint network and processes its outputs.
        self.fe = SuperPointFrontend(weights_path=weights_path,
                             nms_dist=nms_dist,
                             conf_thresh=conf_thresh,
                             nn_thresh=nn_thresh,
                             cuda=cuda)
        print('==> Successfully loaded pre-trained network.')


        #-----------------------------------------------------------------------
        # Tracker - NN comparison for desc
        max_length = 5 # default as in the demo file
        self.tracker = PointTracker(max_length, nn_thresh=self.fe.nn_thresh)


    def plot_pts( self, xcanvas, pts ):
        # pts:  [x_i, y_i, confidence_i]. 3xN
        for i in range( pts.shape[1] ):
            cv2.circle( xcanvas, (int(pts[0,i]), int(pts[1,i]) ), 1, (0,0,255), -1 )

        return xcanvas

    def plot_point_sets( self, im0, pts0, im1, pts1 ):
        # pts:  [x_i, y_i, confidence_i]. 3xN
        # try:
        xcanvas = np.concatenate( (im0, im1), axis=1 )
        if pts0.shape[0] == 0:
            return xcanvas
        # except:
            # code.interact( local=locals() )
        assert( pts0.shape[1] == pts1.shape[1] )
        assert( pts0.shape[0] == 3 or pts0.shape[0] == 2 )
        assert( pts1.shape[0] == 3 or pts1.shape[0] == 2 )


        for i in range( pts0.shape[1] ):
            pt0 = (int(pts0[0,i]), int(pts0[1,i]) )
            pt1 = (int(pts1[0,i] + im0.shape[1] ), int(pts1[1,i]) )
            cv2.circle( xcanvas, pt0, 1, (0,255,0), -1)
            cv2.circle( xcanvas, pt1, 1, (0,255,0), -1)
            cv2.line( xcanvas, pt0, pt1, (0,255,0) )

        return xcanvas



    def read_image(self, impath, img_size):
        """ Read image as grayscale and resize to img_size.
        Inputs
          impath: Path to input image.
          img_size: (W, H) tuple specifying resize size.
        Returns
          grayim: float32 numpy array sized H x W with values in range [0, 1].
        """
        grayim = cv2.imread(impath, 0)
        if grayim is None:
          raise Exception('Error reading image %s' % impath)
        # Image is resized via opencv.
        interp = cv2.INTER_AREA
        grayim = cv2.resize(grayim, (img_size[1], img_size[0]), interpolation=interp)
        grayim = (grayim.astype('float32') / 255.)
        return grayim


    def get_descriptor( self, image ):
        """ Given an image returns the keypoints and descriptors.
            Input
              img - HxW numpy float32 input image in range [0,1].
            Output
              corners - 3xN numpy array with corners [x_i, y_i, confidence_i]^T.
              desc - 256xN numpy array of corresponding unit normalized descriptors.
              heatmap - HxW numpy heatmap in range [0,1] of point confidences.
        """
        start1 = time.time()
        pts, desc, heatmap = self.fe.run(image)
        print 'superpoint computed in %4.2fms' %(1000. * (time.time() - start1))
        return pts, desc, heatmap


    def match_image_pair( self, fname0, fname1 ):
        """ Given two image file names, read the two images (as per SuperPoint's requirement).
        Computes descriptors and then does NN comparison.
        """

        ksp_image0 = ksp.read_image(fname0, [240,320] )
        ksp_image1 = ksp.read_image(fname1, [240,320] )
        image0 = cv2.imread( fname0 )
        image1 = cv2.imread( fname1 )

        pts0, desc0, heatmap0 = ksp.get_descriptor( ksp_image0 )
        pts1, desc1, heatmap1 = ksp.get_descriptor( ksp_image1 )
        print 'len(pts0)=%d, len(pts1)=%d, desc0.shape=%s, desc1.shape=%s' %(pts0.shape[1], pts1.shape[1], str(desc0.shape), str(desc1.shape) )

        start_matching = time.time()
        matches = self.tracker.nn_match_two_way( desc0, desc1, self.fe.nn_thresh  )
        sel_pts0 = np.array([ pts0[:,i] for i in matches[0,:].astype('int32') ]).transpose()
        sel_pts1 = np.array([ pts1[:,i] for i in matches[1,:].astype('int32') ]).transpose()
        print 'matching dones in %4.2fms' %(1000. * (time.time() - start_matching) )
        print 'matches=%d' %( matches.shape[1] )
        if matches.shape[1] == 0:
            print 'NO MATCHES'
            return None, None

        # Plotting
        # xcanvas0 = self.plot_pts(image0.copy(), pts0 )
        # xcanvas1 = self.plot_pts(image1.copy(), pts1 )
        # xcanvas_ = self.plot_point_sets( xcanvas0, sel_pts0, xcanvas1, sel_pts1 )
        # xcanvas_ = self.plot_point_sets( image0.copy(), sel_pts0, image1.copy(), sel_pts1 )
        # cv2.imshow( 'pts0', xcanvas0 )
        # cv2.imshow( 'pts1', xcanvas1 )
        # cv2.imshow( 'joint', xcanvas_ )

        return sel_pts0[0:2,:].transpose() , sel_pts1[0:2,:].transpose()
        # code.interact( local=locals() )


if __name__ == '__main__':
    BASE = '/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/'


    #
    # Open Log File
    #
    LOG_FILE_NAME = BASE+'/log.json'
    print 'Open file: ', LOG_FILE_NAME
    with open(LOG_FILE_NAME) as data_file:
        data = json.load(data_file)


    #
    # Init Keras Model
    ksp = KSuperPointExp()

    if True:
        i = 534
        im = cv2.imread( BASE+'%d.jpg' %(i) )

        im_float = cv2.cvtColor( cv2.resize(im, (0,0), fx=0.5, fy=0.5), cv2.COLOR_BGR2GRAY ).astype('float32') / 255.
        print 'im_float.shape=', im_float.shape, '\tdtype=', im_float.dtype

        sup_pts, sup_desc, sup_heatmap = ksp.get_descriptor( im_float )
        cv2.imshow( str(i), ksp.plot_pts( im, sup_pts*2 ) )

    if True:
        i = 1638
        im = cv2.imread( BASE+'%d.jpg' %(i) )

        im_float = cv2.cvtColor( cv2.resize(im, (0,0), fx=0.5, fy=0.5), cv2.COLOR_BGR2GRAY ).astype('float32') / 255.
        print 'im_float.shape=', im_float.shape, '\tdtype=', im_float.dtype

        sup_pts, sup_desc, sup_heatmap = ksp.get_descriptor( im_float )
        cv2.imshow( str(i), ksp.plot_pts( im, sup_pts*2 ) )

    cv2.waitKey(0)


if __name__ == "__m1ain__":
    BASE = '/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/'


    #
    # Open Log File
    #
    LOG_FILE_NAME = BASE+'/log.json'
    print 'Open file: ', LOG_FILE_NAME
    with open(LOG_FILE_NAME) as data_file:
        data = json.load(data_file)


    #
    # Init Keras Model
    ksp = KSuperPointExp()



    # Loops over all images and precomputes their netvlad vector
    for i in range( len(data['DataNodes']) ):
        a = data['DataNodes'][i]['isKeyFrame']
        b = data['DataNodes'][i]['isImageAvailable']
        c = data['DataNodes'][i]['isPoseAvailable']
        d = data['DataNodes'][i]['isPtCldAvailable']


        if not ( a==1 and b==1 and c==1 and d==1 ): #only process keyframes which have pose and ptcld info
            continue

        im = cv2.imread( BASE+'%d.jpg' %(i) )

        im_float = cv2.cvtColor( cv2.resize(im, (0,0), fx=0.5, fy=0.5), cv2.COLOR_BGR2GRAY ).astype('float32') / 255.
        print 'im_float.shape=', im_float.shape, '\tdtype=', im_float.dtype

        start_time = time.time()
        sup_pts, sup_desc, sup_heatmap = ksp.get_descriptor( im_float )
        print '---', i , '\n',
        print 'Done in %4.4fms' %( 1000. * (time.time() - start_time ) )

        cv2.imshow( 'im_superpoints', ksp.plot_pts( im, sup_pts*2 ) )
        cv2.imshow( 'im', im )
        key = cv2.waitKey(10)
        if key == ord( 'q' ):
            break
