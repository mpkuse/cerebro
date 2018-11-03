#!/usr/bin/env python

from cerebro.srv import *
import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import code
import time


import keras

class SampleGPUComputer:
    def __init__(self):
        self.model = keras.applications.vgg16.VGG16( weights=None)
        self.model._make_predict_function()
        self.model.summary()



    def handle_req( self, req ):
        print 'Handle request '


        # print req
        cv_image = CvBridge().imgmsg_to_cv2( req.ima )
        print 'cv_image.shape', cv_image.shape
        # cv2.imshow( 'cv_image from server', cv_image )
        # cv2.waitKey(0)

        #
        # compute descriptor
        #
        # code.interact( local=locals() )
        im = cv2.resize( cv_image, (224,224) )
        self.model._make_predict_function()
        preds = self.model.predict( np.expand_dims(im,0).astype('float32') )
        print preds


        #
        # Return the descriptor in the message
        #
        result = WholeImageDescriptorComputeResponse()
        result.desc = [ cv_image.shape[0], cv_image.shape[1] ]
        return result


from keras_helpers import *
class NetVLADImageDescriptor:
    def __init__(self):
        ## Build net
        #TODO Read from config file the shape of the images

        # Blackbox 4
        self.im_rows = 512
        self.im_cols = 640
        self.im_chnls = 3

        # EuroC
        # self.im_rows = 480
        # self.im_cols = 752
        # self.im_chnls = 3


        input_img = keras.layers.Input( shape=(self.im_rows, self.im_cols, self.im_chnls) )
        cnn = make_from_mobilenet( input_img )
        out, out_amap = NetVLADLayer(num_clusters = 16)( cnn )
        model = keras.models.Model( inputs=input_img, outputs=out )
        model.summary()
        model_visual_fname = '/app/core.png'
        print 'Writing Model Visual to: ', model_visual_fname
        keras.utils.plot_model( model, to_file=model_visual_fname, show_shapes=True )


        ## Load Model Weights
        #TODO Read from config file the path of keras model
        model_file = '/app/catkin_ws/src/cerebro/scripts/keras.models/core_model.keras'
        print 'model_file: ', model_file
        model.load_weights( model_file )

        self.model = model
        self.model.predict( np.zeros( (1,self.im_rows,self.im_cols,self.im_chnls), dtype='float32' ) )


    def handle_req( self, req ):
        ## Get Image out of req
        cv_image = CvBridge().imgmsg_to_cv2( req.ima )
        print '[Handle Request] cv_image.shape', cv_image.shape, '\ta=', req.a


        assert (cv_image.shape[0] == self.im_rows and
                cv_image.shape[1] == self.im_cols and
                cv_image.shape[2] == self.im_chnls),\
                "\n[whole_image_descriptor_compute_server] Input shape of the image \
                does not match with the allocated GPU memory. Expecting an input image of \
                size %dx%dx%d, but received : %s" %(self.im_rows, self.im_cols, self.im_chnls, str(cv_image.shape) )


        ## Compute Descriptor
        start_time = time.time()
        u = self.model.predict( np.expand_dims( cv_image.astype('float32'), 0 ) )
        print 'Descriptor Computed in %4.4fms' %( 1000. *(time.time() - start_time) ), '\tdesc.shape=', u.shape, '\tinput_image.shape=', cv_image.shape, '\tdtype=', cv_image.dtype



        ## Populate output message
        result = WholeImageDescriptorComputeResponse()
        # result.desc = [ cv_image.shape[0], cv_image.shape[1] ]
        result.desc = u[0,:]
        return result



rospy.init_node( 'whole_image_descriptor_compute_server' )
#gpu_s = SampleGPUComputer()
gpu_netvlad = NetVLADImageDescriptor()
s = rospy.Service( 'whole_image_descriptor_compute', WholeImageDescriptorCompute, gpu_netvlad.handle_req  )
print 'whole_image_descriptor_compute_server is running'


# image = cv2.resize(  cv2.imread( '/app/lena.jpg'), (224,224) )
# print gpu_s.model.predict( np.expand_dims(image,0) )



rospy.spin()
