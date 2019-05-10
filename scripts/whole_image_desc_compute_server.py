#!/usr/bin/env python

from cerebro.srv import *
import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import code
import time
import os
import scipy.io


import keras
from keras_helpers import *
from predict_utils import *
from TerminalColors import bcolors
tcol = bcolors()

import rospkg
THIS_PKG_BASE_PATH = rospkg.RosPack().get_path('cerebro')


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

class ReljaNetVLAD:
    def __init__(self, im_rows=600, im_cols=960, im_chnls=3):
        ## Build net
        from keras.backend.tensorflow_backend import set_session
        import tensorflow as tf
        config = tf.ConfigProto()
        config.gpu_options.per_process_gpu_memory_fraction = 0.1
        # config.gpu_options.visible_device_list = "0"
        set_session(tf.Session(config=config))

        self.im_rows = int(im_rows)
        self.im_cols = int(im_cols)
        self.im_chnls = int(im_chnls)


        input_img = keras.layers.Input( batch_shape=(1,self.im_rows, self.im_cols, self.im_chnls ) )
        cnn = make_from_vgg16( input_img, weights=None, layer_name='block5_pool' )
        out, out_amap = NetVLADLayer(num_clusters = 64)( cnn )
        model = keras.models.Model( inputs=input_img, outputs=out )

        DATA_DIR = '/app_learning/cartwheel_train/relja_matlab_weight.dump/'
        print 'Relja Data Dir: ', DATA_DIR
        model.load_weights( DATA_DIR+'/matlab_model.keras' )
        WPCA_M = scipy.io.loadmat( DATA_DIR+'/WPCA_1.mat' )['the_mat'] # 1x1x32768x4096
        WPCA_b = scipy.io.loadmat( DATA_DIR+'/WPCA_2.mat' )['the_mat'] # 4096x1
        WPCA_M = WPCA_M[0,0]          # 32768x4096
        WPCA_b = np.transpose(WPCA_b) #1x4096


        self.model = model
        self.WPCA_M = WPCA_M
        self.WPCA_b = WPCA_b
        self.model_type = 'relja_matlab_model'



        # Doing this is a hack to force keras to allocate GPU memory. Don't comment this,
        tmp_zer = np.zeros( (1,self.im_rows,self.im_cols,self.im_chnls), dtype='float32' )
        tmp_zer_out = self.model.predict( tmp_zer )
        tmp_zer_out = np.matmul( tmp_zer_out, self.WPCA_M ) + self.WPCA_b
        tmp_zer_out /= np.linalg.norm( tmp_zer_out )
        print 'model input.shape=', tmp_zer.shape, '\toutput.shape=', tmp_zer_out.shape
        print 'model_type=', self.model_type

        print '-----'
        print '\tinput_image.shape=', tmp_zer.shape
        print '\toutput.shape=', tmp_zer_out.shape
        print '\tminmax=', np.min( tmp_zer_out ), np.max( tmp_zer_out )
        print '\tnorm=', np.linalg.norm( tmp_zer_out )
        print '\tdtype=', tmp_zer_out.dtype
        print '-----'


    def handle_req( self, req ):
        ## Get Image out of req
        cv_image = CvBridge().imgmsg_to_cv2( req.ima )
        print '[Handle Request] cv_image.shape', cv_image.shape, '\ta=', req.a, '\tt=', req.ima.header.stamp


        assert (cv_image.shape[0] == self.im_rows and
                cv_image.shape[1] == self.im_cols and
                cv_image.shape[2] == self.im_chnls),\
                "\n[whole_image_descriptor_compute_server] Input shape of the image \
                does not match with the allocated GPU memory. Expecting an input image of \
                size %dx%dx%d, but received : %s" %(self.im_rows, self.im_cols, self.im_chnls, str(cv_image.shape) )

        # cv2.imshow( 'whole_image_descriptor_compute_server:imshow', cv_image.astype('uint8') )
        # cv2.waitKey(10)
        # cv2.imwrite( '/app/tmp/%s.jpg' %( str(req.ima.header.stamp) ), cv_image )

        ## Compute Descriptor
        start_time = time.time()

        # Normalize image
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        avg_image = [122.6778, 116.6522, 103.9997]
        cv_image_rgb[:,:,0]= cv_image_rgb[:,:,0] - avg_image[0]
        cv_image_rgb[:,:,1]= cv_image_rgb[:,:,1] - avg_image[1]
        cv_image_rgb[:,:,2]= cv_image_rgb[:,:,2] - avg_image[2]


        # predict
        u = self.model.predict( np.expand_dims( cv_image_rgb.astype('float32'), 0 ) )

        # WPCA
        u = np.matmul( u, self.WPCA_M ) + self.WPCA_b
        u /= np.linalg.norm( u )

        print 'Descriptor Computed in %4.4fms' %( 1000. *(time.time() - start_time) ),
        print '\tdesc.shape=', u.shape,
        print '\tinput_image.shape=', cv_image.shape,
        print '\tminmax=', np.min( u ), np.max( u ),
        print '\tmodel_type=', self.model_type,
        print '\tdtype=', cv_image.dtype



        ## Populate output message
        result = WholeImageDescriptorComputeResponse()
        # result.desc = [ cv_image.shape[0], cv_image.shape[1] ]
        result.desc = u[0,:]
        result.model_type = self.model_type
        return result




class NetVLADImageDescriptor:
    def __init__(self, im_rows=600, im_cols=960, im_chnls=3):
        ## Build net
        from keras.backend.tensorflow_backend import set_session
        import tensorflow as tf
        config = tf.ConfigProto()
        config.gpu_options.per_process_gpu_memory_fraction = 0.1
        # config.gpu_options.visible_device_list = "0"
        set_session(tf.Session(config=config))

        # Blackbox 4
        # self.im_rows = 512
        # self.im_cols = 640
        # self.im_chnls = 3

        # point grey
        # self.im_rows = 600
        # self.im_cols = 960
        # self.im_chnls = 3

        # EuroC
        # self.im_rows = 480
        # self.im_cols = 752
        # self.im_chnls = 3

        self.im_rows = int(im_rows)
        self.im_cols = int(im_cols)
        self.im_chnls = int(im_chnls)


        #----- @ INPUT LAYER
        input_img = keras.layers.Input( shape=(self.im_rows, self.im_cols, self.im_chnls) )

        #----- @ CNN
        # cnn = make_from_vgg16( input_img, weights=None, layer_name='block5_pool' )
        cnn = make_from_mobilenet( input_img, weights=None, layer_name='conv_pw_7_relu' )

        #----- @ DOWN-SAMPLE LAYER (OPTINAL)
        if False: #Downsample last layer (Reduce nChannels of the output.)
            cnn_dwn = keras.layers.Conv2D( 256, (1,1), padding='same', activation='relu' )( cnn )
            cnn_dwn = keras.layers.normalization.BatchNormalization()( cnn_dwn )
            cnn_dwn = keras.layers.Conv2D( 64, (1,1), padding='same', activation='relu' )( cnn_dwn )
            cnn_dwn = keras.layers.normalization.BatchNormalization()( cnn_dwn )
            cnn = cnn_dwn

        #----- @ NetVLADLayer
        out, out_amap = NetVLADLayer(num_clusters = 16)( cnn )

        model = keras.models.Model( inputs=input_img, outputs=out )
        model.summary()
        # `model_visual_fname` as None will disable writing the image file.
        model_visual_fname = None
        # model_visual_fname = '/app/core.png'
        if model_visual_fname is not None:
            print 'Writing Model Visual to: ', model_visual_fname
            keras.utils.plot_model( model, to_file=model_visual_fname, show_shapes=True )


        #----- @ Load Model Weights
        #TODO Read from config file the path of keras model
        # model_file = '/app/catkin_ws/src/cerebro/scripts/keras.models/core_model.keras'
        # model_type = 'test_keras_model'

        # model_file = '/app_learning/cartwheel_train/models.keras/vgg16/block5_pool_k16_tripletloss2/core_model.keras'
        # model_type = 'block5_pool_k16_tripletloss2'


        # model_file = '/app_learning/cartwheel_train/models.keras/mobilenet_conv7_quash_chnls_allpairloss/core_model.keras'
        # model_type = 'mobilenet_conv7_quash_chnls_allpairloss'

        #model_file = '/app_learning/cartwheel_train/models.keras/mobilenet_conv7_allpairloss/core_model.keras'
        #model_type = 'mobilenet_conv7_allpairloss'

        # model_file = '/app_learning/cartwheel_train/models.keras/mobilenet_new/pw13_quash_chnls_k16_allpairloss/core_model.1800.keras'
        # model_type = 'pw13_quash_chnls_k16_allpairloss'

        # model_file = '/app_learning/cartwheel_train/models.keras/vgg16_new/block5_pool_k16_tripletloss2/core_model.keras'
        # model_type = 'block5_pool_k16_tripletloss2'

        # model_file = '/app_learning/cartwheel_train/models.keras/mobilenet_new/pw13_quash_chnls_k16_allpairloss/core_model.800.keras'
        # model_type = 'pw13_quash_chnls_k16_allpairloss'

        model_file = THIS_PKG_BASE_PATH+'/scripts/keras.models/mobilenet_conv7_allpairloss.keras'
        model_type = 'mobilenet_conv7_allpairloss'

        print 'model_file: ', model_file
        model.load_weights( model_file )

        self.model = model
        self.model_type = model_type
        # ! Done...!

        # Doing this is a hack to force keras to allocate GPU memory. Don't comment this,
        tmp_zer = np.zeros( (1,self.im_rows,self.im_cols,self.im_chnls), dtype='float32' )
        tmp_zer_out = self.model.predict( tmp_zer )
        print 'model input.shape=', tmp_zer.shape, '\toutput.shape=', tmp_zer_out.shape
        print 'model_type=', self.model_type

        print '-----'
        print '\tinput_image.shape=', tmp_zer.shape
        print '\toutput.shape=', tmp_zer_out.shape
        print '\tminmax=', np.min( tmp_zer_out ), np.max( tmp_zer_out )
        print '\tdtype=', tmp_zer_out.dtype
        print '-----'



    def handle_req( self, req ):
        ## Get Image out of req
        cv_image = CvBridge().imgmsg_to_cv2( req.ima )
        print '[Handle Request] cv_image.shape', cv_image.shape, '\ta=', req.a, '\tt=', req.ima.header.stamp


        assert (cv_image.shape[0] == self.im_rows and
                cv_image.shape[1] == self.im_cols and
                cv_image.shape[2] == self.im_chnls),\
                "\n[whole_image_descriptor_compute_server] Input shape of the image \
                does not match with the allocated GPU memory. Expecting an input image of \
                size %dx%dx%d, but received : %s" %(self.im_rows, self.im_cols, self.im_chnls, str(cv_image.shape) )

        # cv2.imshow( 'whole_image_descriptor_compute_server:imshow', cv_image.astype('uint8') )
        # cv2.waitKey(10)
        # cv2.imwrite( '/app/tmp/%s.jpg' %( str(req.ima.header.stamp) ), cv_image )

        ## Compute Descriptor
        start_time = time.time()
        u = self.model.predict( np.expand_dims( cv_image.astype('float32'), 0 ) )
        print 'Descriptor Computed in %4.4fms' %( 1000. *(time.time() - start_time) ),
        print '\tdesc.shape=', u.shape,
        print '\tinput_image.shape=', cv_image.shape,
        print '\tminmax=', np.min( u ), np.max( u ),
        print '\tnorm=', np.linalg.norm(u[0]),
        print '\tmodel_type=', self.model_type,
        print '\tdtype=', cv_image.dtype



        ## Populate output message
        result = WholeImageDescriptorComputeResponse()
        # result.desc = [ cv_image.shape[0], cv_image.shape[1] ]
        result.desc = u[0,:]
        result.model_type = self.model_type
        return result


class JSONModelImageDescriptor:
    """
    This class loads the net structure from the json file, model.json and
    weights from core_model.??.keras. In the argument `kerasmodel_file`
    you need to specify the core_model.??.keras full path (keras model file).
    The model.json need to exist in that folder.
    """
    def __init__(self, kerasmodel_file, im_rows=600, im_cols=960, im_chnls=3):
        ## Build net
        from keras.backend.tensorflow_backend import set_session
        import tensorflow as tf
        config = tf.ConfigProto()
        config.gpu_options.per_process_gpu_memory_fraction = 0.2
        # config.gpu_options.visible_device_list = "0"
        set_session(tf.Session(config=config))

        # Blackbox 4
        # self.im_rows = 512
        # self.im_cols = 640
        # self.im_chnls = 3

        # point grey
        # self.im_rows = 600
        # self.im_cols = 960
        # self.im_chnls = 3

        # EuroC
        # self.im_rows = 480
        # self.im_cols = 752
        # self.im_chnls = 3

        self.im_rows = int(im_rows)
        self.im_cols = int(im_cols)
        self.im_chnls = int(im_chnls)

        LOG_DIR = '/'.join( kerasmodel_file.split('/')[0:-1] )
        print '+++++++++++++++++++++++++++++++++++++++++++++++++++++++'
        print '++++++++++ (JSONModelImageDescriptor) LOG_DIR=', LOG_DIR
        print '+++++++++++++++++++++++++++++++++++++++++++++++++++++++'
        model_type = LOG_DIR.split('/')[-1]
        # code.interact( local=locals() )

        assert os.path.isdir( LOG_DIR ), "The LOG_DIR doesnot exist, or there is a permission issue. LOG_DIR="+LOG_DIR
        assert os.path.isfile(  LOG_DIR+'/model.json'  ), "model.json does not exist in LOG_DIR="+LOG_DIR+'. This file is needed to load the network architecture from json format. This file should have been created when you learned the model using script in github.com/mpkuse/cartwheel_train'



        #----- @ Load Model Structure from JSON
        # Load JSON formatted model
        json_string = open_json_file( LOG_DIR+'/model.json' )
        # print '======================='
        # pprint.pprint( json_string, indent=4 )
        # print '======================='
        model = keras.models.model_from_json(str(json_string),  custom_objects={'NetVLADLayer': NetVLADLayer, 'GhostVLADLayer':GhostVLADLayer} )
        old_input_shape = model._layers[0].input_shape

        model._layers[0]
        print 'OLD MODEL: ', 'input_shape=', str(old_input_shape)
        model.summary()
        model_visual_fname = None
        # model_visual_fname = '/app/core.png'
        if model_visual_fname is not None:
            print 'Writing Model Visual to: ', model_visual_fname
            keras.utils.plot_model( model, to_file=model_visual_fname, show_shapes=True )

        #----- @ Load Weights
        assert os.path.isfile( kerasmodel_file ), 'The model weights file doesnot exists or there is a permission issue.'+"kerasmodel_file="+kerasmodel_file
        model_fname = kerasmodel_file
        print tcol.OKGREEN, 'Load model: ', model_fname, tcol.ENDC
        model.load_weights(  model_fname )


        # Replace Input Layer
        new_model = change_model_inputshape( model, new_input_shape=(1,self.im_rows,self.im_cols,self.im_chnls) )
        new_input_shape = new_model._layers[0].input_shape
        print 'OLD MODEL: ', 'input_shape=', str(old_input_shape)
        print 'NEW MODEL: input_shape=', str(new_input_shape)

        self.model = new_model
        self.model_type = model_type


        # Doing this is a hack to force keras to allocate GPU memory. Don't comment this,
        tmp_zer = np.zeros( (1,self.im_rows,self.im_cols,self.im_chnls), dtype='float32' )
        tmp_zer_out = self.model.predict( tmp_zer )
        print 'model input.shape=', tmp_zer.shape, '\toutput.shape=', tmp_zer_out.shape
        print 'model_type=', self.model_type

        print '-----'
        print '\tinput_image.shape=', tmp_zer.shape
        print '\toutput.shape=', tmp_zer_out.shape
        print '\tminmax=', np.min( tmp_zer_out ), np.max( tmp_zer_out )
        print '\tnorm=', np.linalg.norm( tmp_zer_out )
        print '\tdtype=', tmp_zer_out.dtype
        print '-----'



    def handle_req( self, req ):
        """ The received image from CV bridge has to be [0,255]. In function makes it to
        intensity range [-0.5 to 0.5]
        """
        ## Get Image out of req
        cv_image = CvBridge().imgmsg_to_cv2( req.ima )
        print '[Handle Request] cv_image.shape', cv_image.shape, '\ta=', req.a, '\tt=', req.ima.header.stamp
        if len(cv_image.shape)==2:
            # print 'Input dimensions are NxM but I am expecting it to be NxMxC, so np.expand_dims'
            cv_image = np.expand_dims( cv_image, -1 )
        elif len( cv_image.shape )==3:
            pass
        else:
            assert( False )




        assert (cv_image.shape[0] == self.im_rows and
                cv_image.shape[1] == self.im_cols and
                cv_image.shape[2] == self.im_chnls),\
                "\n[whole_image_descriptor_compute_server] Input shape of the image \
                does not match with the allocated GPU memory. Expecting an input image of \
                size %dx%dx%d, but received : %s" %(self.im_rows, self.im_cols, self.im_chnls, str(cv_image.shape) )

        # cv2.imshow( 'whole_image_descriptor_compute_server:imshow', cv_image.astype('uint8') )
        # cv2.waitKey(10)
        # cv2.imwrite( '/app/tmp/%s.jpg' %( str(req.ima.header.stamp) ), cv_image )

        ## Compute Descriptor
        start_time = time.time()
        # i__image = np.expand_dims( cv_image.astype('float32'), 0 )
        i__image = (np.expand_dims( cv_image.astype('float32'), 0 ) - 128.)/255.

        u = self.model.predict( i__image )

        print tcol.HEADER, 'Descriptor Computed in %4.4fms' %( 1000. *(time.time() - start_time) ), tcol.ENDC
        print '\tinput_image.shape=', cv_image.shape,
        print '\tinput_image dtype=', cv_image.dtype
        print tcol.OKBLUE, '\tinput image (to neuralnet) minmax=', np.min( i__image ), np.max( i__image ), tcol.ENDC
        print '\tdesc.shape=', u.shape,
        print '\tdesc minmax=', np.min( u ), np.max( u ),
        print '\tnorm=', np.linalg.norm(u[0])
        print '\tmodel_type=', self.model_type



        ## Populate output message
        result = WholeImageDescriptorComputeResponse()
        # result.desc = [ cv_image.shape[0], cv_image.shape[1] ]
        result.desc = u[0,:]
        result.model_type = self.model_type
        return result




rospy.init_node( 'whole_image_descriptor_compute_server' )

##
## Load the config file and read image row, col
##
fs_image_width = -1
fs_image_height = -1
fs_image_chnls = 1

if True: # read from param `config_file`
    if not rospy.has_param( '~config_file'):
        print 'FATAL...cannot find param ~config_file. This is needed to determine size of the input image to allocate GPU memory. If you do not specify the config_file, you need to atleast specify the nrows, ncols, nchnls'
        rospy.logerr( '[whole_image_descriptor_compute_server]FATAL...cannot find param ~config_file. This is needed to determine size of the input image to allocate GPU memory. If you do not specify the config_file, you need to atleast specify the nrows, ncols, nchnls' )

        if ( rospy.has_param( '~nrows') and rospy.has_param( '~ncols') ):
            print tcol.OKGREEN, 'However, you seem to have set the parameters nrows and ncols, so will read those.', tcol.ENDC
        else:
            quit()
        # quit only if you cannot see nrows and ncols

    else:
        config_file = rospy.get_param('~config_file')
        print '++++++++\n++++++++ config_file: ', config_file
        if not os.path.isfile(config_file):
            print 'FATAL...cannot find config_file: ', config_file
            rospy.logerr( '[whole_image_descriptor_compute_server]FATAL...cannot find config_file: '+ config_file )
            quit()


        print '++++++++ READ opencv-yaml file: ', config_file
        fs = cv2.FileStorage(config_file, cv2.FILE_STORAGE_READ)
        fs_image_width = int(  fs.getNode( "image_width" ).real() )
        fs_image_height = int( fs.getNode( "image_height" ).real() )
        print '++++++++ opencv-yaml:: image_width=', fs_image_width, '   image_height=', fs_image_height
        print '++++++++'


##
## Load nrows and ncols directly as config
##
if True:  # read from param `nrows` and `ncols`
    if fs_image_width < 0 :
        if ( not rospy.has_param( '~nrows') or not rospy.has_param( '~ncols') or not rospy.has_param( '~nchnls') ):
            print 'FATAL...cannot find param either of ~nrows, ~ncols, ~nchnls. This is needed to determine size of the input image to allocate GPU memory'
            rospy.logerr( '[whole_image_descriptor_compute_server]FATAL...cannot find param either of ~nrows, ~ncols, nchnls. This is needed to determine size of the input image to allocate GPU memory' )
            quit()
        else:
            fs_image_height = rospy.get_param('~nrows')
            fs_image_width = rospy.get_param('~ncols')
            fs_image_chnls = rospy.get_param('~nchnls')



            print '~nrows = ', fs_image_height, '\t~ncols = ', fs_image_width, '\t~nchnls = ', fs_image_chnls


##
## Start Server
##
#gpu_s = SampleGPUComputer()
# gpu_netvlad = NetVLADImageDescriptor( im_rows=fs_image_height, im_cols=fs_image_width )
# gpu_netvlad = ReljaNetVLAD( im_rows=fs_image_height, im_cols=fs_image_width )


# kerasmodel_file = '/models.keras/Apr2019/gray_conv6_K16Ghost1__centeredinput/core_model.%d.keras' %(500)
if rospy.has_param( '~kerasmodel_file'):
    kerasmodel_file = rospy.get_param('~kerasmodel_file')
else:
    print tcol.ERROR, 'FATAL...missing specification of model file. You need to specify ~kerasmodel_file', tcol.ENDC
    quit()

gpu_netvlad = JSONModelImageDescriptor( kerasmodel_file=kerasmodel_file, im_rows=fs_image_height, im_cols=fs_image_width, im_chnls=fs_image_chnls )


s = rospy.Service( 'whole_image_descriptor_compute', WholeImageDescriptorCompute, gpu_netvlad.handle_req  )
print 'whole_image_descriptor_compute_server is running'


# image = cv2.resize(  cv2.imread( '/app/lena.jpg'), (224,224) )
# print gpu_s.model.predict( np.expand_dims(image,0) )



rospy.spin()
