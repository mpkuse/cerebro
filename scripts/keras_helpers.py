from keras import backend as K
from keras.engine.topology import Layer
import keras
import code
import numpy as np

import cv2
import code


#---------------------------------------------------------------------------------
# My Layers
#   NetVLADLayer
#---------------------------------------------------------------------------------

# Writing your own custom layers
class MyLayer(Layer):

    def __init__(self, output_dim, **kwargs):
        self.output_dim = output_dim
        super(MyLayer, self).__init__(**kwargs)

    def build(self, input_shape):
        # Create a trainable weight variable for this layer.
        self.kernel = self.add_weight(name='kernel',
                                      shape=(input_shape[1], self.output_dim),
                                      initializer='uniform',
                                      trainable=True)
        super(MyLayer, self).build(input_shape)  # Be sure to call this at the end

    def call(self, x):
        return [K.dot(x, self.kernel), K.dot(x, self.kernel)]

    def compute_output_shape(self, input_shape):
        return [(input_shape[0], self.output_dim), (input_shape[0], self.output_dim)]


class NetVLADLayer( Layer ):

    def __init__( self, num_clusters, **kwargs ):
        self.num_clusters = num_clusters
        super(NetVLADLayer, self).__init__(**kwargs)

    def build( self, input_shape ):
        self.K = self.num_clusters
        self.D = input_shape[-1]

        self.kernel = self.add_weight( name='kernel',
                                    shape=(1,1,self.D,self.K),
                                    initializer='uniform',
                                    trainable=True )

        self.bias = self.add_weight( name='bias',
                                    shape=(1,1,self.K),
                                    initializer='uniform',
                                    trainable=True )

        self.C = self.add_weight( name='cluster_centers',
                                shape=[1,1,1,self.D,self.K],
                                initializer='uniform',
                                trainable=True)

    def call( self, x ):
        # soft-assignment.
        s = K.conv2d( x, self.kernel, padding='same' ) + self.bias
        a = K.softmax( s )
        self.amap = K.argmax( a, -1 )
        print 'amap.shape', self.amap.shape

        # Dims used hereafter: batch, H, W, desc_coeff, cluster
        a = K.expand_dims( a, -2 )
        # print 'a.shape=',a.shape

        # Core
        v = K.expand_dims(x, -1) + self.C
        # print 'v.shape', v.shape
        v = a * v
        # print 'v.shape', v.shape
        v = K.sum(v, axis=[1, 2])
        # print 'v.shape', v.shape
        v = K.permute_dimensions(v, pattern=[0, 2, 1])
        # print 'v.shape', v.shape
        #v.shape = None x K x D

        # Normalize v (Intra Normalization)
        v = K.l2_normalize( v, axis=-1 )
        v = K.batch_flatten( v )
        v = K.l2_normalize( v, axis=-1 )

        return [v, self.amap]

    def compute_output_shape( self, input_shape ):
        # return (input_shape[0], self.v.shape[-1].value )
        # return [(input_shape[0], self.K*self.D ), (input_shape[0], self.amap.shape[1].value, self.amap.shape[2].value) ]
        return [(input_shape[0], self.K*self.D ), (input_shape[0], input_shape[1], input_shape[2]) ]


#--------------------------------------------------------------------------------
# Base CNNs
#--------------------------------------------------------------------------------

# def make_vgg( input_img ):
#     r_l2=keras.regularizers.l2(0.01)
#     r_l1=keras.regularizers.l1(0.01)
#
#     x_64 = keras.layers.Conv2D( 64, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( input_img )
#     x_64 = keras.layers.normalization.BatchNormalization()( x_64 )
#     x_64 = keras.layers.Conv2D( 64, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_64 )
#     x_64 = keras.layers.normalization.BatchNormalization()( x_64 )
#     x_64 = keras.layers.MaxPooling2D( pool_size=(2,2), padding='same' )( x_64 )
#
#
#     x_128 = keras.layers.Conv2D( 128, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_64 )
#     x_128 = keras.layers.normalization.BatchNormalization()( x_128 )
#     x_128 = keras.layers.Conv2D( 128, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_128 )
#     x_128 = keras.layers.normalization.BatchNormalization()( x_128 )
#     x_128 = keras.layers.MaxPooling2D( pool_size=(2,2), padding='same' )( x_128 )
#
#
#     # x_256 = keras.layers.Conv2D( 256, (3,3), padding='same', activation='relu' )( x_128 )
#     # x_256 = keras.layers.normalization.BatchNormalization()( x_256 )
#     # x_256 = keras.layers.Conv2D( 256, (3,3), padding='same', activation='relu' )( x_256 )
#     # x_256 = keras.layers.normalization.BatchNormalization()( x_256 )
#     # x_256 = keras.layers.MaxPooling2D( pool_size=(2,2), padding='same' )( x_256 )
#
#     #
#     # x_512 = keras.layers.Conv2D( 512, (3,3), padding='same', activation='relu' )( x_256 )
#     # # BN
#     # x_512 = keras.layers.Conv2D( 512, (3,3), padding='same', activation='relu' )( x_512 )
#     # # BN
#     # x_512 = keras.layers.MaxPooling2D( pool_size=(2,2), padding='same' )( x_512 )
#
#
#     x = keras.layers.Conv2DTranspose( 32, (5,5), strides=4, padding='same' )( x_128 )
#     # x = x_128
#
#     return x
#
#
# def make_upsampling_vgg( input_img  ):
#     r_l2=keras.regularizers.l2(0.01)
#     r_l1=keras.regularizers.l1(0.01)
#
#     x_64 = keras.layers.Conv2D( 64, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( input_img )
#     x_64 = keras.layers.normalization.BatchNormalization()( x_64 )
#     x_64 = keras.layers.Conv2D( 64, (3,3), strides=2, padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_64 )
#     x_64 = keras.layers.normalization.BatchNormalization()( x_64 )
#
#     x_128 = keras.layers.Conv2D( 128, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_64 )
#     x_128 = keras.layers.normalization.BatchNormalization()( x_128 )
#     x_128 = keras.layers.Conv2D( 128, (3,3), strides=2, padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_128 )
#     x_128 = keras.layers.normalization.BatchNormalization()( x_128 )
#
#     x_256 = keras.layers.Conv2D( 128, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_128 )
#     x_256 = keras.layers.normalization.BatchNormalization()( x_256 )
#     x_256 = keras.layers.Conv2D( 128, (3,3), strides=2, padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_256 )
#     x_256 = keras.layers.normalization.BatchNormalization()( x_256 )
#
#     z = keras.layers.Conv2DTranspose( 32, (11,11), strides=8, padding='same' )( x_256 )
#     x = keras.layers.Conv2DTranspose( 32, (9,9), strides=4, padding='same' )( x_128 )
#     y = keras.layers.Conv2DTranspose( 32, (7,7), strides=2, padding='same' )( x_64 )
#
#     out = keras.layers.Add()( [x,y,z] )
#     return out
#
# def make_from_vgg19( input_img, trainable=True, weights=None ):
#     base_model = keras.applications.vgg19.VGG19(weights=weights, include_top=False, input_tensor=input_img)
#
#     for l in base_model.layers:
#         l.trainable = trainable
#
#     base_model_out = base_model.get_layer('block2_pool').output
#
#     z = keras.layers.Conv2DTranspose( 32, (9,9), strides=4, padding='same' )( base_model_out )
#     return z
#
# def make_from_vgg19_multiconvup( input_img, trainable=True, weights=None ):
#     base_model = keras.applications.vgg19.VGG19(weights=weights, include_top=False, input_tensor=input_img)
#
#     for l in base_model.layers:
#         l.trainable = trainable
#         #TODO : add kernel regularizers and activity_regularizer to conv layers
#
#     base_model_out = base_model.get_layer('block2_pool').output
#
#     up_conv_out = keras.layers.Conv2DTranspose( 32, (9,9), strides=2, padding='same', activation='relu' )( base_model_out )
#     up_conv_out = keras.layers.normalization.BatchNormalization()( up_conv_out )
#
#     up_conv_out = keras.layers.Conv2DTranspose( 32, (9,9), strides=2, padding='same', activation='relu' )( up_conv_out )
#     up_conv_out = keras.layers.normalization.BatchNormalization()( up_conv_out )
#
#
#     return up_conv_out
#
#
# def make_from_mobilenet( input_img=None, weights=None ):
#     # input_img = keras.layers.BatchNormalization()(input_img)
#
#     base_model = keras.applications.mobilenet.MobileNet( weights=weights, include_top=False, input_tensor=input_img )
#     # base_model = keras.applications.mobilenet.MobileNet( weights=None, include_top=False, input_tensor=input_img )
#     # keras.utils.plot_model( base_model, to_file='base_model.png', show_shapes=True )
#
#     # Pull out a layer from original network
#     base_model_out = base_model.get_layer( 'conv_pw_7_relu').output # can also try conv_pw_7_relu etc.
#
#     # Up-sample
#     # Basic idea is to try upsampling without using the transposed-conv layers. Instead use either of
#     #   a) upsampling2d
#     #   b) depth to space
#     # followed by CBR (conv-BN-relu)
#     # TODO
#     ups_out = base_model_out
#     # ups_out = keras.layers.UpSampling2D( size=(4,4) )( base_model_out )
#
#     # model = keras.models.Model( inputs=input_img, outputs=ups_out )
#     # model.summary()
#     # keras.utils.plot_model( model, to_file='base_model.png', show_shapes=True )
#     # code.interact( local=locals() )
#
#     return ups_out
#
#




#--------------------------------------------------------------------------------
# Base CNNs
#--------------------------------------------------------------------------------

def make_vgg( input_img ):
    r_l2=keras.regularizers.l2(0.01)
    r_l1=keras.regularizers.l1(0.01)

    x_64 = keras.layers.Conv2D( 64, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( input_img )
    x_64 = keras.layers.normalization.BatchNormalization()( x_64 )
    x_64 = keras.layers.Conv2D( 64, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_64 )
    x_64 = keras.layers.normalization.BatchNormalization()( x_64 )
    x_64 = keras.layers.MaxPooling2D( pool_size=(2,2), padding='same' )( x_64 )


    x_128 = keras.layers.Conv2D( 128, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_64 )
    x_128 = keras.layers.normalization.BatchNormalization()( x_128 )
    x_128 = keras.layers.Conv2D( 128, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_128 )
    x_128 = keras.layers.normalization.BatchNormalization()( x_128 )
    x_128 = keras.layers.MaxPooling2D( pool_size=(2,2), padding='same' )( x_128 )


    # x_256 = keras.layers.Conv2D( 256, (3,3), padding='same', activation='relu' )( x_128 )
    # x_256 = keras.layers.normalization.BatchNormalization()( x_256 )
    # x_256 = keras.layers.Conv2D( 256, (3,3), padding='same', activation='relu' )( x_256 )
    # x_256 = keras.layers.normalization.BatchNormalization()( x_256 )
    # x_256 = keras.layers.MaxPooling2D( pool_size=(2,2), padding='same' )( x_256 )

    #
    # x_512 = keras.layers.Conv2D( 512, (3,3), padding='same', activation='relu' )( x_256 )
    # # BN
    # x_512 = keras.layers.Conv2D( 512, (3,3), padding='same', activation='relu' )( x_512 )
    # # BN
    # x_512 = keras.layers.MaxPooling2D( pool_size=(2,2), padding='same' )( x_512 )


    x = keras.layers.Conv2DTranspose( 32, (5,5), strides=4, padding='same' )( x_128 )
    # x = x_128

    return x


def make_upsampling_vgg( input_img  ):
    r_l2=keras.regularizers.l2(0.01)
    r_l1=keras.regularizers.l1(0.01)

    x_64 = keras.layers.Conv2D( 64, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( input_img )
    x_64 = keras.layers.normalization.BatchNormalization()( x_64 )
    x_64 = keras.layers.Conv2D( 64, (3,3), strides=2, padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_64 )
    x_64 = keras.layers.normalization.BatchNormalization()( x_64 )

    x_128 = keras.layers.Conv2D( 128, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_64 )
    x_128 = keras.layers.normalization.BatchNormalization()( x_128 )
    x_128 = keras.layers.Conv2D( 128, (3,3), strides=2, padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_128 )
    x_128 = keras.layers.normalization.BatchNormalization()( x_128 )

    x_256 = keras.layers.Conv2D( 128, (3,3), padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_128 )
    x_256 = keras.layers.normalization.BatchNormalization()( x_256 )
    x_256 = keras.layers.Conv2D( 128, (3,3), strides=2, padding='same', activation='relu', kernel_regularizer=r_l2, activity_regularizer=r_l1 )( x_256 )
    x_256 = keras.layers.normalization.BatchNormalization()( x_256 )

    z = keras.layers.Conv2DTranspose( 32, (11,11), strides=8, padding='same' )( x_256 )
    x = keras.layers.Conv2DTranspose( 32, (9,9), strides=4, padding='same' )( x_128 )
    y = keras.layers.Conv2DTranspose( 32, (7,7), strides=2, padding='same' )( x_64 )

    out = keras.layers.Add()( [x,y,z] )
    return out



def make_from_vgg19_multiconvup( input_img, trainable=False ):
    base_model = keras.applications.vgg19.VGG19(weights='imagenet', include_top=False, input_tensor=input_img)

    for l in base_model.layers:
        l.trainable = trainable
        #TODO : add kernel regularizers and activity_regularizer to conv layers

    base_model_out = base_model.get_layer('block2_pool').output

    up_conv_out = keras.layers.Conv2DTranspose( 32, (9,9), strides=2, padding='same', activation='relu' )( base_model_out )
    up_conv_out = keras.layers.normalization.BatchNormalization()( up_conv_out )

    up_conv_out = keras.layers.Conv2DTranspose( 32, (9,9), strides=2, padding='same', activation='relu' )( up_conv_out )
    up_conv_out = keras.layers.normalization.BatchNormalization()( up_conv_out )

    return up_conv_out


def make_from_mobilenet( input_img, weights=None, layer_name='conv_pw_7_relu' ):
    # input_img = keras.layers.BatchNormalization()(input_img)

    base_model = keras.applications.mobilenet.MobileNet( weights=weights, include_top=False, input_tensor=input_img )

    # Pull out a layer from original network
    base_model_out = base_model.get_layer( layer_name ).output # can also try conv_pw_7_relu etc.

    return base_model_out





def make_from_vgg16( input_img, weights='imagenet', trainable=False, layer_name='block2_pool' ):
    base_model = keras.applications.vgg16.VGG16(weights=weights, include_top=False, input_tensor=input_img)

    for l in base_model.layers:
        l.trainable = trainable

    base_model_out = base_model.get_layer(layer_name).output
    return base_model_out
