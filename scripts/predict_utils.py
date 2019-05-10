import keras
import json
import pprint
import numpy as np
import cv2
import code
from keras import backend as K
from keras.engine.topology import Layer

# This was copied from the training code's CustomNets.py
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
        # print 'amap.shape', self.amap.shape

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

        # return [v, self.amap]
        return v

    def compute_output_shape( self, input_shape ):

        # return [(input_shape[0], self.K*self.D ), (input_shape[0], input_shape[1], input_shape[2]) ]
        return (input_shape[0], self.K*self.D )

    def get_config( self ):
        pass
        # base_config = super(NetVLADLayer, self).get_config()
        # return dict(list(base_config.items()))

        # As suggested by: https://github.com/keras-team/keras/issues/4871#issuecomment-269731817
        config = {'num_clusters': self.num_clusters}
        base_config = super(NetVLADLayer, self).get_config()
        return dict(list(base_config.items()) + list(config.items()))



class GhostVLADLayer( Layer ):

    def __init__( self, num_clusters, num_ghost_clusters, **kwargs ):
        self.num_clusters = num_clusters
        self.num_ghost_clusters = num_ghost_clusters
        super(GhostVLADLayer, self).__init__(**kwargs)

    def build( self, input_shape ):
        # self.K = self.num_clusters
        self.K = self.num_clusters + self.num_ghost_clusters
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
        # print 'amap.shape', self.amap.shape

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
        v = v[:,0:self.num_clusters,:]
        # print 'after ghosting v.shape', v.shape
        v = K.l2_normalize( v, axis=-1 )
        v = K.batch_flatten( v )
        v = K.l2_normalize( v, axis=-1 )


        # return [v, self.amap]
        return v

    def compute_output_shape( self, input_shape ):

        # return [(input_shape[0], self.num_clusters*self.D ), (input_shape[0], input_shape[1], input_shape[2]) ]
        return (input_shape[0], self.num_clusters*self.D )

    def get_config( self ):
        pass


        # As suggested by: https://github.com/keras-team/keras/issues/4871#issuecomment-269731817
        config = {'num_clusters': self.num_clusters, 'num_ghost_clusters': self.num_ghost_clusters}
        base_config = super(GhostVLADLayer, self).get_config()
        return dict(list(base_config.items()) + list(config.items()))


#--------------------------- UTILS --------------------------------------------#
def open_json_file( fname ):
    print 'Load JSON file: ', fname
    jsonX = json.loads(open(fname).read())
    return jsonX


def change_model_inputshape(model, new_input_shape=(None, 40, 40, 3), verbose=False):
    """
    Given a model and new input shape it changes all the allocations.

    Note: It uses custom_objects={'NetVLAD': NetVLADLayer}. If you have any other
    custom-layer change the code here accordingly.
    """
    # replace input shape of first layer
    model._layers[0].batch_input_shape = new_input_shape

    # feel free to modify additional parameters of other layers, for example...
    # model._layers[2].pool_size = (8, 8)
    # model._layers[2].strides = (8, 8)

    # rebuild model architecture by exporting and importing via json
    new_model = keras.models.model_from_json(model.to_json(), custom_objects={'NetVLADLayer': NetVLADLayer, 'GhostVLADLayer':GhostVLADLayer}  )
    # new_model.summary()

    # copy weights from old model to new one
    print 'copy weights from old model to new one....this usually takes upto 10 sec'
    for layer in new_model.layers:
        try:
            if verbose:
                print( 'transfer weights for layer.name='+layer.name )
            layer.set_weights(model.get_layer(name=layer.name).get_weights())
        except:
            print '[Almost certainly FATAL] '
            print 'If you see the warning like wieghts cannot be transfer, this is usually bad. In this case make sure the desired input dimensions and those in the modeljson file are compatible'
            print("Could not transfer weights for layer {}".format(layer.name))

    # test new model on a random input image
    # X = np.random.rand(new_input_shape[0], new_input_shape[1], new_input_shape[2], new_input_shape[3] )
    # y_pred = new_model.predict(X)
    # print('try predict with a random input_img with shape='+str(X.shape)+ str(y_pred) )

    return new_model

#--------------------------- END UTILS ----------------------------------------#
