#!/usr/bin/env python

import numpy as np
import code
import cv2
import time

import json


import keras
class SampleGPUComputer:
    def __init__(self):
        self.model = keras.applications.vgg16.VGG16( weights=None)
        self.model.summary()

    def compute( self, im ):
        im = cv2.resize( im, (224,224) )
        preds = self.model.predict( np.expand_dims(im,0) )
        return preds

if __name__ == "__main__":
    BASE = '/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/'
    LOG_FILE_NAME = BASE+'/log.json'

    print 'Open file: ', LOG_FILE_NAME
    with open(LOG_FILE_NAME) as data_file:
        data = json.load(data_file)


    gpu_s = SampleGPUComputer()



    # Loops over all images and precomputes their netvlad vector
    for i in range( len(data['DataNodes']) ):
        a = data['DataNodes'][i]['isKeyFrame']
        b = data['DataNodes'][i]['isImageAvailable']
        c = data['DataNodes'][i]['isPoseAvailable']
        d = data['DataNodes'][i]['isPtCldAvailable']


        if not ( a==1 and b==1 and c==1 and d==1 ):
            continue

        im = cv2.imread( BASE+'%d.jpg' %(i) )

        start_time = time.time()
        print '---', i , '\n', gpu_s.compute( im )
        print 'Done in %4.4fms' %( 1000. * (time.time() - start_time ) )

        cv2.imshow( 'im', im )
        key = cv2.waitKey(10)
        if key == ord( 'q' ):
            break
