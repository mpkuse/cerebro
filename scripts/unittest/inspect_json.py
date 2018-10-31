#!/usr/bin/env python

import numpy as np
import code
import cv2

import json

if __name__ == "__main__":
    # LOG_FILE_NAME = '/Bulk_Data/_tmp_cerebro/euroc_MH_01_easy/log.json'
    LOG_FILE_NAME = '/Bulk_Data/_tmp_cerebro/bb4_multiple_loops_in_lab/log.json'

    print 'Open file: ', LOG_FILE_NAME
    with open(LOG_FILE_NAME) as data_file:
        data = json.load(data_file)


    print "len(data['DataNodes'])=", len(data['DataNodes'])
    Q = {}
    Q1 = []
    for i in range(  len(data['DataNodes']) ):
        S = ""
        S += str(data['DataNodes'][i]['isKeyFrame'])
        S += str(data['DataNodes'][i]['isImageAvailable'])
        S += str(data['DataNodes'][i]['isPoseAvailable'])
        S += str(data['DataNodes'][i]['isPtCldAvailable'])
        # print i, S

        Q1.append( S )
        if S not in Q.keys():
            Q[S] = 1
        else:
            Q[S] += 1
    print 'Q : ', Q
