#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import LaserScan
from FGM_pathplanningpy import FollowTheGapMethod
from disparity_extender import DisparityExtenderMethod
from enum import Enum
from threading import Thread
from multiprocessing import Process
from multiprocessing import Value, Manager
import sys
from multiprocessing import Lock
import threading
import os

class AlgorithmType(Enum):
    algorithmFGM = 0
    DisparityExtender = 1
    processkill = 2

class AlgorithmSelector():
    def __init__(self):
        self.name = 'algorithmFGM'
        self.algorithm_num = 0
        #self.object = FollowTheGapMethod()
        self.flat = False
        self.lock = threading.Lock()

    def input_name(self):
        while not rospy.is_shutdown():
            print('FGM : 0, disparity extender : 1')
            self.lock.acquire()
            try:
                self.algorithm_num = int(input())
                self.flat = True
                self.name = AlgorithmType(self.algorithm_num).name
            finally:
                self.lock.release()
            #print(self.name)

    def selector(self,):
        v = Value('i', self.algorithm_num)
        p1 = Process(target=FollowTheGapMethod().run, args=(v,))
        p2 = Process(target=DisparityExtenderMethod().run)
        while not rospy.is_shutdown():
            if self.flat==True:
                if self.name == 'algorithmFGM':
                    p1.start()
                    p1.join()
                elif self.name=='DisparityExtender':
                    p2.start()
                    p2.join()
                self.flat = False

if __name__ == '__main__':
    #rospy.init_node('sdfjaklf')
    #rospy.init_node('Algorithm Controller_2')
    a = AlgorithmSelector()
    th1 = Thread(target=a.input_name)
    th2 = Thread(target=a.selector)
    th1.start()
    th2.start()
    th1.join()
    th2.join()

'''
class AlgorithmType(Enum):
    algorithmFGM = 0
    DisparityExtender = 1
    processkill = 2

class AlgorithmSelector():
    def __init__(self):
        self.name = 'algorithmFGM'
        self.algorithm_num = 0
        #self.object = FollowTheGapMethod()
        self.flat = False
        self.lock = threading.Lock()

    def input_name(self):
        while True:
            print('FGM : 0, disparity extender : 1')
            self.lock.acquire()
            try:
                self.algorithm_num = int(input())
                self.flat = True
                self.name = AlgorithmType(self.algorithm_num).name
            finally:
                self.lock.release()
            #print(self.name)

    def selector(self,):
        p1 = None
        p2 = None

        while True:
            if self.flat==True:
                if self.name == 'algorithmFGM':
                    p1 = Process(target=FollowTheGapMethod().run, daemon=True)
                    p1.start()
                    p1.join()
                elif self.name=='DisparityExtender':
                    p2 = Process(target=DisparityExtenderMethod().run)
                    p2.start()
                    p2.join()
                self.flat = False



if __name__ == '__main__':
    #rospy.init_node('Algorithm Controller')
    a = AlgorithmSelector()
    th1 = Thread(target=a.input_name)
    th2 = Thread(target=a.selector)
    th1.start()
    th2.start()
    th1.join()
    th2.join()
    rospy.Subscriber('')
'''