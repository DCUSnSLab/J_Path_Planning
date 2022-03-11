#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from utils import TrajectoryPoint, ObstaclePositionEstimation
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from FGM_pathplanningpy2 import FollowTheGapMethod
from KJH_pathplanning import AstarAndExpenderDisparity
from disparity_extender import DisparityExtenderMethod
from enum import Enum
from threading import Thread
import threading

class AlgorithmType(Enum):
    processkill = 0
    algorithmFGM = 1
    DisparityExtender = 2
    J_pathplanning = 3

class AlgorithmSelector():
    def __init__(self):
        self.name = 'DisparityExtender'
        self.algorithm_num = 0
        #self.object = FollowTheGapMethod()
        self.flat = False
        self.lock = threading.Lock()
        self.ackermannMSG = AckermannDriveStamped()

        self.drivePub = rospy.Publisher('/driveTest', AckermannDriveStamped, queue_size=1)
        self.rplidarSub = rospy.Subscriber('/scan', LaserScan, self.rplidar_2d_callback)
        self.initialposeSub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialposeCallback)
        self.goalDestinationSub = rospy.Subscriber('/move_base_simple/goal',
                                                   PoseStamped, self.goalDestinationCallback)
        self.waypointSub = rospy.Subscriber('/trajectory/current', PolygonStamped, self.waypointCallback)
        self.odomSub = rospy.Subscriber('/pf/pose/odom', Odometry, self.odomCallback)
        self.particleSub = rospy.Subscriber('/pf/viz/particles', PoseArray, self.particleCallback)
        self.driveSub = rospy.Subscriber('/drive', AckermannDriveStamped, self.driveCallback)

        self.tp = TrajectoryPoint()

        self.lidar_data = []
        self.waypointList = np.array([])
        self.previous_firstValue = np.array([])
        self.waypointFlat = False
        self.particleX = None
        self.particleY = None
        self.odomX = None
        self.odomY = None
        self.waypoint_idx = 0

        self.speedA = 0.0
        self.steering_angleA = 0.0

    def rplidar_2d_callback(self, msg):
        #self.lidar_data = self.preprocess_lidar(msg.ranges)
        self.lidar_data = msg.ranges
        self.radians_per_point = (2 * np.pi) / len(msg.ranges)
    def initialposeCallback(self, msg):
        pass

    def goalDestinationCallback(self, msg):
        #self.waypointFlat = True
        pass

    def waypointCallback(self, msg):
        #print(type(msg.polygon.points))
        waypoint = np.array([])
        #self.waypointFlat = True
        for value in msg.polygon.points:
            waypoint = np.append(waypoint, value.x)
            waypoint = np.append(waypoint, value.y)

        self.waypointList = np.reshape(waypoint, (int((len(waypoint) / 2)), 2)) # Each waypoint x,y coordinate in 2d array

    def odomCallback(self, msg):
        pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, TrajectoryPoint().quaternion_to_angle(msg.pose.pose.orientation)])
        self.odomX = pose[0]
        self.odomY = pose[1]

    def particleCallback(self, msg):
        #print(msg.poses[0].position)
        positionX = np.array([])
        positionY = np.array([])
        for pose in msg.poses:
            positionX = np.append(positionX, pose.position.x)
            positionY = np.append(positionY, pose.position.y)
        self.particle_PositionX = positionX.mean()
        self.particle_PositionY = positionY.mean()
        #print('position X :', positionX)
        #print('position Y :', positionY)
        #print('position X :', self.particle_PositionX)
        #print('position Y :', self.particle_PositionY)

    def driveCallback(self, msg):
        self.speedA = msg.drive.speed
        self.steering_angleA = msg.drive.steering_angle

    def input_name(self):
        while not rospy.is_shutdown():
            print('1 : Follow the Gap Method(FGM)\n'
                  '2 : disparity extender\n'
                  '3 : J_pathplanning\n'
                  '0 : exit')
            self.lock.acquire()
            try:
                self.algorithm_num = int(input())
                self.name = AlgorithmType(self.algorithm_num).name
            finally:
                self.lock.release()
            #print(self.name)

    def selector(self,):
        #print(self.waypointFlat)
        speed = 0.0
        steering_angle = 0.0
        while not rospy.is_shutdown():
            #if self.flat==True:
            if self.name == 'algorithmFGM':
                speed, steering_angle = FollowTheGapMethod().inputRawData(self.lidar_data)
            elif self.name=='DisparityExtender':
                speed, steering_angle = DisparityExtenderMethod().inputRawData(self.lidar_data)
            elif self.name=='J_pathplanning':
                if np.array_equal(self.waypointList,self.previous_firstValue) == False:
                    self.waypoint_idx = 0

                current_positionList = self.tp.odom_and_particle(self.particleX, self.particleY, self.odomX, self.odomY)
                self.waypoint_idx = self.tp.next_waypoint(self.waypointList[self.waypoint_idx:], current_positionList, self.waypoint_idx)
                waypoint_range = ObstaclePositionEstimation().waypoint_range(self.waypointList[self.waypoint_idx:], current_positionList,
                                                                             self.radians_per_point)
                waypoint_angle = ObstaclePositionEstimation().waypointInLidar(self.waypointList[self.waypoint_idx:self.waypoint_idx+4], current_positionList)

                # speed, steering_angle, self.waypoint_idx = AstarAndExpenderDisparity().inputData(self.lidar_data,
                #                                                                            self.current_positionList,
                #                                                                            self.waypoint_idx,
                #                                                                            self.waypointList[self.waypoint_idx:])
                self.previous_firstValue = self.waypointList
            elif self.name == 'processkill':
                exit()
            #print('lenth of waypoint :', len(self.waypointList))
            self.ackermannMSG.drive.speed = self.speedA
            self.ackermannMSG.drive.steering_angle = self.steering_angleA
            self.drivePub.publish(self.ackermannMSG)

if __name__ == '__main__':
    rospy.init_node('algorithm_controller')
    #rospy.init_node('Algorithm Controller_2')
    a = AlgorithmSelector()
    th1 = Thread(target=a.input_name)
    th2 = Thread(target=a.selector)
    th1.start()
    th2.start()
    th1.join()
    th2.join()