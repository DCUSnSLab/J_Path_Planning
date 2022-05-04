#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from utils import TrajectoryPoint, ObstaclePositionEstimation, UserTransform
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from FGM_pathplanningpy2 import FollowTheGapMethod
from KJH_pathplanning import AstarAndExpenderDisparity
from disparity_extender import DisparityExtenderMethod
from enum import Enum
from threading import Thread
import threading

import math

from squaternion import Quaternion

import tf.transformations
import tf

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
        self.utf = UserTransform()
        self.oe = ObstaclePositionEstimation()
        self.j = AstarAndExpenderDisparity()

        self.euler = 0
        self.lidar_data = []
        self.waypointList = np.array([])
        self.previous_firstValue = np.array([])
        self.waypointFlat = False
        self.particle_positionX = None
        self.particle_positionY = None
        self.particleX = None
        self.particleY = None
        self.orientationX = None
        self.orientationY = None
        self.orientationZ = None
        self.orientationW = None
        self.odomX = None
        self.odomY = None
        self.waypoint_idx = 0

        self.speedA = 0.0
        self.steering_angleA = 0.0


        self.br = tf.TransformBroadcaster()

    def rplidar_2d_callback(self, msg):
        #self.lidar_data = self.preprocess_lidar(msg.ranges)
        self.lidar_data = np.array(msg.ranges)
        self.radians_per_point = (2 * np.pi) / len(msg.ranges)
        if math.isnan(self.radians_per_point):
            self.radians_per_point = 0

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

        self.tp.ros_marker(self.waypointList, 0, 0, 1)

    def odomCallback(self, msg):
        #print(msg.pose.pose.orientation.x)
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        #print('\neuler',tf.transformations.euler_from_quaternion(quaternion))
        #print()
        #print('quaternion')
        #print(msg.pose.pose.orientation)
        pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, TrajectoryPoint().quaternion_to_angle(msg.pose.pose.orientation)])
        self.odomX = pose[0]
        self.odomY = pose[1]

    def particleCallback(self, msg):
        #print(msg.poses[0].orientation)

        positionX = np.array([])
        positionY = np.array([])
        orientationX = np.array([])
        orientationY = np.array([])
        orientationZ = np.array([])
        orientationW = np.array([])
        for pose in msg.poses:
            positionX = np.append(positionX, pose.position.x)
            positionY = np.append(positionY, pose.position.y)
            orientationX = np.append(orientationX, pose.orientation.x)
            orientationY = np.append(orientationY, pose.orientation.y)
            orientationZ = np.append(orientationZ, pose.orientation.z)
            orientationW = np.append(orientationW, pose.orientation.w)

        self.particle_positionX = positionX.mean()
        self.particle_positionY = positionY.mean()

        self.orientationX = orientationX.mean()
        self.orientationY = orientationY.mean()
        self.orientationZ = orientationZ.mean()
        self.orientationW = orientationW.mean()

        quaternion  = [self.orientationX, self.orientationY, self.orientationZ, self.orientationW]
        #self.euler = 3.14159-tf.transformations.euler_from_quaternion(quaternion)[2]
        self.euler = tf.transformations.euler_from_quaternion(quaternion)[2]
        # particle_positionX = self.particle_positionX * math.cos(self.euler) - self.particle_positionY * math.sin(self.euler)
        # particle_positionY = self.particle_positionX * math.sin(self.euler) + self.particle_positionY * math.cos(self.euler)

        #print(tf.transformations.euler_from_quaternion(quaternion)[2])

        #print("["+str(particle_positionX) + " " + str(particle_positionY) + "]")


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
        waypoint_transform = np.array([])
        while not rospy.is_shutdown():
            #if self.flat==True:
            if self.name == 'algorithmFGM':
                speed, steering_angle = FollowTheGapMethod().inputRawData(self.lidar_data)
            elif self.name=='DisparityExtender':
                speed, steering_angle = DisparityExtenderMethod().inputRawData(self.lidar_data)
                self.ackermannMSG.drive.speed = speed
                self.ackermannMSG.drive.steering_angle = steering_angle
            elif self.name=='J_pathplanning':

                object = False

                #speed, steering_angle = self.j.inputData(200, 160, self.lidar_data)

                # self.br.sendTransform((0.0, 0.0, 0.0),
                #               (0.0, 0.0, 0.0, 1.0),
                #               rospy.Time.now(),
                #               "laser",
                #               "map")

                if np.array_equal(self.waypointList, self.previous_firstValue) == False:
                    self.waypoint_idx = 0

                    current_positionList = self.tp.odom_and_particle(self.particle_positionX, self.particle_positionY, self.odomX, self.odomY)

                if len(self.waypointList) > 4:
                    waypoint_transform = self.utf.coordinate_transform(self.waypointList[:5], self.euler, current_positionList)
                elif len(self.waypointList < 4):
                    waypoint_transform = self.utf.coordinate_transform(self.waypointList[:2], self.euler,
                                                                       current_positionList)
                if len(self.waypointList) > 0:
                    self.waypoint_idx = self.tp.next_waypoint(waypoint_transform[self.waypoint_idx:], self.waypoint_idx)

                    # print('index :',self.waypoint_idx)

                if len(self.waypointList) > 4:
                    waypoint_left, waypoint_right =  self.oe.waypoint_range(waypoint_transform[self.waypoint_idx:], self.radians_per_point, self.waypoint_idx)
                    # print('waypoint left :', waypoint_left)
                    # print('waypoint right :', waypoint_right)

                    # if waypoint_transform[self.waypoint_idx][1] < 0:   # When the y-coordinate of the next waypoint is greater than zero
                    #     w_left_side_degree, w_right_side_degree, left_dist, right_dist = self.oe.waypoint_side(180-waypoint_right, waypoint_transform[self.waypoint_idx], self.radians_per_point)   # Find the angle of both sides of the next waypoint
                    # else:    # When the y-coordinate of the next waypoint is less than zero
                    #     w_left_side_degree, w_right_side_degree, left_dist, right_dist = self.oe.waypoint_side(waypoint_left-180, waypoint_transform[self.waypoint_idx], self.radians_per_point)   # Find the angle of both sides of the next waypoint

                    if waypoint_transform[self.waypoint_idx][1] < 0:   # When the y-coordinate of the next waypoint is greater than zero
                        waypoint_side_radian, waypoint_side_distance = self.oe.waypoint_side(180-waypoint_right, waypoint_transform[self.waypoint_idx], self.radians_per_point)   # Find the angle of both sides of the next waypoint
                    else:    # When the y-coordinate of the next waypoint is less than zero
                        waypoint_side_radian, waypoint_side_distance = self.oe.waypoint_side(waypoint_left-180, waypoint_transform[self.waypoint_idx], self.radians_per_point)   # Find the angle of both sides of the next waypoint

                    # print('waypoint side up coordinate :', waypoint_side_radian)
                    # print('waypoint_side_distance :', waypoint_side_distance)

                    for i in range(len(waypoint_side_radian)):
                        object = self.oe.object_detection(self.lidar_data[waypoint_side_radian[i][1]: waypoint_side_radian[i][0]], waypoint_side_distance[i])

                if object == True:
                    self.j.inputData(0, 0, self.lidar_data)
                    speed, steering_angle = DisparityExtenderMethod().inputRawData(self.lidar_data)


                else:
                    speed = self.speedA
                    steering_angle = self.steering_angleA

                    # print('waypoint :', waypoint_left[self.waypoint_idx][1])

                self.ackermannMSG.drive.speed = speed
                self.ackermannMSG.drive.steering_angle = steering_angle

                #waypoint_angle = self.oe.waypointInLidar(self.waypointList[self.waypoint_idx:self.waypoint_idx+4], current_positionList)

                # speed, steering_angle, self.waypoint_idx = AstarAndExpenderDisparity().inputData(self.lidar_data,
                #                                                                            self.current_positionList,
                #                                                                            self.waypoint_idx,
                #                                                                            self.waypointList[self.waypoint_idx:])


                    #speed, steering_angle = self.j.inputData(int(waypoint_left), int(waypoint_right), self.lidar_data)
                    #speed, steering_angle = FollowTheGapMethod().inputRawData2(int(waypoint_left), int(waypoint_right), self.lidar_data)

                    #print(speed, steering_angle)
                    #print(speed)
                    # self.ackermannMSG.drive.speed = speed
                    # self.ackermannMSG.drive.steering_angle = steering_angle
                self.previous_firstValue = self.waypointList

            elif self.name == 'processkill':
                exit()
            #print('lenth of waypoint :', len(self.waypointList))
            # self.ackermannMSG.drive.speed = self.speedA
            # self.ackermannMSG.drive.steering_angle = self.steering_angleA
            #self.ackermannMSG.drive.speed = speed
            #self.ackermannMSG.drive.steering_angle = steering_angle
            # self.ackermannMSG.drive.speed = 0
            # self.ackermannMSG.drive.steering_angle = 0
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