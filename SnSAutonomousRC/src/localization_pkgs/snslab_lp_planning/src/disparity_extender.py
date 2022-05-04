#!/usr/bin/env python3
import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import LaserScan
import time
import math

class DisparityExtenderMethod:
    CAR_WIDTH = 0.31  # simulator car
    # CAR_WIDTH = 0.45 # snslab car
    # the min difference between adjacent LiDAR points for us to call them disparate
    DIFFERENCE_THRESHOLD = 2.
    SPEED = 5.
    # the extra safety room we plan for along walls (as a percentage of car_width/2)
    SAFETY_PERCENTAGE = 300.

    def __int__(self):
        self.speed = 0.0
        self.speed_angle = 0.0

    def preprocess_lidar(self, ranges):
        """ Any preprocessing of the LiDAR data can be done in this function.
            Possible Improvements: smoothing of outliers in the data and placing
            a cap on the maximum distance a point can be.
        """
        # remove quadrant of LiDAR directly behind us
        eighth = int(len(ranges) / 8)
        #print(len(np.array(ranges[eighth:-eighth])))
        return np.array(ranges[eighth:-eighth])

    def get_differences(self, ranges):
        """ Gets the absolute difference between adjacent elements in
            in the LiDAR data and returns them in an array.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        differences = [0.]  # set first element to 0
        for i in range(1, len(ranges)):
            differences.append(abs(ranges[i] - ranges[i - 1])) # Index of difference is 0~270
        return differences

    def get_disparities(self, differences, threshold):
        """ Gets the indexes of the LiDAR points that were greatly
            different to their adjacent point.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        disparities = []
        #print(differences)
        for index, difference in enumerate(differences): # index : 0~269
            if difference > threshold:
                disparities.append(index) # index : 1 ~ 269
        #print(disparities)
        return disparities

    def get_num_points_to_cover(self, dist, width):
        """ Returns the number of LiDAR points that correspond to a width at
            a given distance.
            We calculate the angle that would span the width at this distance,
            then convert this angle to the number of LiDAR points that
            span this angle.
            Current math for angle:
                sin(angle/2) = (w/2)/d) = w/2d
                angle/2 = sininv(w/2d)
                angle = 2sininv(w/2d)
                where w is the width to cover, and d is the distance to the close
                point.
            Possible Improvements: use a different method to calculate the angle
        """
        angle = 2 * np.arcsin(width / (2 * dist))

        #print('width', width)
        #print('dist', dist)
        #print('angle', angle)
        #print('radians_per_point', self.radians_per_point)

        if math.isnan(angle) :  # if angle is NAN
            num_points = 0

        else:
            num_points = int(np.ceil(angle / self.radians_per_point))
            #print('num_points', num_points)

        return num_points

    def cover_points(self, num_points, start_idx, cover_right, ranges):
        """ 'covers' a number of LiDAR points with the distance of a closer
            LiDAR point, to avoid us crashing with the corner of the car.
            num_points: the number of points to cover
            start_idx: the LiDAR point we are using as our distance
            cover_right: True/False, decides whether we cover the points to
                         right or to the left of start_idx
            ranges: the LiDAR points

            Possible improvements: reduce this function to fewer lines
        """
        new_dist = ranges[start_idx]
        if cover_right:
            for i in range(num_points):
                next_idx = start_idx + 1 + i
                if next_idx >= len(ranges): break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        else:
            for i in range(num_points):
                next_idx = start_idx - 1 - i
                if next_idx < 0: break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        return ranges

    def extend_disparities(self, disparities, ranges, car_width, extra_pct):
        """ For each pair of points we have decided have a large difference
            between them, we choose which side to cover (the opposite to
            the closer point), call the cover function, and return the
            resultant covered array.
            Possible Improvements: reduce to fewer lines
        """
        num_points_to_cover = 0
        width_to_cover = (car_width / 2) * (1 + extra_pct / 100)
        #print(disparities)

        for index in disparities:
            first_idx = index - 1
            points = ranges[first_idx:first_idx + 2]

            if points == []:
                break
            close_idx = first_idx + np.argmin(points)
            far_idx = first_idx + np.argmax(points)
            close_dist = ranges[close_idx]
            num_points_to_cover = self.get_num_points_to_cover(close_dist,
                                                               width_to_cover)
            cover_right = close_idx < far_idx
            #print('num', num_points_to_cover)

            if num_points_to_cover == 0:
                ranges = []
                break
            else:
                ranges = self.cover_points(num_points_to_cover, close_idx,
                                       cover_right, ranges)

        if num_points_to_cover == 0:
            ranges = []
        return ranges

    def get_steering_angle(self, range_index, range_len):
        """ Calculate the angle that corresponds to a given LiDAR point and
            process it into a steering angle.
            Possible improvements: smoothing of aggressive steering angles
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_point
        steering_angle = np.clip(lidar_angle, np.radians(-90), np.radians(90))
        return steering_angle

    def _process_lidar(self, ranges):
        """ Run the disparity extender algorithm!
            Possible improvements: varying the speed based on the
            steering angle or the distance to the farthest point.
        """
        self.radians_per_point = (2 * np.pi) / len(ranges)
        proc_ranges = self.preprocess_lidar(ranges)
        differences = self.get_differences(proc_ranges) # two point difference of distance
        disparities = self.get_disparities(differences, self.DIFFERENCE_THRESHOLD)

        proc_ranges = self.extend_disparities(disparities, proc_ranges,
                                          self.CAR_WIDTH, self.SAFETY_PERCENTAGE)
        #print(proc_ranges)
        if proc_ranges == []:
            #print(3)
            speed = 0
            steering_angle = 0
        #print(proc_ranges.argmax())
        else:
            steering_angle = self.get_steering_angle(proc_ranges.argmax(),
                                                 len(proc_ranges))
            speed = self.SPEED
        return speed, steering_angle

    def inputRawData(self, data):
        if len(data)==0:
            self.speed, self.steering_angle = 0.0, 0.0
        else:
            self.speed, self.steering_angle = self._process_lidar(data)
        return self.speed, self.steering_angle

    def process_observation(self, ranges, ego_odom):
        return self._process_lidar(ranges)