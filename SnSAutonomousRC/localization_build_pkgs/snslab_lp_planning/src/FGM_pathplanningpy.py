#!/usr/bin/env python3
import rospy
import numpy as np
import sys
from threading import Thread
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import LaserScan


class FollowTheGapMethod():
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 3
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_SPEED = 8.0
    CORNERS_SPEED = 5.0
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    def __init__(self,):
        # used when calculating the angles of the LiDAR data
        self.drive = None
        self.points = []
        self.distances = []    # The distance between waypoint
        self.radians_per_elem = None
        self.speed = 0.0
        self.steering_angle = 0.0

        self.ackermannMSG = AckermannDriveStamped()
        self.driveSub = rospy.Subscriber('/drive', AckermannDriveStamped, self.drivingCallback)
        self.waypointSub = rospy.Subscriber('/trajectory/current', PolygonStamped, self.way_pointCallback)
        self.rplidarSub = rospy.Subscriber('/scan', LaserScan, self.rplidar2DCallback)
        self.drivePub = rospy.Publisher('/driveTest', AckermannDriveStamped, queue_size=1)
        print('FGM start')
    #def __del__(self):
        #print('Sutdown FGM Algorithm')


    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        #print('ranges', len(ranges))
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        # we won't use the LiDAR data from directly behind us
        # proc_ranges = np.array(ranges[45:-45])  # real lidar
        proc_ranges = np.array(ranges[45:-45]) # simulator lidar

        # sets each value to the mean over a given window
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE # Moving average
        #print(proc_ranges)
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST) # n[i] < 0 -> n[i]=0,  n[i]>3000000 -> n[i]=3000000
        #print(proc_ranges)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
            free_space_ranges: list of LiDAR data which contains a 'bubble' of zeros
        """
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # get slice for each contigous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        # I think we will only ever have a maximum of 2 slices but will handle an
        # indefinitely sized list for portablility

        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        #print(chosen_slice.start)
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        #print(averaged_max_gap)
        #print(averaged_max_gap.argmax())
        #print(averaged_max_gap.argmax() + start_i)
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """
        Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle

    def process_lidar(self, ranges):
        """
        Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(ranges)
        # Find closest point to LiDAR
        closest = proc_ranges.argmin() # return index of min value in list
        # Eliminate all points inside 'bubble' (set them to zero)
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0  # Value of minimum distance obstacle is zero
        #print(proc_ranges)

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Publish Drive message
        steering_angle = self.get_angle(best, len(proc_ranges))
        if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
            speed = self.CORNERS_SPEED
        else:
            speed = self.STRAIGHTS_SPEED
        print('Steering angle in degrees: {}'.format((steering_angle / (np.pi / 2)) * 90))
        return speed, steering_angle


    def rplidar2DCallback(self, msg):
        self.speed, self.steering_angle = self.process_lidar(msg.ranges)
        #print(msg.ranges[135])
        #proc_ranges = np.array(ranges[135:-135])
        #print(self.speed)
        #print(self.steering_angle)
        self.ackermannMSG.drive.speed = self.speed
        self.ackermannMSG.drive.steering_angle = self.steering_angle
        self.drivePub.publish(self.ackermannMSG)
        #print(msg.ranges[-135])

    def drivingCallback(self, msg):
        self.drive=msg.drive
        #print(self.drive)

    def fromPolygon(self, trajMsg):
        for p in trajMsg.points:
            self.points.append((p.x, p.y))
            if p.z >= 0:
                self.speed_profile.append(p.z)
        self.update_distances()
        #print("Loaded new trajectory with:", len(self.points), "points")

    def update_distances(self):
        num_distances = len(self.distances)
        num_points = len(self.points)
        #print(num_distances)
        #print(num_points)
        for i in range(num_distances, num_points):
            if i == 0:
                self.distances.append(0)
            else:
                p0 = self.points[i - 1]
                p1 = self.points[i]
                delta = np.array([p0[0] - p1[0], p0[1] - p1[1]])
                self.distances.append(self.distances[i - 1] + np.linalg.norm(delta))
        #print(self.distances)

    def way_pointCallback(self, msg):
        waypoint = msg.polygon
        self.fromPolygon(waypoint)


'''
    def shutdownProcess(self, v):
        while True:
            if v!=0:
                sys.exit(0)
        print(v.value)


    def rosinit(self):
        self.driveSub = rospy.Subscriber('/drive', AckermannDriveStamped, self.drivingCallback)
        self.waypointSub = rospy.Subscriber('/trajectory/current', PolygonStamped, self.way_pointCallback)
        self.rplidarSub = rospy.Subscriber('/scan', LaserScan, self.rplidar2DCallback)
        self.drivePub = rospy.Publisher('/driveTest', AckermannDriveStamped, queue_size=1)
        rospy.spin()

    def run(self, v):
        rospy.init_node('FGM_Algorithm')
        value = v.value

        th1 = Thread(target=self.rosinit)
        #th2 = Thread(target=self.shutdownProcess, args=(value, ))
        th2 = Thread(target=self.shutdownProcess, args=(value))
        th1.start()
        th1.join()
        th2.start()
        th2.join()
'''



if __name__ == '__main__':
    rospy.init_node('FGM_Algorithm')
    FGM = FollowTheGapMethod()
    rospy.spin()
