import rospy
import numpy as np
from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped, PoseArray
import tf.transformations
import tf
from sympy import *
import math

MINIMUM_DISTANCE_APPROACH = 0.3
CAR_WIDTH = 0.29
SAFETY_PERCENTAGE = 300.

class TrajectoryPoint():
    def __init__(self):
        #self.waypointSub = rospy.Subscriber('/trajectory/current', PolygonStamped, self.waypointCallback)
        pass

    # def waypointCallback(self, msg):
    #     waypoint = np.array([])
    #     for value in msg.polygon.points:
    #         waypoint = np.append(waypoint, value.x)
    #         waypoint = np.append(waypoint, value.y)
    #     self.waypointList = np.reshape(waypoint, (int((len(waypoint) / 2)), 2))

    def quaternion_to_angle(self,q):
        """Convert a quaternion _message_ into an angle in radians.
        The angle represents the yaw.
        This is not just the z component of the quaternion."""
        x, y, z, w = q.x, q.y, q.z, q.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
        return yaw

    def odom_and_particle(self, particleX, particleY, odomX, odomY):
        # self.particleX = particleX
        # self.particleY = particleY
        # self.odomX = odomX
        # self.odomY = odomY
        current_position = np.array([])
        if particleX and particleY and odomX and odomY != None:
            current_position = np.append(current_position, (particleX+odomX)/2)
            current_position = np.append(current_position, (particleY+odomY/2))
        if particleX and particleY !=None:
            current_position = np.append(current_position, particleX)
            current_position = np.append(current_position, particleY)
        if odomX and odomY !=None:
            current_position = np.append(current_position, odomX)
            current_position = np.append(current_position, odomY)
        return current_position

    def next_waypoint(self, waypointList, current_position, waypoint_idx):
        if len(waypointList) <= 1:
            index=0
        else:
            index = waypoint_idx
            distance = np.linalg.norm(current_position - waypointList[1])
            waypoint_distance = np.linalg.norm(waypointList[1] - waypointList[0])
            #print('distance :', distance)
            #print('waypoint_distance :',waypoint_distance)
            if abs(distance - waypoint_distance) < MINIMUM_DISTANCE_APPROACH:
                index += 1
            if distance > 3:
                index = 0
            #print('index:', index)
            #print('leaving waypoint length :', len(waypointList))
        return index

class ObstaclePositionEstimation():
    # Specify the waypoint range
    def waypoint_range(self, waypointList, current_position, radians_per_point):
        waypointRange = waypointList

        # The distance between the current car and the waypoint
        if len(waypointList) > 1:
            distance = np.linalg.norm(current_position - waypointList[1])
            width_to_cover = (CAR_WIDTH / 2) * (1 + SAFETY_PERCENTAGE / 100)

            radian = 2 * np.arcsin(width_to_cover / (2 * distance))
            if radian != nan:
                #print('angle', radian * (180/math.pi))
                num_points = int(np.ceil(radian / radians_per_point))
                #print('num_points', num_points)

        self.waypointInLidar(waypointList, current_position)

        #hypotenuse = (car_width)
        ##MINIMUM_DISTANCE_APPROACH

        #x, y, = symbols('x y')
        #print(solve([Eq(x ** 2 - y ** 2, CAR_WIDTH/2), Eq(2 * x ** 2 - x * y + y ** 2, 8)], [x, y]))

        #return num_points

    def waypointInLidar(self, waypointList, current_position):
        start_waypoint = np.array([])
        projection = np.array([])
        opposite = np.array([])
        hypotenuse = np.array([])
        for waypoint in waypointList:
            projection = np.array([waypoint[0], current_position[1]])
            opposite = np.linalg.norm(projection - current_position)
            hypotenuse = np.linalg.norm(waypoint - projection)
            oppoDivideHypo = hypotenuse / opposite
            angle_between = np.arctan(oppoDivideHypo)
            start_waypoint = np.append(start_waypoint, angle_between)
        if len(start_waypoint)>0:
            print('angle', start_waypoint[0] * (180/math.pi))
        # print('angle', radian * (180/math.pi))
        #print(len(waypointList))

    def determining_existence_obstacle(self, ranges, waypointList):
        pass
