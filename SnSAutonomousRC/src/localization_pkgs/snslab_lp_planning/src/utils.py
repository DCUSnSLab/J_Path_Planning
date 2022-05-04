import rospy
import numpy as np
from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped, PoseArray
import tf.transformations
import tf
from sympy import *
import math
import copy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
import time

MINIMUM_DISTANCE_APPROACH = 0.3
CAR_WIDTH = 0.29
SAFETY_PERCENTAGE = 300.
FORWARD_INDEX = 180

class UserTransform():
    def __init__(self,):
        pass

    def coordinate_transform(self, coordinate, direction_radian, current_position):
        coor = np.array([])
        coordi = np.array([])
        if len(coordinate) !=0:

            for i in range(0,len(coordinate)):
                if direction_radian >= 0:
                    coor = np.append(coor,(coordinate[i][0]-current_position[0])*math.cos(direction_radian) + (coordinate[i][1]-current_position[1])*math.sin(direction_radian)) # 0
                    coor = np.append(coor, -(coordinate[i][0]-current_position[0])*math.sin(direction_radian) + (coordinate[i][1]-current_position[1])*math.cos(direction_radian)) # 0

                else:
                    coor = np.append(coor,(coordinate[i][0]-current_position[0])*math.cos(-direction_radian) - (coordinate[i][1]-current_position[1])*math.sin(-direction_radian)) # 0
                    coor = np.append(coor,(coordinate[i][0]-current_position[0])*math.sin(-direction_radian) + (coordinate[i][1]-current_position[1])*math.cos(-direction_radian)) # 0

            coordi = np.reshape(coor, (len(coordinate), 2))

            TrajectoryPoint().ros_marker2(coordi,1,0,0)
        return coordi

class TrajectoryPoint():
    def __init__(self):
        #self.waypointSub = rospy.Subscriber('/trajectory/current', PolygonStamped, self.waypointCallback)
        self.publisher = rospy.Publisher("visualization_maker_array", MarkerArray)
        self.markerArray = MarkerArray()

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
        if particleX!=None  and odomX != None:
            current_position = np.append(current_position, (particleX+odomX)/2)
            current_position = np.append(current_position, (particleY+odomY)/2)
            #print('p', particleX)
            #print('o', odomX)
        elif particleX !=None and odomX == None:
            current_position = np.append(current_position, particleX)
            current_position = np.append(current_position, particleY)
        elif odomX != None and particleX==None:
            current_position = np.append(current_position, odomX)
            current_position = np.append(current_position, odomY)
        return current_position

    def next_waypoint(self, waypointList, waypoint_idx):

        if waypointList[len(waypointList)-1][0] <= 2:
            waypoint_idx = 0
        else:
            # print('waypointList :', waypointList)
            # print('current_position :', current_position[0])
            # print('index :', waypoint_idx)
            # print('leaving waypoint length :', len(waypointList))
            if waypointList[0][0] <= 0:
                waypoint_idx += 1

        return waypoint_idx

    def ros_marker(self, waypointList, r, g, b):
        id = 0

        for waypoint in waypointList:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            marker.color.a = 1
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.id =id
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0

            self.markerArray.markers.append(marker)
            id +=1

        #print(self.markerArray)
        self.publisher.publish(self.markerArray)


    def ros_marker2(self, waypointList, r, g, b):
        id = 300

        for waypoint in waypointList:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.id =id
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0

            self.markerArray.markers.append(marker)
            id +=1

        #print(self.markerArray)
        self.publisher.publish(self.markerArray)


    def ros_marker3(self, waypointList, r, g, b):
        id = 400

        for waypoint in waypointList:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.id =id
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0

            self.markerArray.markers.append(marker)
            id +=1

        #print(self.markerArray)
        self.publisher.publish(self.markerArray)


    def ros_marker4(self, waypointList, r, g, b):
        id = 800

        for waypoint in waypointList:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.id =id
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0

            self.markerArray.markers.append(marker)
            id +=1

        #print(self.markerArray)
        self.publisher.publish(self.markerArray)

class ObstaclePositionEstimation():
    # Specify the waypoint range

    def __init__(self):
        self.width_to_cover = (CAR_WIDTH / 2) * (1 + SAFETY_PERCENTAGE / 100)

    def waypoint_range(self, waypointList, radians_per_point, waypoint_idx):
        # file = open('/home/jaehoon/SnSAutonomousRC/data_file/waypoint_distance_data2-1.txt', 'a')
        waypointRange = waypointList
        point_v = 0.0
        point_w = 0.0
        range_end_coordi = np.array([])
        vehicle_width_points = np.array([])
        waypoint_radian_points = np.array([])
        result_points_left = np.array([])
        result_points_right = np.array([])
        lidar_ranges = np.array([])
        waypointList_copy = copy.deepcopy(waypointList)

        distance_array = np.array([])  # data print array
        vehicle_width_radian_array = np.array([]) # data print array



        for w in waypointList_copy:
            waypoint_radian = np.arctan(w[1] / w[0]) # The angle of from vehicle to waypoint

            #print(waypoint_radian)
            distance = math.sqrt(w[0]**2 + w[1]**2)  # The distance from vehicle to waypoint (m)
            distance_array = np.append(distance_array, distance)
            #print('waypoint_distance', distance)
            vehicle_width_radian = 2 * np.arcsin(self.width_to_cover / (2 * distance))
            print('vehicle_width_radian_arcsin :', vehicle_width_radian)
            #print('vehicle_width_radian_arctan :', 2 * np.arctan2(self.width_to_cover,(2 * distance)))
            if math.isnan(vehicle_width_radian) == False:
                vehicle_width_radian_array = np.append(vehicle_width_radian_array, vehicle_width_radian)
                point_v = int(math.ceil(vehicle_width_radian / radians_per_point)) # The angle of from waypoint to end of waypoint in vehicle width
                point_w = FORWARD_INDEX + int(math.ceil(waypoint_radian / radians_per_point))  # The angle of from vehicle to waypoint in lidar ranges
                #point_w = int(math.ceil(waypoint_radian / radians_per_point))

            result_left = point_w + point_v   # The angle of from waypoint
            result_right = point_w - point_v

            #print('result_left', result_left)
            #print('result_right', result_right)
            vehicle_width_points = np.append(vehicle_width_points, point_v)
            waypoint_radian_points = np.append(waypoint_radian_points, point_w)
            result_points_left = np.append(result_points_left, result_left)
            result_points_right = np.append(result_points_right, result_right)

        # file.write('from vehicle to waypoint :' + str(distance_array[0]) + '\n\n')
        # file.write('x :' + str(w[0]) + ' y :' + str(w[1]) + '\n\n')
        # file.write('waypoint index :' + str(waypoint_idx) + '\n\n')
        # file.write('y :' + str(w[1]) + '\n\n')
        # # file.write('vehicle width radian :' + str(vehicle_width_radian_array[0]) + '\n\n')
        # file.write('left_radian :' + str(result_points_left[0]) + '\n\n')
        # file.write('right_radian :' + str(result_points_right[0]) + '\n\n')

        print('distance array :', distance_array)
        #print('vehicle_width_points', vehicle_width_points)
        print('waypoint_radian_points', waypoint_radian_points)
        #print('result_left', result_points_left)
        #print('result_right', result_points_right)
        #print('waypoint radain_points :', waypoint_radian_points[2])

        #file.close()

        # return 1 : Left radian value as far as the width of the vehicle around the waypoint
        # return 2 : Right radian value as far as the width of the vehicle around the waypoint
        return int(result_points_left[0]), int(result_points_right[0])

    def waypoint_angle(self, waypoint):
        angle = np.array([])
        for w in waypoint:
            angle = np.append(angle, math.atan2(w[1], w[0]))

        return angle


    def waypoint_side(self, waypoint_side, waypoint, radian_per_point):
        #file = open('/home/jaehoon/SnSAutonomousRC/data_file/waypoint_distance_data7.txt', 'a')
        waypoint_side = waypoint_side * math.pi/180
        waypoint_dist = math.sqrt(waypoint[0]**2 + waypoint[1]**2)

        x = math.cos(waypoint_side) * waypoint_dist  # The x coordinates separated by the width of the vehicle
        y = math.sin(waypoint_side) * waypoint_dist  # The y coordinates of right side separated by the width of the vehicle
        r = math.sqrt((waypoint[0] - x)**2 + (waypoint[1] - y)**2)  # radius of a virtual circle

        left_y = waypoint[1] + r    #
        right_y = waypoint[1] - r

        waypoint_up_side_radian, waypoint_side_distance = self.waypoint_up_side(waypoint, left_y, right_y, r, radian_per_point)

        left_side_radian = math.atan2(left_y, waypoint[0])
        right_side_radian = math.atan2(right_y, waypoint[0])



        print('waypoint (x,y) :', [waypoint[0],waypoint[1]])
        print('transform_waypoint_left_point :', np.array([waypoint[0], waypoint[1]+r]))
        print('transfrom_waypoint_right_point :', np.array([waypoint[0], waypoint[1]-r]))

        # file.write('x :' + str(x) + '\n\n')
        # file.write('y :' + str(y) + '\n\n')
        # file.write('waypoint distance :' + str(waypoint_dist) + '\n\n')
        # file.write('waypoint (x,y) : ' + str([waypoint[0],waypoint[1]]) + '\n\n')
        # file.write('waypoint side radian : ' + str(waypoint_side) + '\n\n')
        # file.write('transfrom_waypoint_left_point : ' + str(np.array([waypoint[0], waypoint[1]+r])) + '\n\n')
        # file.write('transfrom_waypoint_right_point : ' + str(np.array([waypoint[0], waypoint[1]-r])) + '\n\n')
        # file.write('radius : ' + str(r) + '\n\n')
        # file.close()

        #TrajectoryPoint().ros_marker3(a, 0, 1, 0)


        return waypoint_up_side_radian, waypoint_side_distance

        # # 1 : The radian of waypoint left side, 2 : The radian of waypoint right side, 3 : left side distance, 4 : right side distance
        # return int(math.ceil(left_side_radian / radian_per_point)), int(math.ceil(right_side_radian / radian_per_point)), \
        #        math.sqrt(waypoint[0]**2 + (waypoint[1]+r)**2), math.sqrt(waypoint[0]**2 + (waypoint[1]-r)**2)


        #return np.array([waypoint[0], waypoint[1]-r])  # 1 : waypoint_left_side, 2 : waypoint_right_side


    def waypoint_up_side(self, waypoint, waypoint_left_y, waypoint_right_y, radius, radian_per_point):
        waypoint_side_radian = np.array([], dtype=np.int)
        waypoint_side_distance = np.array([])
        a = np.array([])

        for i in range(0, 8, 2):
            side_radian = math.pi + math.atan2(waypoint_left_y, waypoint[0] + i/10)
            side_radian = int(math.ceil(side_radian / radian_per_point))
            waypoint_side_radian = np.append(waypoint_side_radian, side_radian)

            side_radian = math.pi + math.atan2(waypoint_right_y, waypoint[0] + i/10)
            side_radian = int(math.ceil(side_radian / radian_per_point))

            waypoint_side_radian = np.append(waypoint_side_radian, side_radian)
            waypoint_side_distance = np.append(waypoint_side_distance, math.sqrt((waypoint[0]+i/10)**2 + waypoint_left_y**2)) # The distance of left side
            waypoint_side_distance = np.append(waypoint_side_distance, math.sqrt((waypoint[0]+i/10)**2 + waypoint[1]**2))
            waypoint_side_distance = np.append(waypoint_side_distance, math.sqrt((waypoint[0]+i/10)**2 + waypoint_right_y**2)) # The distance of right side

            a = np.append(a, waypoint[0]+i/10)
            a = np.append(a, waypoint_left_y)
            a = np.append(a, waypoint[0] + i/10)
            a = np.append(a, waypoint_right_y)

        waypoint_side_radian = np.reshape(waypoint_side_radian, (int(len(waypoint_side_radian)/2), 2))
        waypoint_side_distance = np.reshape(waypoint_side_distance, (int(len(waypoint_side_distance)/3), 3))

        a = np.reshape(a, (int(len(a)/2), 2))
        TrajectoryPoint().ros_marker4(a, 0, 1, 0)

        print('w_left_side :', waypoint_side_radian[0][0])
        print('w_right_side :', waypoint_side_radian[0][1])

        print('left_dist :', math.sqrt(waypoint[0]**2 + waypoint_left_y**2))
        print('right_dist :', math.sqrt(waypoint[0]**2 + waypoint_right_y**2))

        return waypoint_side_radian, waypoint_side_distance

    def object_detection(self, lidar_data, waypoint_distance):
        print('lidar_data :', lidar_data)
        #print('lidar_data_min :', np.min(lidar_data))
        print('waypoint_side_distance2 :', waypoint_distance)
        object = False

        for i in range(len(waypoint_distance)):
            min_waypoint_distance = np.min(waypoint_distance[i])

            try:
                lidar_data_min = np.min(lidar_data)
            except:
                lidar_data_min = 0.0001
            if lidar_data_min <= min_waypoint_distance:
                object = True

        return object



    def object_detection2(self, waypoint_left_end, waypoint_right_end, waypoint, lidar_data):
        print('waypoint left-180 :', waypoint_left_end)
        print('waypoint right-180 :', waypoint_right_end)
        waypoint_left_endX = waypoint[0]
        waypoint_left_endY = math.tan(waypoint_left_end * math.pi/180) * waypoint[0]

        print('waypoint x :', waypoint[0])
        print('waypoint y :', waypoint[1])
        print('waypoint_left_end y :', waypoint_left_endY)
        waypoint_right_endX = waypoint[0]
        waypoint_right_endY = math.tan(waypoint_right_end * math.pi/180) * waypoint[0]
        print('tan', math.tan(waypoint_right_end * math.pi/180))
        print('radian', waypoint_right_end * math.pi/180)
        print('waypoint_right_end y :', waypoint_right_endY)

        waypoint_left_distance = math.sqrt(waypoint_left_endX**2 + waypoint_left_endY**2)
        waypoint_distance = math.sqrt(waypoint[0]**2 + waypoint[1]**2)
        waypoint_right_distance = math.sqrt(waypoint_right_endX**2 + waypoint_right_endY**2)
        print('waypoint_right_distance', waypoint_right_distance)


        # print('a', int((waypoint_left_end + waypoint_right_end)/2))
        # print(np.min(lidar_data[int((waypoint_left_end+waypoint_right_end)/2):waypoint_left_end]))
        # if np.min(lidar_data[int((waypoint_left_end+waypoint_right_end)/2):waypoint_left_end]) <= waypoint_left_distance:
        #     print(true)
        #
        # elif np.min(lidar_data[waypoint_right_end:(int(waypoint_left_end+waypoint_right_end)/2)]) <= waypoint_distance:
        #     print(false)

    def waypointInLidar(self, waypointList):
        start_waypoint = np.array([])
        projection = np.array([])
        opposite = np.array([])
        hypotenuse = np.array([])
        for waypoint in waypointList:
            projection = np.array([waypoint[0], current_position[1]])
            opposite = np.linalg.norm(projection - current_position)
            #hypotenuse = np.linalg.norm(waypoint - current_position)
            base = np.linalg.norm(waypoint - projection)
            oppoDivideBase = base / opposite
            #angle_between = oppoDivideBase
            angle_between = math.atan2(base, opposite)
            start_waypoint = np.append(start_waypoint, angle_between)
        #if len(start_waypoint)>0:
            #print('angle', start_waypoint[0] * (180/math.pi))
        # print('angle', radian * (180/math.pi))
        #print(len(waypointList))

    def determining_existence_obstacle(self, ranges, waypointList):
        pass
