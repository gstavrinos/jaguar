#!/usr/bin/env python
import os
import tf
import math
import time
import rospy
import rospkg
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap

odom = None
dir_ = ''
file_id = 0

def odom_callback(odom_):
    global odom
    odom = odom_

def trav_callback(input_tm_):
    global odom, dir_, file_id
    resolution = input_tm_.info.resolution;
    trav = ['traversability_slope', 'traversability_step', 'traversability_roughness']
    pos = [odom.pose.pose.position.x, odom.pose.pose.position.z, odom.pose.pose.position.y]
    slope = []
    step = []
    roughness = []
    (rr, rp, ry) = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
    for t in trav:
        index_of_interest = -1
        for i in range(len(input_tm_.layers)):
            if t == input_tm_.layers[i]:
                index_of_interest = i
                break
        if index_of_interest >= 0:
            index_x = input_tm_.data[index_of_interest].layout.dim[1].size;
            index_y = input_tm_.data[index_of_interest].layout.dim[0].size;
            for i in range(len(input_tm_.data[index_of_interest].data)):
                cellv = input_tm_.data[index_of_interest].data[i];
                if cellv == cellv:
                    cx = resolution * (index_x / 2 - i % index_x);
                    cy = resolution * (index_y / 2 - i / index_y);
                    d = math.sqrt(cx**2 + cy**2);
                    cellx = d * math.cos(math.atan(cy/cx)+ry) + pos[0];
                    celly = d * math.sin(math.atan(cy/cx)+ry) + pos[1];
                    if t == 'traversability_slope':
                        slope.append([cellx, celly, cellv])
                    elif t == 'traversability_step':
                        step.append([cellx, celly, cellv])
                    elif t == 'traversability_roughness':
                        roughness.append([cellx, celly, cellv])
    with open(dir_+str(file_id)+'.txt', 'a+') as log:
        first = True
        for p in pos:
            if not first:
                log.write(',')
            log.write(str(p))
            first = False

        log.write('\n#\n')

        first = True
        for s in slope:
            for s1 in s:
                if not first:
                    log.write(',')
                log.write(str(s1))
                first = False
            log.write('\n')
            first = True
        log.write('#\n')

        first = True
        for s in step:
            for s1 in s:
                if not first:
                    log.write(',')
                log.write(str(s1))
                first = False
            log.write('\n')
            first = True
        log.write('#\n')

        first = True
        for r in roughness:
            for r1 in r:
                if not first:
                    log.write(',')
                log.write(str(r1))
                first = False
            log.write('\n')
            first = True
        log.write('###\n')




if __name__ == '__main__':
    rospy.init_node('traversability_data_collector')
    topic  = rospy.get_param('~odom_topic', 'odom/perfect')
    topic2  = rospy.get_param('~odom_topic', '/traversability_estimation/traversability_map')
    rospack = rospkg.RosPack()
    dir_ = rospack.get_path('jaguar_navigation')+'/logs/'
    if not os.path.exists(dir_):
        os.makedirs(dir_)
    while(os.path.exists(dir_+str(file_id)+'.txt')):
        file_id += 1
    rospy.Subscriber(topic, Odometry, odom_callback)
    rospy.Subscriber(topic2, GridMap, trav_callback)
    rospy.spin()