#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5  # max deceleration


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)  #new

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose =None 
        self.stopline_wp_idx = -1 
        self.base_waypoints = None 
        self.waypoints_2d = None
        self.waypoint_tree= None 

        #rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while (not rospy.is_shutdown()): # and (self.waypoint_tree is not None):
            if True and (self.pose is not None) and (self.base_waypoints is not None) and (self.waypoint_tree is not None):
                closest_waypoint_idx = self.get_closest_waypoint_id()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_id(self):
        x=self.pose.pose.position.x
        y=self.pose.pose.position.y
        closest_idx=self.waypoint_tree.query([x,y],1)[1]

        # Check if closest_idx is ahead of or behind the vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx -1]

        # Equ for hyperplane through closest coores
        cl_vect=np.array(closest_coord)
        prev_vect= np.array(prev_coord)
        pos_vect= np.array([x,y])

        # the following acts like ceil() (modulo lenth of array ... because we have around course ?!)
        val= np.dot(cl_vect-prev_vect,pos_vect-cl_vect) # positive when closest_idx is behind vehicle
        if val > 0:  # if indeed behind, pick the next waypoint instead
            closest_idx =(closest_idx +1 ) % len(self.waypoints_2d)
        return closest_idx



    def publish_waypoints(self,closest_idx):
        lane = self.generate_lane()
        self.final_waypoints_pub.publish(lane)

    def generate_lane(self):
        closest_idx = self.get_closest_waypoint_id()
        lane= Lane()
        lane.header=self.base_waypoints.header
        #lane.waypoints.header=self.base_waypoints.waypoints.header
        horizon_waypoints= self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        additional_waypoints = self.base_waypoints.waypoints[0: LOOKAHEAD_WPS-len(horizon_waypoints)]  # fill up the list respecting modulo
        horizon_waypoints.extend(additional_waypoints)

        discriminant = (self.stopline_wp_idx -(closest_idx + LOOKAHEAD_WPS))
        no_of_wpts= len(self.base_waypoints.waypoints)
        discriminant = ((discriminant + no_of_wpts/2) % no_of_wpts)-no_of_wpts/2

        #no_need_to_stop = (self.stopline_wp_idx ==-1) or (self.stopline_wp_idx >= closest_idx + LOOKAHEAD_WPS)
        no_need_to_stop = (self.stopline_wp_idx ==-1) or (discriminant>=0) 
        if no_need_to_stop:
            lane.waypoints = horizon_waypoints
        else:
            #lane.waypoints = horizon_waypoints
            lane.waypoints = self.decelerate_waypoints(horizon_waypoints,closest_idx)
        
        return lane
    
    def decelerate_waypoints(self, horizon_waypoints,closest_idx):
        # the waypoints start at actual postiion of the car to the horizon (plus LOOKAHEAD_WPS)
        temp = []   ## create a new list of differently spaced waypoints (assumption new waypoint every 20ms)
        for i,wp in enumerate(horizon_waypoints):

            p=Waypoint()
            p.pose=wp.pose

            stop_idx = max(self.stopline_wp_idx-closest_idx-2,0)

            distance = self.distance(horizon_waypoints,i,stop_idx) # sum of Euklidian distances between relevant waypoints
            vel= math.sqrt(2*MAX_DECEL * distance)
            if vel < 0.1:
                vel=0.0
            
            p.twist.twist.linear.x = min(vel,wp.twist.twist.linear.x )
            temp.append(p)

        return temp


    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree      =  KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, horizon_waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(horizon_waypoints[wp1].pose.pose.position, horizon_waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
