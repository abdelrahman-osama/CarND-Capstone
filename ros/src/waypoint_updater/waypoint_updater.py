#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.currpos = None
        self.waypoints = None
        self.stopline_wp_idx = -1

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()



    def pose_cb(self, msg):
        # TODO: Implement
        self.currpos = msg.pose
        

    def waypoints_cb(self, waypoints):
        # TODO: Implement 
        self.waypoints = waypoints.waypoints
        
        
        



    def main(self):
        if (self.currpos != None and self.waypoints!=None):
            start = 0
            closestdist = 999999.0

            x = self.currpos.position.x
            y = self.currpos.position.y
       # pass
            for i in range(0, len(self.waypoints)):
                map_x = self.waypoints[i].pose.pose.position.x
                map_y = self.waypoints[i].pose.pose.position.y
                dist = math.sqrt((x-map_x)*(x-map_x) + (y-map_y)*(y-map_y))
                if (dist < closestdist):
                        closestdist = dist
                        start = i


            car_orientation = self.currpos.orientation
            quaternion = (car_orientation.x, car_orientation.y, car_orientation.z, car_orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]

            cx = self.waypoints[start].pose.pose.position.x
            cy = self.waypoints[start].pose.pose.position.y
            heading = math.atan2((cy - self.currpos.position.y), (cx - self.currpos.position.x))
            angle = abs(yaw - heading)
            if (angle > math.pi/4):
                  start += 1
                  if (start > len(self.waypoints)-1):
                    start = 0

            if (len(self.waypoints)-start) <= LOOKAHEAD_WPS:
                final = self.waypoints[start:]
            else:
                final = self.waypoints[start : start+LOOKAHEAD_WPS]

            l= Lane()
            if(self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= start+LOOKAHEAD_WPS)):
                l.waypoints = final
            else:
                l.waypoints = self.decelerate_waypoints(final, start)
            l.header.stamp = rospy.Time(0)
            self.final_waypoints_pub.publish(l)
        

   
        
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2*MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp

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

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
