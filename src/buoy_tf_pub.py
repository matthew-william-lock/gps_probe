#!/usr/bin/env python

import rospy

import numpy as np

# from geometry_msgs.msg import Quaternion, TransformStamped
# from sensor_msgs.msg import NavSatFix
# from nav_msgs.msg import Odometry
# import tf
# from geodesy import utm
# import numpy as np
# import tf2_ros
# import message_filters

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion, TransformStamped

import tf
import tf2_ros

from geodesy import utm

import time

class MeasureBuoyPositions(object):
    
    def trigger_measurement_callback(self, msg):
        rospy.loginfo("Trigger measurement callback")
        self.measure_now = True
        
    def publisher_transform(self, navSatFix):
        
        num_buoys = len(self.buoy_positions)
        buoy_frame = "buoy_{}_frame".format(num_buoys)
        
        self.buoy_positions.append(navSatFix)
        
        buoy_utm = utm.fromLatLong(navSatFix.latitude, navSatFix.longitude)
        
        try:
            (world_trans, world_rot) = self.listener.lookupTransform(self.utm_frame, 
                                                                    buoy_frame,
                                                                    rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException):
            rospy.loginfo("GPS node: broadcasting transform %s to %s" % (self.utm_frame, buoy_frame))            
            transformStamped = TransformStamped()
            quat = tf.transformations.quaternion_from_euler(np.pi, -np.pi/2., 0., axes='rxzy')
            transformStamped.transform.translation.x = buoy_utm.easting
            transformStamped.transform.translation.y = buoy_utm.northing
            transformStamped.transform.translation.z = 0.
            transformStamped.transform.rotation = Quaternion(*quat)               
            transformStamped.header.frame_id = self.utm_frame
            transformStamped.child_frame_id = buoy_frame
            transformStamped.header.stamp = rospy.Time.now()
            self.static_tf_bc.sendTransform(transformStamped)
            
    def sam_gps(self, msg):
        self.last_gps_msg = msg
    
    
    def __init__(self):
        
        self.gps_topic = rospy.get_param('~gps_topic', 'fix')
        self.utm_frame = rospy.get_param('~utm_frame', 'utm')
        
        self.last_gps_msg = None # type: NavSatFix
        
        self.gps_sam_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.sam_gps)
        
        # Broadcast UTM to map frame
        self.listener = tf.TransformListener()        
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster()
        
        rospy.loginfo("Measure buoy positions node started")
        
        self.buoy_positions = []
        
        # String subscriber
        self.trigger_measurement = rospy.Subscriber("/sam/trigger_measurement", String, self.trigger_measurement_callback)
        self.measure_now = False
        
    def loop(self):
        
        # Two measurements per second
        self.rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            
            if self.measure_now:
                
                if self.last_gps_msg is None:
                    rospy.logwarn("No GPS message received yet")
                    self.measure_now = False
                    
                measurements = []
                    
                for i in range(0, 10):
                    
                    # Get lat and lon
                    lat = self.last_gps_msg.latitude
                    lon = self.last_gps_msg.longitude
                    
                    measurements.append(self.last_gps_msg)
                    rospy.loginfo("Measurement {} of 10 : {} {}".format(i, lat, lon))
                    time.sleep(1)
                    
                self.measure_now
                    
                # Average measurements
                average_lat = sum([m.latitude for m in measurements]) / len(measurements)
                average_lon = sum([m.longitude for m in measurements]) / len(measurements)
                
                sv_str = "Average lat: {} lon: {}".format(average_lat, average_lon)
                rospy.loginfo(sv_str)
                
                # New NavSatFix message
                navsatfix = NavSatFix()
                navsatfix.latitude = average_lat
                navsatfix.longitude = average_lon
                
                # Publish transform
                self.publisher_transform(navsatfix)
                self.measure_now = False
                    

            self.rate.sleep()
        

if __name__ == "__main__":

    rospy.init_node('gps_node', anonymous=False) #True)

    check_server = MeasureBuoyPositions()
    check_server.loop()

    rospy.spin()