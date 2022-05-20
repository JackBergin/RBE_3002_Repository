#!/usr/bin/env python2

import rospy
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool

class map_conversion:

    def __init__(self):
        rospy.init_node("map_conversion")
        rospy.Rate(10)
        self.map_service = rospy.Service("static_map", GetMap, self.send_map)
        self.PhaseChanger = rospy.Subscriber("/map_found", Bool, self.change_phase)
        self.phase = 1
        self.static_map = None
        #END METHOD

    def change_phase(self, msg):
        #If the entire map has been found:
        if(msg.data):
            #The phase will change to two with the request still being put through for the dynamic map
            self.phase = 2
            rospy.loginfo("Requesting the final map")
            rospy.wait_for_service('dynamic_map')
            try:
                map_service = rospy.ServiceProxy('dynamic_map', GetMap)
                response = map_service()
                self.static_map = response.map
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: s%" %e)
                #END METHOD

    def send_map(self, foo):
        """
        This will request the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
        """
        if(self.phase == 1):
            rospy.loginfo("Requesting the map for map converter")
            rospy.wait_for_service('dynamic_map')
            try:
                map_service = rospy.ServiceProxy('dynamic_map', GetMap)
                response = map_service()
                return response.map
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: s%" %e)
                return None
        else:
            return self.static_map
            #END METHOD

    def run(self):
        rospy.spin()
        #END METHOD

if __name__ == "__main__":
    MC = map_conversion()
    MC.run()
    #END METHOD
