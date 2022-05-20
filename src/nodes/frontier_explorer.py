#!/usr/bin/env python

import math
import time
import cv2
import numpy as np
import rospy
import copy
from lab2 import Lab2
from path_planner import PathPlanner
from rbe_3002_final.msg import keypoint, keypoint_map
from std_msgs.msg import Bool
from nav_msgs.srv import GetMap, GetMapResponse
from nav_msgs.msg import OccupancyGrid, GridCells
from priority_queue import PriorityQueue
from geometry_msgs.msg import Point, Pose, PoseStamped

class Fronteir_Explorer:

    def __init__(self):
        rospy.init_node("frontier_explorer")
        rospy.Rate(10.0)
        #Tells ROS that this node subsribes to all new paths
        self.MapFound = rospy.Publisher("/map_found", Bool, queue_size=1)
        #Publishes the cspace to RVIZ
        self.C_spacePublisher = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=1)
        #Will publish the map and keypoints to path planner node to store
        self.Keypoint_map_publisher = rospy.Publisher("/Keypoint_map", keypoint_map, queue_size=1)
        self.unwalkable = {}
        rospy.sleep(3.0)
        rospy.loginfo("Frontier Explorer Node Initalized") 
        
    
    #Placement here because other non primed methods will trigger error.
    def run(self):
        '''
        Puts all of the methods together to parse 
        through the dynamic map, create a fronteir, 
        and create a finalized map. When the map is 
        fully found, there will be a boolean message 
        published to change the states of phases.
        '''
        while(1):
            #Gets the dynamic map
            mapdata = self.get_map()

            #Creates a deep copy of the mapdata for find fronteir
            real_map = copy.deepcopy(mapdata)

            #Creates the cspace map data
            c_space_data = self.calc_cspace(mapdata, 2)

            #Creates the frontier
            keypoints = self.find_Frontier(c_space_data, real_map, True)
            
            if len(keypoints)==0:
                break

            keypoint_simplify = self.keypoint_tuples(keypoints)
            keypoints_to_send = self.create_keypoints(keypoint_simplify)
            self.send_keypoints(c_space_data, keypoints_to_send)
        
        msg = Bool()
        msg.data = True
        self.MapFound.publish(msg)
        rospy.sleep(10)
        rospy.signal_shutdown("The fronteirs have been found")
        #END METHOD
   
    def create_keypoints(self, keypoints):
        """
        Creates a message format for the keypoints 
        to be sent with the keypoint map publisher
        """
        #Initializes the array
        kp = []
        for kp_ in keypoints:
            temp = keypoint()
            temp.x = kp_[0]
            temp.y = kp_[1]
            temp.size = kp_[2]
            kp.append(temp)
        return kp
        #END METHOD

    def send_keypoints(self, mapdata, keypoints):
        '''
        This will act as the publishing force for the 
        several keypoints within the fronteir space
        '''
        map_message = keypoint_map()
        map_message.map = mapdata
        map_message.keypoints = keypoints
        self.Keypoint_map_publisher.publish(map_message)
        #END METHOD

    def keypoint_tuples(self, keypoints):
        '''
        This method converts the keypoints into
        tuples for them to be parsed easier.
        '''
        keypoint_tuple = []

        for kp in keypoints:
            keypoint_tuple.append((int(kp.pt[0]), int(kp.pt[1]), kp.size)) 
            
        return keypoint_tuple
        #END METHOD

    def get_map(self):
        # Blocks access to try/except logic until the service
        # for retriving map: 'dynamic_map' (nav_msgs/GetMap) is listed as "available"
        rospy.wait_for_service('dynamic_map')
        
        while 1:
            try:
                #Creates service linked to dynamic_map of type GetMap
                map_service = rospy.ServiceProxy('dynamic_map', GetMap)
                #Creates return type for the map
                responce = map_service()
                return responce.map
            except rospy.ServiceException as e:
                #If there is an error, this message will be returned with 
                # the type of 'none' as the return type
                rospy.loginfo("Service call failed:  %s" % e)
                #END METHOD
    
    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        rospy.loginfo("Calculating C-Space")

        # Define padded grid list
        padded_grid = []

        # Creates a GridCells message and publish it
        grid = GridCells()
        grid.header.frame_id = "map"
        grid.cells = padded_grid
        grid.cell_width = mapdata.info.resolution
        grid.cell_height = mapdata.info.resolution
        self.C_spacePublisher.publish(grid)

        padded_map = []

        for cell in mapdata.data:
            padded_map.append(cell)

        # Apply kernel to grid and create padded grid
        x = 0
        y = 0
        
        # Cell_num being used to read through entirety of the length of mapdata.data
        for cell_num in range(len(mapdata.data)):
            # cell_num to x,y is utilized to check for if a cell is occupied
            if mapdata.data[cell_num] > 20:  # if a cell is not walkable perform dilation
                for Y in range(-int(padding), 1 + int(padding)):
                    for X in range(-int(padding), 1 + int(padding)):
                        if x+X in range(0, mapdata.info.width) and y+Y in range(0, mapdata.info.width) and PathPlanner.is_cell_walkable(mapdata, x+X, y+Y):
                            # set grid cell at this index to unwalkable
                            padded_map[PathPlanner.grid_to_index(mapdata, x+X, y+Y)] = 100
                            padded_grid.append(PathPlanner.grid_to_world(mapdata, x+X, y+Y))
                            # add all the cells around a blocked cell as long as they are within the grid size
            x += 1
            if x % mapdata.info.width == 0:
                y += 1
                x = 0

        # Create a GridCells message and publishes it
        grid = GridCells()
        grid.header.frame_id = "map"
        grid.cells = padded_grid
        grid.cell_width = mapdata.info.resolution
        grid.cell_height = mapdata.info.resolution
        self.C_spacePublisher.publish(grid)
       
        # Return the C-space with padded map array
        mapdata.data = padded_map
        return mapdata
        #END METHOD
        
    def find_Frontier(self, cspacemap, mapdata, debug):
        """
        This mnethod will generate the Keypoints based on 
        the map data where this method performes a map service call
        :return: keypoints [cv2.keypoints]
        """
        rospy.loginfo("Calculate the frontiers")

        #Definition of the walls and cspace from the dynamic map service from gmapping
        cspace = np.array(cspacemap.data)
        cspace = np.reshape(cspace, (-1, cspacemap.info.width))
        walls = np.array(mapdata.data)
        walls = np.reshape(walls, (-1, mapdata.info.width))
        walls[walls >= 0] = 255
        walls[walls < 0] = 0
        cspace[cspace > 0] = 255
        walls = walls.astype(np.uint8)
        cspace = cspace.astype(np.uint8)


        kernel = np.ones((2, 2), np.uint8)
        kernel_erode = np.ones((1, 1), np.uint8)
        #Gets the edges between the know and unknown
        evidence_grid = cv2.Canny(walls, 99, 100)
        #Reshapes map to be more smooth w/o noise 2 times 
        evidence_grid = cv2.erode(evidence_grid, kernel_erode)
        evidence_grid = cv2.erode(evidence_grid, kernel_erode)

        evidence_grid = cv2.dilate(evidence_grid, kernel, iterations=2)
        #Subtracts the cspace from the image of the current map
        subtracted = np.subtract(evidence_grid, cspace)
        #Resmooths w/o noise the map for 5 iterations
        combined = cv2.dilate(subtracted, kernel, iterations=5)
        
        #Initializes the blob detector used by cv2
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 200
        params.filterByColor = False
        params.filterByArea = False
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False

        detector = cv2.SimpleBlobDetector_create(params)
        detector.empty()
        keypoint = detector.detect(combined)

        if debug:
            on_map_frontier = cv2.drawKeypoints(evidence_grid, keypoint, np.array([]), (0, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv2.imshow("Blob Detection on the Frontier", on_map_frontier)
            cv2.imshow("combined", combined)
            cv2.imshow("subtracted", subtracted)
            cv2.imshow("evidince grid", evidence_grid)
            cv2.imshow("walls", walls)
            cv2.waitKey(5000)

        rospy.loginfo("Found the frontiers")

        return keypoint
        #END METHOD

if __name__ == '__main__':
   ff = Fronteir_Explorer()
   ff.run()