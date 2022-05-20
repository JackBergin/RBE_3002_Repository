#!/usr/bin/env python

import math
import time
import rospy
import numpy as np
import cv2
from std_msgs.msg import Bool
from rbe_3002_final.msg import keypoint, keypoint_map
from priority_queue import PriorityQueue
from nav_msgs.srv import GetPlan, GetPlanResponse, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

class PathPlanner:

    def __init__(self):
        """
        Class constructor
        """
        # Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        
        # Create a new service called "plan_path" that accepts messages of
        # type GetPlan and calls self.plan_path() when a message is received
        self.path_plan_service = rospy.Service("plan_path", GetPlan, self.plan_path)
        #Recieves the keypoint map from the frontier explorer node.
        self.Keypoint_Receiver = rospy.Subscriber("/Keypoint_map", keypoint_map, self.store_data)
        # Create a publisher for the C-space (the enlarged occupancy grid)
        # The topic is "/path_planner/cspace", the message type is GridCells
        self.C_space_publisher = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=1)
        # Create publishers for A* (expanded cells, frontier, ...)
        # Choose a the topic names, the message type is GridCells
        #This is for the wave front of the astar algorithim in which each square checked it highlighted
        self.A_star_publisher = rospy.Publisher("/path_planner/astar_ckecked", GridCells, queue_size=1)
        #This is the path for astar where the optimized path is shown.
        self.A_star_publisher_2 = rospy.Publisher("/path_planner/astar_path", Path, queue_size=1)
        #Publisher for statemachine that showcases which path we are on in regard to phase
        self.Path_Phase_Publisher = rospy.Publisher("/new_path", Bool, queue_size=1)
        
        #Initializing globals for this class to be utilized in the storage of 
        #maps or transition of states within this node.
        self.stored_map_data = None
        self.stored_key_points = None
        self.new_data = False
        self.most_recent_path = []

        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")
    
    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        #Gives the width of the map data
        width = mapdata.info.width
        #Creates the index from the width and the given (x, y) coordinates
        index = y*width + x
        #Return the index
        return index
        #END METHOD
    
    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        #Final x point
        x = x2 - x1
        #Final y point
        y = y2 - y1
        #Distance to the final point
        distance = math.sqrt(x**2+y**2)
        #Returns the distance
        return distance
        #END METHOD


    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        #Gets the origin of x and y in the real world
        grid_x_origin = mapdata.info.origin.position.x
        grid_y_origin = mapdata.info.origin.position.y

        #Gets the size of each of the cells
        resolution = mapdata.info.resolution
        
        #Scales the x coordinates to the resolution of the cell size
        real_world_x = resolution*(x+0.5) + grid_x_origin
        real_world_y = resolution*(y+0.5) + grid_y_origin

        point = Point(real_world_x, real_world_y, 0)
        
        return point
        #END METHOD

    
    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        #Gets world points
        wp_X = wp.x
        wp_Y = wp.y

        #Gets orgion in real world
        orgin_X = mapdata.info.origin.position.x
        orgin_Y = mapdata.info.origin.position.y

        #Gets the resolution of each cell
        resolution = mapdata.info.resolution

        #Gets the world point distance from the maps world orgin
        x_offest_from_cell_orgin = wp_X - orgin_X
        y_offest_from_cell_orgin = wp_Y - orgin_Y

        #Finds the distance from the cell's edge
        x_remainder = x_offest_from_cell_orgin % resolution
        y_remainder = y_offest_from_cell_orgin % resolution

        #Finds the index of the cell
        x_cell_index = (x_offest_from_cell_orgin - x_remainder) / resolution
        y_cell_index = (y_offest_from_cell_orgin - y_remainder) / resolution
        
        return (int(x_cell_index), int(y_cell_index))
        #END METHOD

    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """

        #Array to store all of the poses in
        poses = []
        #Parsing through the path to attain separate poses
        for pose in path:
            #Creates a point from two poses
            point = PathPlanner.grid_to_world(mapdata, pose[0], pose[1])
            #Pose message
            pose_stamp = PoseStamped()
            #Sets pose message to the position of the point
            pose_stamp.pose.position = point
            #Appends this position to the pose array
            poses.append(pose_stamp)

        return poses
        #END METHOD


    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        #Pixels with occupancy probability less than this threshold are considered completely free.
        free_thresh = 19.6
        
        #If the x and y values are within the actual grid
        if x < mapdata.info.width and x >= 0 and y < mapdata.info.height and y >= 0:
            #Pulling the free_thresh value from the grid_to_index
            cell = PathPlanner.grid_to_index(mapdata, x, y)
            #Debugging testing param:
            temp = mapdata.data[cell]
            #And if the cell is within an unoccupied space
            return (mapdata.data[cell] < free_thresh)
        return False
        #END METHOD

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        #List initialized to hold the coordinates of free cells.
        neighbors_4 = []

        #The following logic checks for the cells next to, above, and below 
        # the current cell and sends these coordinates to the is_cell_walkable 
        # function in order to see if the path in the surrounding four cells 
        # are walkable or not.
        if PathPlanner.is_cell_walkable(mapdata, x - 1, y):
            neighbors_4.append((x-1, y))

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y):
            neighbors_4.append((x+1, y))

        if PathPlanner.is_cell_walkable(mapdata, x, y + 1):
            neighbors_4.append((x, y+1))

        if PathPlanner.is_cell_walkable(mapdata, x, y - 1):
            neighbors_4.append((x, y-1))

        return neighbors_4
        #END METHOD

    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        #List initialized to hold the coordinates of free cells.
        neighbors_8 = []

        #The following logic checks for the cells next to, above, and below 
        # the current cell and sends these coordinates to the is_cell_walkable 
        # function in order to see if the path in the surrounding 8 cells 
        # are walkable or not.

        if PathPlanner.is_cell_walkable(mapdata, x - 1, y + 1):
            neighbors_8.append((x-1, y+1))

        if PathPlanner.is_cell_walkable(mapdata, x, y + 1):
            neighbors_8.append((x, y+1))

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y + 1):
            neighbors_8.append((x+1, y+1))

        if PathPlanner.is_cell_walkable(mapdata, x - 1, y):
            neighbors_8.append((x-1, y))

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y):
            neighbors_8.append((x+1, y))

        if PathPlanner.is_cell_walkable(mapdata, x - 1, y - 1):
            neighbors_8.append((x-1, y-1))

        if PathPlanner.is_cell_walkable(mapdata, x, y - 1):
            neighbors_8.append((x, y-1))

        if PathPlanner.is_cell_walkable(mapdata, x + 1, y - 1):
            neighbors_8.append((x+1, y-1))

        return neighbors_8
        #END METHOD

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        """
        rospy.loginfo("Requesting the map")
        # Blocks access to try/except logic until the service
        # for retriving map: static_map (nav_msgs/GetMap) is listed as "available"
        rospy.wait_for_service('static_map')

        try:
            #Creates service linked to static_map of type GetMap
            static_map_service = rospy.ServiceProxy('static_map', GetMap)
            #Creates return type for the map
            respond = static_map_service()
            return respond.map
        except rospy.ServiceException as e:
            #If there is an error, this message will be returned with 
            # the type of 'none' as the return type
            rospy.loginfo("Service call failed at: %s" % e)
            return None
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
        self.C_space_publisher.publish(grid)

        padded_map = []

        for cell in mapdata.data:
            padded_map.append(cell)

        # Apply kernel to grid and create padded grid
        x = 0
        y = 0
        
        #Cell_num being used to read through entirety of the length of mapdata.data
        for cell_num in range(len(mapdata.data)):
            #If the cell is not walkable, then a dialiation is executed
            if mapdata.data[cell_num] > 20: 
                # Nested for to cover the x and y range of the x,1 line of the padded_map
                # Note: the (1 +) in the positive range is to integrate the current cell into the range
                for Y in range(-int(padding), 1 + int(padding)):
                    for X in range(-int(padding), 1 + int(padding)):
                        # Makes sure not to go over the bounds of the range
                        if x+X in range(0, mapdata.info.width) and y+Y in range(0, mapdata.info.width):
                            # This will set the grid cell at this index to unwalkable
                            padded_map[PathPlanner.grid_to_index(mapdata, x+X, y+Y)] = 100
                            # This will add all the cells around a blocked cell as long as they are within the grid size
                            padded_grid.append(PathPlanner.grid_to_world(mapdata, x+X, y+Y))
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
        self.C_space_publisher.publish(grid)
       
        # Return the C-space with padded map array
        mapdata.data = padded_map
        return mapdata
        #END METHOD

    def a_star(self, mapdata, start, goal):
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
       
        #Creates priority_que obj
        Priority_Queue = PriorityQueue()
        #Puts the starting point of the que at the starting point of the bot
        Priority_Queue.put(start, 0)

        #Initialization of dict modifiers for method
        came_from = {}
        cost = {}
        came_from[start] = 0
        cost[start] = 0
        
        #Timing to exit based on error for how long the bot takes
        start_time = time.time()
        time.clock()
        timeout = 3

        #Initialization of check_grid with the starting node
        checked_grid = [PathPlanner.grid_to_world(mapdata, start[0], start[1])]
        
        #While the que is not empty do this:
        while not Priority_Queue.empty():
            current = Priority_Queue.get()

            #Failsafe if the astar takes too long
            if time.time() - start_time > timeout:
                rospy.loginfo("A-Star Fail-Safe has been triggered")
                return [], GridCells()

            #If we are already at the goal, kick out of the method
            if current == goal:
                break
            
            #If we are not at the goal, then begin the A* Algorithim
            for next in PathPlanner.neighbors_of_8(mapdata, current[0], current[1]):
                #Add 1 because we will be moving by constant cells. 
                #Creates the costs of specific paths
                #Heuristic is the euclidean distance + the turn cost
                new_cost = cost[current] + PathPlanner.euclidean_distance(current[0], current[1], next[0], next[1]) + PathPlanner.turn_Cost(came_from[current],current, next)
                
                if not next in cost or new_cost < cost[next]:
                    cost[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(goal[0], goal[1], next[0], next[1])
                    Priority_Queue.put(next, priority)
                    #This will add the cell to the list of cells visited for rviz
                    checked_grid.append(PathPlanner.grid_to_world(mapdata, next[0], next[1]))
                    came_from[next] = current
        
        path_list = []
        
        #This will backtrack through came_from dict until at start pos, then reverse list
        while not came_from[current] == 0:
            path_list.append(current)
            current = came_from[current]
        path_list.reverse()
        #Adds the goal to the end of the path
        if len(path_list) > 0:
            path_list.append(goal)

        #Creates a GridCells message
        grid = GridCells()
        grid.header.frame_id = "map"
        #Adds the cells that were visited to the message
        grid.cells = checked_grid
        #Adds the cell info to the message
        grid.cell_width = mapdata.info.resolution
        grid.cell_height = mapdata.info.resolution
        #Publishes the message
        self.A_star_publisher.publish(grid)
        print("Done with A*")
        return path_list, grid
        #END METHOD

    @staticmethod
    def turn_Cost(pos0, pos1, pos2):
        """
        This function will add a cost of 0.01 if the robot has to 
        make a turn between the two positions        
        """
        if pos0 == 0:
            return 0
        theta1 = PathPlanner.calc_Angle(pos0, pos1)
        theta2 = PathPlanner.calc_Angle(pos1, pos2)
        return abs(abs(theta1) - abs(theta2)) * 0.025

    @staticmethod
    def calc_Angle(start, end):
        """
        This function will calculate the angle between 2 grid points
        """
        return math.atan2(end[1]-start[1], end[0]-start[0])
    
    @staticmethod
    def no_need_path(start, end, tolerance):
        """
        This function will check to see if the start pose and the end pose 
        are too close so there would be no need in generating a path
        """
        for i in range(start[0]-tolerance, start[0]+tolerance+1):
            for j in range(start[1]-tolerance, start[1]+tolerance+1):
                if end == (i, j):
                    return True
        return False

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        rospy.loginfo("Optimizing path")
        #This variable will create a list to store the optimized path
        optimal_path = [path[0]]
        #This is a variable that stores the slope between the 
        #current point and the point prior
        slope_1 = 0
        #This is a variable that stores the slope between the 
        #current point and the point after
        slope_2 = 0
        
        #Parsing loop for the path given to the method
        for i in range(1, (len(path) - 1)):
            
            #This logic will check the prevous node
            if (path[i][0] - path[i-1][0]) == 0:
                slope_1 = float('inf')
            else:
                slope_1 = (path[i][1] - path[i-1][1]) / (path[i][0] - path[i-1][0])
            
            #This logic will check the next node
            if (path[i+1][0] - path[i][0]) == 0:
                slope_2 = float('inf')
            else:
                slope_2 = (path[i+1][1] - path[i][1]) / (path[i+1][0] - path[i][0])
             
            #If the slopes are not equal then the optimal_path will automatically
            #append the point.
            if slope_1 != slope_2:
                optimal_path.append(path[i])

        optimal_path.append(path[len(path) - 1])
        return optimal_path

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        # This initializes the path variable
        world_path = Path()
        world_path.poses = []
        world_path.header.frame_id = "map"
        # The following for loop will loop through the path
        for index in range(len(path)):
            # Initialization of the current pose
            stamped_pose = PoseStamped()
            # Locates point in the real world
            world_point = self.grid_to_world(mapdata, path[index][0], path[index][1])
            # Adds the real world point to the pose of the robot
            stamped_pose.pose.position = world_point

            # Calculates the finale angle for the pose of the
            # turtle bot based on what the next angle is.
            if(index == (len(path) - 1)):
                # If the last element is hit already, the angle will be kept the same.
                pass
            else:
                # If the last angle has yet to be hit, this will
                # calculate the angle between the last and current point
                angle = math.atan2(path[index+1][1] - path[index][1], path[index+1][0] - path[index][0])
                # Converts the angle to quaternion and proceeds to set it as the angle for the current node
                stamped_pose.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, angle))

            # This will add the pose to the path
            world_path.poses.append(stamped_pose)

        rospy.loginfo("Returning a Path message")
        #Publishes the path to rviz
        self.A_star_publisher_2.publish(world_path)
        #Returns the path
        return world_path
        #END METHOD

    def pick_frontier(self, mapdata, keypoints, start):
        """
        This will select the frontier to navigate to specific "waypoints" by checking 
        the size of the frontier and the size of the path
        """
        max_h = 0
        alpha = 0.9999
        beta = 0.0001

        bestPath = []
        best_Grid = GridCells()

        for point in keypoints:

            size = point[2]
            point = self.check_if_in_cspace(mapdata, point)

            path, grid = self.a_star(mapdata, start, (point[0], point[1]))

            if not PathPlanner.no_need_path(start, point, 5) and len(path) > 0 and max_h < alpha/len(path) + beta * (size/9):
                max_h = alpha/len(path) + beta * (size/9)
                bestPath = path
                best_Grid = grid
        self.A_star_publisher.publish(best_Grid)
        return bestPath

    def check_if_in_cspace(self, cspacedata, start):
        """
        If the bot is in the cspace then it will look at the 
        surrounding spots and their availability so that it 
        can escape an "occupied" zone.
        """
        rospy.loginfo("Changing the start pose due to interference with the cspace")
        if not PathPlanner.is_cell_walkable(cspacedata, start[0], start[1]):
            count = 0
            while 1:
                if PathPlanner.is_cell_walkable(cspacedata, start[0]+count, start[1]):
                    return (start[0]+count, start[1])

                if PathPlanner.is_cell_walkable(cspacedata, start[0]-count, start[1]):
                    return (start[0]-count, start[1])

                if PathPlanner.is_cell_walkable(cspacedata, start[0], start[1]-count):
                    return (start[0], start[1]-count)

                if PathPlanner.is_cell_walkable(cspacedata, start[0], start[1]+count):
                    return (start[0], start[1]+count)

                count += 1
        else:
            return start
            #END METHOD

    def check_path_walkable(self, cspacedata, path):
        """
        Will check if all of the cells in the path are walkable.
        """
        for cell in path:
            if not PathPlanner.is_cell_walkable(cspacedata, cell[0], cell[1]):
                return False
        return True
        #END METHOD

    def store_data(self, msg):
        #New data primer
        self.new_data = True
        #Stores the map data
        self.stored_map_data = msg.map
        #Creates the keypoints
        self.stored_key_points = self.make_keypoints(msg.keypoints)
        #If the path isnt walkable
        if not self.check_path_walkable(msg.map, self.most_recent_path):
            msg = Bool()
            msg.data = True
            self.Path_Phase_Publisher.publish(msg)
            #END METHOD

    def make_keypoints(self, keypoints):
        #Creates a list of all the keypoints within the kp map to be stored
        keypointarr = []
        for keypoint in keypoints:
            keypointarr.append((keypoint.x, keypoint.y, keypoint.size))
        return keypointarr
        #END METHOD

    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        
        #Path showcases the phase
        phase = msg.tolerance
        rospy.loginfo("Planning the path")

        if phase == 1:
            #Checks for the map in the storage
            #In the case of no stored map, a new empty map will be created
            if not self.new_data:
                return Path()

            c_space_data = self.stored_map_data
            keypoints = self.stored_key_points
            #Gets the start pose from the msg being imported to the plan_path
            start = self.check_if_in_cspace(c_space_data, PathPlanner.world_to_grid(c_space_data, msg.start.pose.position))
            #Calculates path through heristic
            path = self.pick_frontier(c_space_data, keypoints, start)
        else:
            #In the case of an error, this will return an empty path:
            mapdata = PathPlanner.request_map()
            if mapdata is None:
                return Path()
            c_space_data = self.calc_cspace(mapdata, 2)
            start = self.check_if_in_cspace(c_space_data, PathPlanner.world_to_grid(c_space_data, msg.start.pose.position))
            #Creates the navigation goal 
            goal = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
             #Calculates the C-space and publish it
            goal = self.check_if_in_cspace(c_space_data, goal)
            #Executes A*
            path, foo = self.a_star(c_space_data, start, goal)

        #Optimizes waypoints
        if path == []:
            return Path()
        self.most_recent_path = path
        waypoints = PathPlanner.optimize_path(path)
        # Return a Path message
        return GetPlanResponse(self.path_to_message(c_space_data, waypoints))
        #END METHOD

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()
        #END METHOD

if __name__ == '__main__':
    Path_Planner = PathPlanner()
    Path_Planner.run()
    #END METHOD