#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanResponse
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import time

class PathController:

    def __init__(self):
        rospy.init_node("path_controller", anonymous=True)
        rospy.Rate(10.0)
        # This will tell ROS that this node publishes a Path message on the "/current_path" topic
        self.PathPublisher = rospy.Publisher("/current_path", Path, queue_size=1)
        
        self.MapPublisher = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
        # Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        # When a message is received, call self.handle_nav_goal
        self.PoseSubscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.handle_nav_goal)
        
        #Subscriber to transition phases if the entire map has been found
        self.MapFound = rospy.Subscriber("/map_found", Bool, self.update_Phase)
       
        #Path planner service 
        self.get_pathService = rospy.Service("next_path", GetPlan, self.get_path)
        
        #Current phase of the bot
        self.phase = 1
        #Initial pose for the get_PathService service
        self.initial_pose = None
        #Goal point/pose state for the get_PathService service
        self.new_nav_goal = False

        rospy.sleep(3.0)
        rospy.loginfo("Path Controller Node Initalized")
        #END METHOD


    def handle_nav_goal(self, msg):
       """
       This will update the 2D nav goal
       :param msg [PoseStamped] the 2D Nav Goal
       """
       #Creates a new target
       self.nav_target = msg
       #There is now a new navigation goal
       self.new_nav_goal = True
       #END METHOD


    def wait_2D_Navigation(self):
        """
        This will make sure the function waits 
        for a 2D navigation goa to become available
        """
        while not self.new_nav_goal:
            rospy.sleep(1)
        return self.nav_target
        #END METHOD


    def update_Phase(self, msg):
       """
       Will be utilized if the map is found in frontier explorer
       :param msg [Bool] T/F if the map has been discovered
       """
       if msg.data:
           self.phase = 2
           #END METHOD

    def get_path(self, msg):
       """
       When the get_pathService call is executed, this 
       will create a case for a path statemachine from 
       path_planner and wait until a path is delivered
       :param msg [GetPlan] Current pose to the goal pose message
       """
       #If the pose is still none then the starting pose will be assigned
       if(self.initial_pose == None):
           self.initial_pose = msg.start
       target = None
       plan = None
       
       #State machine of targeted goals for the turtlebot
       while not target or not plan:
           phase = self.phase
           #First phase target will be posestamped
           if(phase == 1):
              target = PoseStamped()

           elif(phase == 2):
              target = self.initial_pose

           elif(phase == 3):
               if(msg.goal == PoseStamped()):
                   target = self.wait_2D_Navigation()
                   #Resets the primer for the 2D nav goal
                   self.new_nav_goal = False
               else:
                   target = msg.goal

           #Creation of plan to drive bot in phase 1-3 to a targeted destination 
           # whether pose stamped, initial pose, or msg.goal
           plan = self.nav_to_point(msg.start, target, phase)
           if (not plan):
               rospy.sleep(1)

       #After phase 2 is completed, phase 3 will be initiated.
       if(self.phase == 2):
           time.sleep(5)
           self.phase = 3
       
       #Returns the complete set of field values
       return GetPlanResponse(plan.plan)
       #END METHOD

    @staticmethod
    def nav_to_point(current_pose, goal, phase):
        """
        Creates the service proxy for making a new 
        path off of the static map which will be 
        used inside of the path_planner node. Basically, 
        this method is what we used in Lab 3 for deliverind 
        the 2D nav goal in the robot controller from RVIZ.
        """
        rospy.wait_for_service("plan_path")
        if not goal:
            return None
        try:
            plan = rospy.ServiceProxy('plan_path', GetPlan)
            #The current pose, goal, and phase are sent within the plan proxy to compare
            response = plan(current_pose, goal, phase)
            #If already at the goal the length of these poses will be zero so return none
            if len(response.plan.poses) < 1:
                return None
            return response
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" %e)
        #END METHOD

    def run(self):
       """
       Runs the node until Ctrl-C is pressed.
       """
       rospy.spin()
       #END METHOD

if __name__ == "__main__":
    Path_Controller = PathController()
    Path_Controller.run()
    #END METHOD
