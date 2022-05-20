#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan, GetMap
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
import math
import time

class Lab2:
  def __init__(self):
      """
      Class constructor
      """
      ### Initialize node, name it 'lab2'
      rospy.init_node('Robot Controller Created', anonymous=True)
      rospy.Rate(10.0)

      ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
      self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
     
      ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
      ### When a message is received, call self.update_odometry
      self.OdometrySubscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.update_odometry)
      self.AMCL_OdometrySubscriber = rospy.Subscriber('/odom', Odometry , self.update_odometry)
      
      ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
      ### When a message is received, call self.handle_a_star
      #self.PoseSubscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped , self.handle_nav_goal)
      self.PathSubscriber = rospy.Subscriber("/current_path", Path, self.handle_path)
      self.PoseCloudSubscriber = rospy.Subscriber("/particlecloud", PoseArray, self.handle_pose_prob)
      self.PathSubscriber = rospy.Subscriber("/new_path", Bool, self.update_path)
      self.PhaseChanger = rospy.Subscriber("/map_found", Bool, self.change_phase)
      self.AMCL_posePublisher = rospy.Publisher("/initial_pose", PoseWithCovarianceStamped, queue_size = 1)
      
      
      ### Turtlebot Parameters
      # pose for x
      self.px = 0 
      # pose for y
      self.py = 0 
      # yaw angle
      self.pth = 0 

      #linear speed limit for turtlebot 
      self.max_Wheel_Speed = 0.22 * 0.5
      self.max_Angular_Speed = self.max_Wheel_Speed*2 / 0.178 * 0.5
      self.cur_path_id = 0

      # done with navigation initialization
      self.done_nav_flag = True

      # initial pose of the bot
      self.inital_pos = PoseStamped()
      self.localized = True

      # Starting phase
      self.phase = 1

      #Initialization of variables to be used in state machine
      self.new_path = False
      self.latest_pos_covariance = None
      self.saved_pose = None
      self.fail_safe = False
      self.last_goal = None

      rospy.loginfo("Robot Controller Node Initalized")
      #END METHOD

  def change_phase(self, msg):
      if(msg.data):
          self.new_path = True
          rospy.sleep(1)
          self.phase = 2
          self.max_Wheel_Speed = 0.11
          self.max_Angular_Speed = self.max_Angular_Speed * 0.7
          rospy.sleep(1)
          #END METHOD

  def send_speed(self, linear_speed, angular_speed):
      """
      Sends the speeds to the motors.
      :param linear_speed  [float] [m/s]   The forward linear speed.
      :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
      """
      #Logic Catch if invalid speed
      if abs(linear_speed) > self.max_Wheel_Speed:
          linear_speed = self.max_Wheel_Speed * (linear_speed/abs(linear_speed))
      if abs(angular_speed) > self.max_Angular_Speed:
            angular_speed = self.max_Angular_Speed * (angular_speed/abs(angular_speed))

      # Creates the twist message
      msg_cmd_vel = Twist()
      # Linear velocity where X axis is holonomic
      msg_cmd_vel.linear.x = linear_speed
      msg_cmd_vel.linear.y = 0.0
      msg_cmd_vel.linear.z = 0.0
      # Angular velocity where z is the axis of rotation
      msg_cmd_vel.angular.x = 0.0
      msg_cmd_vel.angular.y = 0.0
      msg_cmd_vel.angular.z = angular_speed
      # Send command
      self.cmd_vel.publish(msg_cmd_vel)
      #END METHOD
      
  def rotate(self, angle):
        """
         Corrective PID control with the rotation of the Bot
        :param angle         [float] The angle to rotate.
        """
        rospy.loginfo("Started Rotate")
        #General drive parameters
        tolerance = 0.04
        sleep = 0.0250
        #Initial robot pose location
        start_angle = self.pth
        target_angle = start_angle - angle + math.pi*4
        current_angle = self.pth + 4*math.pi
        
        #Angular PID control
        Rp = 3
        Ri = 0.003
        Rd = -0.0000
        
        #Recursive effort for angular correction
        angular_effort = 0
        angular_effort_old = 0

        #Initialization of start and end time as well as the system clock
        last_time = time.time()
        start_time = time.time()
        time.clock()
        
        #Angular error storage for recursion
        angular_error_queue = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        while((abs(target_angle - (current_angle))) > tolerance):
            if(self.new_path):
                break
            time.sleep(sleep)
            #Angular PID control
            #Proportional control on the the robots yaw
            current_angle = self.pth + 4*math.pi
            angular_error = (current_angle - target_angle + math.pi) % (math.pi*2) - math.pi
            angular_error_p = (angular_error if angular_error > - math.pi else angular_error + (math.pi*2))
            
            #Integral control on the robots yaw
            angular_error_queue.pop(0)
            angular_error_queue.append(angular_error_p)
            angular_error_i = sum(angular_error_queue)
            
            #Derivative control on the robots yaw
            angular_error_d = (angular_effort - angular_effort_old) / (time.time() - last_time)
            
            #Official corrective angular effort
            angular_effort_old = angular_effort
            angular_effort = (angular_error_p * Rp) + (angular_error_i * Ri) + (angular_error_d * Rd)
            self.send_speed(0, -angular_effort)
            
            last_time = time.time()

            #Failsafe for rotational failure
            if (time.time() - start_time > 5):
                rospy.loginfo("Rotating Fail-Safe triggered")
                break

        #This will stop the robot from spinning
        self.send_speed(0, 0)
        rospy.loginfo("Done Rotating")
        #END METHOD

  def drive(self, distance, linear_speed):
      #Initial bot position
      init_x = self.px
      init_y = self.py
      init_angle = self.pth
     
      #While logic parameters
      tolerance = 0.08
      run = True

      #This will kickoff the robot driving
      self.send_speed(linear_speed, 0)
     
      while run:
          #Waits until robot is at the correct distance within the  tolerance
          if abs(distance - math.sqrt((self.px - init_x) ** 2 + (self.py - init_y) ** 2)) <= tolerance:
              # end the control loop
              run = False

          else:
              #Angular Proportional correction for drive being off
              angular_error = self.fixed_angle(init_angle, self.pth)
              angular_effort = angular_error * 1
              #Corrective speed
              self.send_speed(linear_speed, angular_effort)
      self.send_speed(0, 0)
      print("Done driving")
      #END METHOD

  def normalize(self, *angle):
    """
    This function will convert all angles into -pi to pi space
    :param  angle  [float] [radians] a list of angles 
    :return angles [float] [radians] a list of angles returned -pi to pi
    """
    # r will convert angle to: 0-2pi 
    r = [math.fmod(math.fmod(a, 2*math.pi) + 2*math.pi, 2*math.pi) 
        for a in angle]
    # resultant will convert from wrap angle form pi to 2pi to -pi to zero
    resultant = tuple([a-2*math.pi if a > math.pi else a for a in r])

    # if len of result is 1 unpack tuple, else nothing
    if len(resultant) == 1:
        return resultant[0]
    else:
        return resultant
    return resultant
    #END METHOD

  def fixed_angle(self, angle_1, angle_2):
    """
    Organizational caller for normalize function
    :param angle_1 [float] [radians] first angle
    :param bngle_2 [float] [radians] second angle
    :return answer [float] [radians] angle_1-angle_2
    """
    # normalized angle_1 and angle_2 will be normalized, 
    # and then subtracted to normalize again
    angle_1, angle_2 = self.normalize(angle_1, angle_2)
    return self.normalize(angle_1-angle_2)
    #END METHOD

  def go_to(self, msg):
      """
      Calls rotate(), drive(), and rotate() to attain a given pose.
      This method is a callback bound to a Subscriber.
      :param msg [PoseStamped] The target pose.
      """
      # Parameters:
      target_x = msg.pose.position.x
      target_y = msg.pose.position.y
      quat_orig = msg.pose.orientation  # (x,y,z,w)
      quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
      (roll, pitch, yaw) = euler_from_quaternion(quat_list)
      
      
      #-------------------------------ROTATE---------------------------------------
      to_target_angle = self.pth-math.atan2(target_y-self.py, target_x-self.px)
      #Rotates the robot from calculated angle above
      self.rotate(to_target_angle)


      #--------------------------------DRIVE---------------------------------------
      #to_target_distance = math.sqrt((target_x-self.px)**2 + (target_y-self.py)**2)
      #Drives the robot to the calculated distance
      #self.drive(to_target_distance, 0.22)
      
      #Desired point to drive to with corrective smoothing methods
      point = msg.pose.position
      self.drive_to_point(point, 0.09)
      
      #-------------------------ROTATE TO FINAL ANGLE------------------------------
      #This is used to calculate the angle between the current yaw and the target yaw
      #to_target_yaw = self.pth - yaw
      #self.rotate(to_target_yaw, 0.2) 
      #self.send_speed(0, 0)
      rospy.loginfo("Done with go_to")
      #END METHOD       

  def update_odometry(self, msg):
      """
      Updates the current pose of the robot.
      This method is a callback bound to a Subscriber.
      :param msg [Odometry] The current odometry information.
      """
      message_type = str(msg._type)

      if ((self.phase != 3 and message_type == "nav_msgs/Odometry") or (self.phase == 3 and message_type == "geometry_msgs/PoseWithCovarianceStamped")):
         if(self.phase != 1):
                print("Pose has been updated using: " + str(message_type))
         #Reads the pose from the msg
         self.px = msg.pose.pose.position.x
         self.py = msg.pose.pose.position.y
         self.saved_pose = msg.pose

         #Converts the quaterniaon into an angle
         quat_orig = msg.pose.pose.orientation
         quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
         (roll, pitch, yaw) = euler_from_quaternion(quat_list)
         self.pth = yaw      

      if(message_type == "geometry_msgs/PoseWithCovarianceStamped"):
         self.latest_pos_covariance = msg.pose.covariance
         #END METHOD

  def drive_to_point(self, point, linear_speed):
        """
        This utilized PID control and smooth driving 
        to go from an angle adjustment to a linear movement.
        :param point       : The target endpoint
        :param linear_speed: The forward linear speed.
        """
        rospy.loginfo("Driving to Point")
        
        #Given tolerance for the drive distance
        tolerance = 0.04
        
        #Cranked up tolerance for phase three
        if(self.phase == 3):
            tolerance = 0.08

        #Set sleep paremeter
        sleep = 0.06
        
        #Primes the start time for the drive 
        #function as well as the clock
        start_time = time.time()
        time.clock()

        #PID variables
        Kp = 4
        Ki = 0.03
        Kd = 0.0001
        Rp = 3
        Ri = 0.001
        Rd = -0.0000

        #Recursive proportional control
        p_error_old = 0

        #Adjustment in angular efforts to stay within the line
        angular_effort = 0
        angular_effort_old = 0
        last_time = 0

        #The error correction for both linear and angular adjustments
        error_queue = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        angular_error_queue = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        #This logic kicks off the drive to point control loop
        while abs(math.sqrt((point.x - self.px) ** 2 + (point.y - self.py) ** 2)) > tolerance:
            #Primer for the timing of the loop
            if(self.new_path):
                break
            current_time = rospy.get_rostime().nsecs/1000000

            #PID calculations:
            #Linear PID
            
            #Proportional error calculation
            p_error = abs(math.sqrt((point.x - self.px) ** 2 + (point.y - self.py) ** 2))
            
            #Integral error calculation using a queue
            error_queue.pop(0)
            error_queue.append(p_error)
            i_error = sum(error_queue)
            
            #Derivative error calculation
            d_error = (p_error - p_error_old) / (current_time - last_time)
            
            #Linear effort to drive the bot
            linear_effort = linear_speed * ((p_error * Kp) + (i_error * Ki) + (d_error * Kd))

            # Angular PID
            #Proportional control for the the robots yaw adjustment
            target_angle = math.atan2((point.y - self.py), (point.x - self.px)) + 4*math.pi
            current_angle = self.pth + 4*math.pi
            angular_error = (current_angle - target_angle + math.pi) % (math.pi*2) - math.pi
            angular_error_p = (angular_error if angular_error > - math.pi else angular_error + (math.pi*2))
            
            #Integral control for the robots yaw adjustment
            angular_error_queue.pop(0)
            angular_error_queue.append(angular_error_p)
            angular_error_i = sum(angular_error_queue)

            #Derivative control on the robots yaw adjustment
            angular_error_d = (angular_effort - angular_effort_old) / (current_time - last_time)
            
            #Angular effort to be utilized in the drive adjustment of the bot
            angular_effort_old = angular_effort
            angular_effort = (angular_error_p * Rp) + (angular_error_i * Ri) + (angular_error_d * Rd)

            #This will send the speeds that have been calculated
            self.send_speed(linear_effort, -angular_effort)
            
            #Pauses for 0.06 seconds and utilizes the time control within the loop
            last_time = current_time
            rospy.sleep(sleep)
            
            #Primes the variables for the next loop
            p_error_old = p_error

            if (time.time() - start_time > 30):
                self.new_path = True
                self.fail_safe = True

        #Once the point has been achieved, this will stop the robot from driving
        self.send_speed(0, 0)
        rospy.loginfo("Done with driving_to_point")
        #END METHOD

  def update_path(self, msg):
      self.new_path = msg.data
      #END METHOD

  def handle_path(self, msg):
        """
        Called by the navigation handler above to drive the 
        robot to the goal or to a waypoint that leads to the goal.
        """        
        #Gets the path from the msg
        path = msg.plan.poses

        #Goes to each point in the path unless 
        #new one required due to blockage or new phase
        for waypoint in path:
            self.go_to(waypoint)
            if (self.new_path):
                rospy.loginfo("Path impeded")
                break
        self.done_nav_flag = True
        if(self.phase == 2):
            self.localized = False
            self.phase = 3
        rospy.loginfo("Done with the path")  
            #END METHOD

  
  def handle_pose_prob(self, msg):
     """
     This will be to handle the probability of the 
     particle cloud generated by the amcl node through 
     calculating the covarience of the x and y point 
     and the tolerance.
     """
     #Initialize variables
     tolerance = 0.25
     sum_x = 0
     sum_y = 0
     sum_dx = 0
     sum_dy = 0
     
     #Calculates the error within the pose list
     if(not self.localized):
        #Finds the average pose
        for i in msg.poses:
            sum_x+=i.position.x
            sum_y+=i.position.y
        
        #Averages of the x and y poses
        average_x = sum_x/len(msg.poses)
        average_y = sum_y/len(msg.poses)
        
        #Creates the sum of the derivatives of the x and y positions
        for j in msg.poses:
            sum_dx = abs(j.position.x - average_x)
            sum_dy = abs(j.position.x - average_y)
        
        #Averages of the dx and dy poses
        average_dx = sum_dx/len(msg.poses)
        average_dy = sum_dy/len(msg.poses)
        
        #Creates a distance between the pose derivative that must fall 
        #within the tolerance or else the bot will become localized
        #through a corrective function
        distance = math.sqrt(average_dx **2 + average_dy **2)
        if(distance < tolerance):
            self.send_speed(0,0)
            self.localized = True
        else:
            self.localize()
     else:
        pass
        #END METHOD

  def localize(self):
      self.send_speed(0, 0.3)
      #END METHOD

  def run(self):
        #Waits for these two services to be created
        rospy.wait_for_service("next_path", timeout=None)
        rospy.wait_for_message("/odom", Odometry)

        #Saves the initial pose
        self.inital_pos.pose.position.x = self.px
        self.inital_pos.pose.position.y = self.py

        while(1):
            if(self.done_nav_flag):
                if(self.localized):
                    try:
                        #Plan the service call
                        plan = rospy.ServiceProxy('next_path', GetPlan)
                        goal = PoseStamped()
                        if(self.fail_safe and self.phase == 3):
                            goal = self.last_goal
                            self.fail_safe = False

                        #Path parameters for position
                        cur_pose = PoseStamped()
                        cur_pose.pose.position.x = self.px
                        cur_pose.pose.position.y = self.py

                        #This will get a path and a response
                        response = plan(cur_pose, goal, 0.15)
                        self.done_nav_flag = False
                        self.new_path = False

                        if(self.phase == 2):
                            new_pose = PoseWithCovarianceStamped()
                            new_pose.pose.covariance = self.latest_pos_covariance
                            new_pose.pose.pose = self.saved_pose.pose
                            new_pose.header.frame_id = "odom"
                            self.AMCL_posePublisher.publish(new_pose)
                            localization = rospy.ServiceProxy('global_localization', Empty)
                            null = localization()

                        self.last_goal = response.plan.poses[len(response.plan.poses)-1]
                        self.handle_path(response)
                    except rospy.ServiceException as e:
                        rospy.loginfo("Service failed: %s" % e)
                else:
                    print("Not Localized!!!")
                    self.localize()
            else:
                print("Nav Flag = false!")
            time.sleep(1)

if __name__ == '__main__':
   robot = Lab2()
   robot.run()
   rospy.spin() 