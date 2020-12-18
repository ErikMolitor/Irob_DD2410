# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy

from numpy import  linalg as LA

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from sensor_msgs.msg import JointState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState

class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1
        rospy.sleep(1)
        # succeed after count is done
        if self.i < self.n:
            return pt.common.Status.RUNNING
        else:
            return pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        # self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")
        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):
        # command to tuck arm if haven't already
        if not self.sent_goal:
            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            rospy.sleep(4)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():
            # than I'm finished!
            self.sent_goal = False
            rospy.Duration(10)
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            self.sent_goal = False
            return pt.common.Status.FAILURE

        # if I'm still trying
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raises the head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")
        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):
        # try if not tried
        if not self.tried:
            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True
            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.tried = False
            rospy.sleep(2)
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            self.tried = False
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


class pickup(pt.behaviour.Behaviour):
    def __init__(self):

        rospy.loginfo("Initialising pickup behaviour.")

        # server
        pickup_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.pick_up_srv = rospy.ServiceProxy(pickup_srv_nm, SetBool)
        rospy.wait_for_service(pickup_srv_nm, timeout=30)

        # execution checker
        self.tried = False

        # become a behaviour
        super(pickup, self).__init__("pickup!")
    
    def update(self):
        # try if not tried
        if not self.tried:
            # command
            self.pick_up_req = self.pick_up_srv()
            self.tried = True
            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.pick_up_req.success:
            rospy.loginfo('Pick cube success')
            rospy.sleep(1)
            self.tried = False
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_up_req.success:
            rospy.loginfo('Pick cube failure')
            self.tried = False
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


class place(pt.behaviour.Behaviour):
    def __init__(self):

        rospy.loginfo("Initialising pickup behaviour.")

        # server
        place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.place_srv = rospy.ServiceProxy(place_srv_nm, SetBool)
        rospy.wait_for_service(place_srv_nm, timeout=30)


        # execution checker
        self.tried = False

        # become a behaviour
        super(place, self).__init__("Place!")

    def update(self):
        # try if not tried
        if not self.tried:
            # command
            self.place_req = self.place_srv()
            self.tried = True
            rospy.sleep(5.0)
            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.place_req.success:
            rospy.loginfo('Place cube success')
            self.tried = False
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_req.success:
            rospy.loginfo('Place cube failure')
            self.tried = False
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


class detect_cb(pt.behaviour.Behaviour):

    def aruco_pose_cb(self, aruco_pose_msg):
        self.aruco_pose_rcv = True
        self.aruco_pose = aruco_pose_msg

    def __init__(self):

        rospy.loginfo("Initialising detect behaviour.")            
        # access to topic
        self.dtct_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        # sub to topic
        self.dtct_sub = rospy.Subscriber(self.dtct_top, PoseStamped, self.aruco_pose_cb, queue_size=1)
        self.aruco_pose_rcv = False
        self.aruco_pose = None

        super(detect_cb, self).__init__("Detect!")

    def update(self):
        now = rospy.get_time()
        rospy.sleep(2)

        if not self.aruco_pose_rcv:
            rospy.loginfo('Aruco pose message not recived')
            return pt.common.Status.FAILURE

        if self.aruco_pose_rcv: 
            if (self.aruco_pose.header.stamp.secs + 1) < now:
                rospy.loginfo("The cube was not detected")
                return pt.common.Status.FAILURE
            else: 
                rospy.loginfo("The cube is detected")
                return pt.common.Status.SUCCESS


class localize(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo("Localizing position")

        # parameter
        self.lclz_srv = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.clrcstmp_srv = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        self.state_estimate_topic = rospy.get_param(rospy.get_name() + '/amcl_estimate')

        # service
        self.localize_srv = rospy.ServiceProxy(self.lclz_srv, Empty)

        #init publisher
        self.cmd_vel_top = "/key_vel"
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        self.startPos = True

        super(localize, self).__init__("Localize!")

    def update(self):
        
        state_estimate = rospy.wait_for_message(self.state_estimate_topic,PoseWithCovarianceStamped)
        euc_norm = LA.norm(state_estimate.pose.covariance)

        # if euc_norm < 0.06 and not self.startPos:
        #     rospy.loginfo('Already located')
        #     self.startPos = False
        #     return pt.common.Status.SUCCESS

        localize_req = self.localize_srv()
        rate = rospy.Rate(10)

        move_msg = Twist()
        move_msg.angular.z = 0.6

        rospy.loginfo("Turning around for localizing")
        euc_norm = 0.1
        state_estimate = rospy.wait_for_message(self.state_estimate_topic,PoseWithCovarianceStamped)
        t_0 = state_estimate.header.stamp.secs
        dt = 1
        while not rospy.is_shutdown() and euc_norm >0.02 and dt < 30:
            self.cmd_vel_pub.publish(move_msg)
            state_estimate = rospy.wait_for_message(self.state_estimate_topic,PoseWithCovarianceStamped)
            euc_norm = LA.norm(state_estimate.pose.covariance)
            t_1 = state_estimate.header.stamp.secs
            dt = t_1 - t_0
            rate.sleep()
        
        if dt < 30:
            rospy.loginfo("Localizing done")
            
            # stop spin 
            move_msg.angular.z = 0
            self.cmd_vel_pub.publish(move_msg)
            
            rospy.loginfo("Clearing costmap")
            clear_costmap_srv = rospy.ServiceProxy(self.clrcstmp_srv, Empty)
            clear_costmap_req = clear_costmap_srv()
            self.startPos = False
            return pt.common.Status.SUCCESS

        else: 
            rospy.loginfo("LOCALIZING TIME EXCEEDED")
            return pt.common.Status.FAILURE


class nav_to_pick(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo('Navigating to pick pose')
        
        # driving action
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)

        # get pose 
        self.pkpose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.state_estimate_topic = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        self.nav_goal_tp = rospy.get_param(rospy.get_name() + '/nav_goal_topic')
        self.feedback_tp = rospy.get_param(rospy.get_name()+ '/move_base_feedback')

        # prepare message
        self.goal = MoveBaseGoal()
        self.sent_goal = False
        self.finished = False

        super(nav_to_pick, self).__init__("Navigate to pick!")


    def update(self):

        if not self.sent_goal:    
            rospy.loginfo('I am sending the goal to pick pose')
            # get pose
            pose = rospy.wait_for_message(self.pkpose_top, PoseStamped)
            # put pose in message
            self.goal.target_pose = pose
            # send message to action
            self.move_base_ac.send_goal(self.goal)
            # sent = true
            self.sent_goal = True
            # tell the tree you're running
            return pt.common.Status.RUNNING
        
        state_estimate = rospy.wait_for_message(self.state_estimate_topic, PoseWithCovarianceStamped)
        euc_norm = LA.norm(state_estimate.pose.covariance)

        if euc_norm > 0.06:
            self.move_base_ac.cancel_goal()
            self.sent_goal = False
            rospy.loginfo("GOAL CANCELED")
            return pt.common.Status.FAILURE

        feedback = rospy.wait_for_message(self.feedback_tp, MoveBaseActionFeedback)
        dt  = feedback.header.stamp.secs - feedback.status.goal_id.stamp.secs

        if dt > 50:
            rospy.loginfo('Time for Navigation exceeded')
            self.move_base_ac.cancel_goal()
            self.sent_goal = False
            return pt.common.Status.FAILURE

        # if success
        if self.move_base_ac.get_result() and not self.move_base_ac.get_result() == None and self.sent_goal:
            rospy.loginfo('I succeded')
            self.sent_goal = False
            # I'm finished!
            return pt.common.Status.SUCCESS


        # if failure
        if not self.move_base_ac.get_result() and not self.move_base_ac.get_result() == None:
            rospy.loginfo('I failed')
            return pt.common.Status.FAILURE


        # if I'm still trying
        else:
            #rospy.loginfo('I am running')
            return pt.common.Status.RUNNING 



class nav_to_place(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.loginfo('Navigating to place pose')
        
        # driving action
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)

        # get pose 
        self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.state_estimate_topic = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        self.nav_goal_tp = rospy.get_param(rospy.get_name() + '/nav_goal_topic')
        self.feedback_tp = rospy.get_param(rospy.get_name()+ '/move_base_feedback')

        # prepare message
        self.goal = MoveBaseGoal()
        self.sent_goal = False
        self.finished = False

        super(nav_to_place, self).__init__("Navigate to place!")

    def update(self):

        if not self.sent_goal:
            rospy.loginfo('I am sending the goal to place')
            # get pose
            pose = rospy.wait_for_message(self.place_pose_top, PoseStamped)
            # put pose in message
            self.goal.target_pose = pose
            # send message to action
            self.move_base_ac.send_goal(self.goal)
            # sent = true
            self.sent_goal = True
            # tell the tree you're running
            rospy.Duration(10)
            return pt.common.Status.RUNNING
        
        state_estimate = rospy.wait_for_message(self.state_estimate_topic, PoseWithCovarianceStamped)
        euc_norm = LA.norm(state_estimate.pose.covariance)

        if euc_norm > 0.06:
            self.move_base_ac.cancel_goal()
            self.sent_goal = False
            rospy.loginfo("GOAL CANCELED")
            return pt.common.Status.RUNNING

        Joint_State = rospy.wait_for_message("/joint_states", JointState)
        left_gripper = Joint_State.position[7]
        right_gripper = Joint_State.position[8]

        if left_gripper < 0.02 and right_gripper < 0.02:
            self.move_base_ac.cancel_goal()
            rospy.loginfo("WE DROPPED THE CUBE")
            self.sent_goal = False
            return pt.common.Status.FAILURE

        feedback = rospy.wait_for_message(self.feedback_tp, MoveBaseActionFeedback)
        dt  = feedback.header.stamp.secs - feedback.status.goal_id.stamp.secs

        if dt > 50:
            rospy.loginfo('Time for Navigation exceeded')
            self.move_base_ac.cancel_goal()
            self.sent_goal = False
            return pt.common.Status.FAILURE


        # if success
        elif self.move_base_ac.get_result() and not self.move_base_ac.get_result() == None and self.sent_goal:
            rospy.loginfo(self.move_base_ac.get_result())
            rospy.loginfo('I succeded')
            self.sent_goal = False
            # I'm finished!

            return pt.common.Status.SUCCESS
 
        # if failure
        elif not self.move_base_ac.get_result() and not self.move_base_ac.get_result() == None:
            rospy.loginfo('I failed')
            return pt.common.Status.FAILURE

        # if I'm still trying
        else:
            #rospy.loginfo('I am running')
            return pt.common.Status.RUNNING
    

class respawnCube(pt.behaviour.Behaviour):
    def __init__(self):

        self.respawn_cb = '/gazebo/set_model_state'
        rospy.wait_for_service(self.respawn_cb, timeout= 30)   

        super(respawnCube, self).__init__("Respawning cube")

    def update(self):

        respawn_cb_srv = rospy.ServiceProxy(self.respawn_cb, SetModelState)
        respawn_msg = ModelState()
        respawn_msg.model_name = "aruco_cube" 
        respawn_msg.pose.position.x = -1.130530
        respawn_msg.pose.position.y = -6.653650
        respawn_msg.pose.position.z = 0.86250
        respawn_msg.pose.orientation.x = 0
        respawn_msg.pose.orientation.y = 0
        respawn_msg.pose.orientation.z = 0
        respawn_msg.pose.orientation.w = 1
        respawn_msg.twist.linear.x = 0
        respawn_msg.twist.linear.y = 0
        respawn_msg.twist.linear.z = 0
        respawn_msg.twist.angular.x = 0
        respawn_msg.twist.angular.y = 0
        respawn_msg.twist.angular.z = 0
        respawn_msg.reference_frame = "map"
        respawn_cb_srv(respawn_msg)
        rospy.loginfo('Respawn success')

        return pt.common.Status.SUCCESS
