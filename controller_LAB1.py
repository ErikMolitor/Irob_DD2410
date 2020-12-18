#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot

# Use to transform between frames
tf_buffer = None
listener = None


# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0

#Own variablenames
setpoint = None
max_linear_velocity = 0.5
max_angular_velocity = 1.0


def move(path):
    global control_client, robot_frame_id, pub
    # Call service client with path
    rospy.wait_for_service('get_setpoint') # wait to connect to service (until it's not busy)
	
    response = control_client(path) # send path to collition avoidance node, get response 
    setpoint = response.setpoint    # take setpoint from response 
    new_path = response.new_path    # take new path from response

    while new_path.poses:
    # Transform Setpoint from service client
        setpoint_frame_id = setpoint.header.frame_id    # take frame ID from setpoint/header to use in transform 

        transform = tf_buffer.lookup_transform(robot_frame_id,setpoint_frame_id,rospy.Time(0)) #look up transform function

        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform) # do transform 

    # Create Twist message from the transformed Setpoint
        message = Twist()   #set message type
        message.linear.x = hypot(transformed_setpoint.point.x, transformed_setpoint.point.y)    #find length to new point
        message.angular.z = atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)   #find angle to new point

    # Publish Twist
        #set roof on velocity
        if message.linear.x > max_linear_velocity:
            message.linear.x = max_linear_velocity
        else:
            message.linear.x = message.linear.x

        if message.angular.z > max_angular_velocity:
            message.angular.z = max_angular_velocity
            message.linear.x = 0
        else:
            message.angular.z = message.angular.z 

        # to not publish to fast we use sleep (limit freq of pub)
        
        pub.publish(message)
        rate = rospy.Rate(15)
        rate.sleep()  

    # Call service client again if the returned path is not empty and do stuff again
        if new_path.poses:
            response = control_client(new_path)
            setpoint = response.setpoint
            new_path = response.new_path

    # Send 0 control Twist to stop robot
    message.angular.z = 0
    message.linear.x = 0
    pub.publish(message)
    # Get new path from action server
    get_path()





def get_path():
    global goal_client

    # Get path from action server
    goal_client.wait_for_server() # connect to server 

    goal = irob_assignment_1.msg.GetNextGoalAction()  # create goal to send to action server
	
    goal_client.send_goal(goal) #send goal to action server

    goal_client.wait_for_result()   #wait for result from action server

    result = goal_client.get_result() #get result 

    path = result.path #takes path from result 

    # Call move with path from action server
    move(path) #send path to move 
    

if __name__ == "__main__":
    # Init controller node
    rospy.init_node('controller')

    # Init publisher to command velocity node message type twist (x vel and z ang)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Init simple action client 
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)

    # Init service client
    control_client = rospy.ServiceProxy('get_setpoint', irob_assignment_1.srv.GetSetpoint)

    # Create TF buffer save old frames and transform map to baselink 
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Call get path 
    get_path()	

    # Spin	make the program continiue 
    rospy.spin()
    
