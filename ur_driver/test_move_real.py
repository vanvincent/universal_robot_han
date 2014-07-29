#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import math
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
client = None
invp=tuple()
invv=tuple()
invp2=tuple()
invp_old=(0,0,0,0,0,0)
initialization = 1;

#give 7 seconds for right arm to initialize itself
def callback3(data):

    global initialization
    
    if initialization==0:
        return
        
    global invp
    global invv
    global invp_old
    
    invp=data.position
    invv=data.velocity
    #mirror joint state
    invp = list(invp)
    invp=[-invp[0],-invp[1]-math.pi,-invp[2],-invp[3]-math.pi,-invp[4],-invp[5]]
    invp = tuple(invp)
    #mirror velocity
    invv = list(invv)
    invv=[-invv[0],-invv[1],-invv[2],-invv[3],-invv[4],-invv[5]]
    invv = tuple(invv)
    #prepare goal
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions=invp,velocities=[0]*6, time_from_start=rospy.Duration(5.0))]
    
    client.send_goal(g)
    print "Send goal:", g
    try:
        client.wait_for_result()
        print "Waiting..."
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    print "Initialization done"
    #initialization=0;
    print "sleep"
    rospy.sleep(5.)
    print "sleep finish"
    
def callback(data):

    global initialization
    if initialization:
        return
        
    global invp
    global invv
    global invp_old
    
    invp=data.position
    invv=data.velocity
    
    invp = list(invp)
    invp=[-invp[0],-invp[1]-math.pi,-invp[2],-invp[3]-math.pi,-invp[4],-invp[5]]
    invp = tuple(invp)
    
    invv = list(invv)
    invv=[-invv[0],-invv[1],-invv[2],-invv[3],-invv[4],-invv[5]]
    invv = tuple(invv)
    
    #self deviation and left arm deviation; for later use 
    #dev=math.sqrt((invp[0] - invp2[0])**2 + (invp[1] - invp2[1])**2+ (invp[2] - invp2[2])**2+ (invp[3] - invp2[3])**2+ (invp[4] - invp2[4])**2+ (invp[5] - invp2[5])**2)
    #dev2=math.sqrt((invp[0] - invp_old[0])**2 + (invp[1] - invp_old[1])**2+ (invp[2] - invp_old[2])**2+ (invp[3] - invp_old[3])**2+ (invp[4] - invp_old[4])**2+ (invp[5] - invp_old[5])**2)
    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions=invp,velocities=invv,time_from_start=rospy.Duration(1.))]
    
    #client.send_goal(g)
    print "Send goal:", g.trajectory.points
    invp_old=invp
    rospy.sleep(1.)
    
    
    
        
def callback2(data):

    global initialization
    if initialization:
        return
        
    global invp2
    invp2=data.position
    




def listener():

    global initialization;
    global client   
    initialization = 1
    
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('/ur5_r/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        #initialization
        rospy.Subscriber("/joint_states", JointState, callback3)
        #rospy.Subscriber("/joint_states", JointState, callback)
        rospy.Subscriber("/joint_states", JointState, callback2)
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise   
    rospy.spin()
if __name__ == '__main__': listener()
