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


JOINT_NAMES = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q0 = [0.0,0.0,0.0,0,0,0]
Q1 = [-1.57,0.0,0.0,0,0,0]
Q2 = [3.14,0.0,0.0,0,0,0]
Q3 = [0.0,0.0,0.0,0,0,0]
Q = list();
client = None
invp=tuple()
invv=tuple()
invp2=tuple()
invp_old=(0,0,0,0,0,0)

def callback(data):

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
    
    dev=math.sqrt((invp[0] - invp2[0])**2 + (invp[1] - invp2[1])**2+ (invp[2] - invp2[2])**2+ (invp[3] - invp2[3])**2+ (invp[4] - invp2[4])**2+ (invp[5] - invp2[5])**2)
    dev2=math.sqrt((invp[0] - invp_old[0])**2 + (invp[1] - invp_old[1])**2+ (invp[2] - invp_old[2])**2+ (invp[3] - invp_old[3])**2+ (invp[4] - invp_old[4])**2+ (invp[5] - invp_old[5])**2)
    

    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    #g.trajectory.points = [JointTrajectoryPoint(positions=data.position, velocities=data.velocity, time_from_start=rospy.Duration(0.02))]
    g.trajectory.points = [JointTrajectoryPoint(positions=invp,velocities=invv,time_from_start=rospy.Duration(0.02))]
    
    client.send_goal(g)
    print "send goal"
    #print g
    #client.wait_for_result()
    #print "waiting"
    #print g
    #try:
        #client.wait_for_result()
    #except KeyboardInterrupt:
        #client.cancel_goal()
        #raise
    invp_old=invp
        
def callback2(data):

    global invp2
    invp2=data.position


def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions=Q, velocities=[0]*6, time_from_start=rospy.Duration(5.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:   
        client.cancel_goal()
        raise

def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    g.trajectory.points = [
        JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    client.wait_for_result()
    
def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    
    d = 2.0
    g.trajectory.points = []
    for i in range(10):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    
    client.send_goal(g)
    time.sleep(2.0)
    print "Interrupting"
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def listener():

    #rospy.init_node('listener', anonymous=True)

    

    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('/ur5_r/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        rospy.Subscriber("/ur5_l/joint_states", JointState, callback)
        rospy.Subscriber("/ur5_r/joint_states", JointState, callback2)
		#while 1:
            #print 'Enter Tra Array:'
            #s = raw_input()
	        #split1 = s.split(" ")
            #for i in split1:
	            #Q.append(float(i))
            #move1()
	        #del Q[0:len(Q)]
        #move_repeated()
        #move_disordered()
        #move_interrupt()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
        
    rospy.spin()
if __name__ == '__main__': listener()
