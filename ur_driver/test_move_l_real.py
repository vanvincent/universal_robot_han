#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q0 = [0.0,0.0,0.0,0,0,0]
Q1 = [-1.57,0,0,0,0,0]
Q2 = [1.57,-1,1,-1,1,-1]
Q3 = [1.5,-0.2,-1.57,0,0,0]
position1 = [2.7136, -1.0514, 1.368, -1.4945, 2.3242, 0.3496]

position11= [2.5322, -1.7314, 2.0405, -1.3462, 2.2429, 0.564]

position12 = [1.7027, -2.4982, 2.2283, -0.4772, 1.6977, 1.2586]

position13 = [ 1.7071290319668746,-1.052462445776813,  1.086463352839103,-0.8606732617236705, 1.5521975705536804, -0.1511573118099383]

position2 = [1.985245876551879, -0.8032840078752788, 0.7275086122303955, -0.7749373140050988, 1.8143981845360102, -0.36476032641438394]

position3 = [2.0368537871144063,-1.0199954063618017, 1.406140377739991,  -1.2434676663968656, 1.8464052377476499, -0.4006549195327187]

position4 = [1.9988048961314355, -0.8864630354676404,1.4085687821202733,  -2.0680578809159718, 1.517141096070917, -0.4178686867959831]

position5 = [2.063781334334074, -0.9086649631965962, 0.956720503594668, 0.08731215773248469, 2.0243384800694155, -0.02162296086043014]
Q = list();
client = None

def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    #g.trajectory.points = [JointTrajectoryPoint(positions=Q, velocities=[0]*6, time_from_start=rospy.Duration(10.0))]
    g.trajectory.points = [JointTrajectoryPoint(positions=position1, velocities=[0]*6, time_from_start=rospy.Duration(10.0)),JointTrajectoryPoint(positions=position11, velocities=[0]*6, time_from_start=rospy.Duration(15.0)),JointTrajectoryPoint(positions=position12, velocities=[0]*6, time_from_start=rospy.Duration(20.0)),JointTrajectoryPoint(positions=position11, velocities=[0]*6, time_from_start=rospy.Duration(25.0)),JointTrajectoryPoint(positions=position1, velocities=[0]*6,time_from_start=rospy.Duration(30.0))]
    client.send_goal(g)
    print "Sent 1"
    try:
        client.wait_for_result()
        print "Finish goal 1"
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move2():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    #g.trajectory.points = [JointTrajectoryPoint(positions=Q, velocities=[0]*6, time_from_start=rospy.Duration(10.0))]
    g.trajectory.points = [JointTrajectoryPoint(positions=position4, velocities=[0]*6, time_from_start=rospy.Duration(5.0)),JointTrajectoryPoint(positions=position3, velocities=[0]*6, time_from_start=rospy.Duration(10.0)),JointTrajectoryPoint(positions=position5, velocities=[0]*6, time_from_start=rospy.Duration(15.0)),JointTrajectoryPoint(positions=position3, velocities=[0]*6, time_from_start=rospy.Duration(20.0)),JointTrajectoryPoint(positions=position2, velocities=[0]*6, time_from_start=rospy.Duration(25.0)),JointTrajectoryPoint(positions=position13, velocities=[0]*6, time_from_start=rospy.Duration(27.0)),JointTrajectoryPoint(positions=position12, velocities=[0]*6, time_from_start=rospy.Duration(29.0)),JointTrajectoryPoint(positions=position11, velocities=[0]*6, time_from_start=rospy.Duration(31.0))]
    client.send_goal(g)
    print "sent 2"
    try:
        client.wait_for_result()
        print "finish goal 2"
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
        
def move3():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    #g.trajectory.points = [JointTrajectoryPoint(positions=Q, velocities=[0]*6, time_from_start=rospy.Duration(10.0))]
    g.trajectory.points = [JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(15.0))]
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

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        while 1:
            move1()
            #move2()
            #print 'Enter Tra Array:'
            #s = raw_input()
            #split1 = s.split(" ")
            #for i in split1:
	        #    Q.append(float(i))
            #move1()
            #del Q[0:len(Q)]
            #move1()
        #move_repeated()
        #move_disordered()
        #move_interrupt()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
