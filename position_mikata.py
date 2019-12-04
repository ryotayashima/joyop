#!/usr/bin/python
from core_tool import *
import rospy
from std_msgs.msg import Float64

def Help():
    return '''Position contorl script.
    Add argument 'position' or 'rotation'.
    Usage: joyop.position'''

def Run(ct, *args):

    if len(args) == 0:
        return '''Position contorl script.
        Add argument 'position' or 'rotation'.
        Usage: joyop.position'''

    if args[0] == 'position':
        x = ct.robot.FK()
        def moveX(msg):
            x[0] += 0.002*msg.data
            ct.robot.MoveToX(x, 0.1)
        def moveY(msg):
            x[1] += 0.002*msg.data
            ct.robot.MoveToX(x, 0.1)
        def moveZ(msg):
            x[2] += 0.002*msg.data
            ct.robot.MoveToX(x, 0.1)
        # def move(joy_msg):
        #     x[0] += -0.002*joy_msg.axes[0]
        #     x[1] += 0.002*joy_msg.axes[1]
        #     x[2] += 0.002*joy_msg.axes[4]
        #     ct.robot.MoveToX(x, 0.1)
        #     rospy.loginfo(x)
        
        subX = rospy.Subscriber('keyX', Float64, moveX)
        subY = rospy.Subscriber('keyY', Float64, moveY)
        subZ = rospy.Subscriber('keyZ', Float64, moveZ)
        # joymove = rospy.Subscriber('joy', Float64, move)
        rospy.spin()

    elif args[0] == 'rotation':
        q = ct.robot.Q()
        def rotation1(msg):
            q[0] += -0.03*msg.data
            ct.robot.MoveToQ(q, 0.1)
        def rotation2(msg):
            q[1] += -0.03*msg.data
            ct.robot.MoveToQ(q, 0.1)
        def rotation3(msg):
            q[2] += -0.03*msg.data
            ct.robot.MoveToQ(q, 0.1)
        def rotation4(msg):
            q[3] += 0.03*msg.data
            ct.robot.MoveToQ(q, 0.1)

        subQ1 = rospy.Subscriber('rot1', Float64, rotation1)
        subQ2 = rospy.Subscriber('rot2', Float64, rotation2)
        subQ3 = rospy.Subscriber('rot3', Float64, rotation3)
        subQ4 = rospy.Subscriber('rot4', Float64, rotation4)
        rospy.spin()
