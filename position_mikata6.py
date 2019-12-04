#/usr/bin/python
from core_tool import *
import rospy
from sensor_msgs.msg import Joy

def Help():
  return '''Position contorl script for mikata6 arm.
  Add argument 'position' or 'rotation'.
  Usage: joyop.position_mikata6'''

def Run(ct, *args):

  if len(args) == 0:
    return '''Position contorl script.
    Add argument 'position' or 'rotation'.
    Usage: joyop.position'''

  if args[0] == 'position':
    x = ct.robot.FK()
    def move(msg):
      if msg.axes[0] > 0.5:
        x[0] += 0.001*msg.axes[0]
      elif msg.axes[1] > 0.5:
        x[1] += 0.001*msg.axes[1]
      elif msg.axes[4] > 0.5:
        x[2] += 0.001*msg.axes[4]
      ct.robot.MoveToX(x, 0.1)
    sub = rospy.Subscriber('joy', Joy, move)
    rospy.spin()

  elif args[0] == 'rotation':
    q = ct.robot.Q()
    def rotation(msg):
      # rospy.loginfo(msg.axes)
      q = ct.robot.Q()
      if msg.axes[5] > 0.5:
        if abs(msg.axes[0]) > 0.5:
          q = ct.robot.Q()
          q[0] += 0.01*msg.axes[0]
          ct.robot.MoveToQ(q, 0.1, blocking=True)
        elif abs(msg.axes[1]) > 0.5:
          q = ct.robot.Q()
          q[1] += -0.01*msg.axes[1]
          ct.robot.MoveToQ(q, 0.1, blocking=True)
        elif abs(msg.axes[3]) > 0.5:
          q = ct.robot.Q()
          q[2] += 0.01*msg.axes[3]
          ct.robot.MoveToQ(q, 0.1, blocking=True)
        elif abs(msg.axes[4]) > 0.5:
          q = ct.robot.Q()
          q[4] += -0.01*msg.axes[4]
          ct.robot.MoveToQ(q, 0.1, blocking=True)
        ct.robot.MoveToQ(q, 0.1, blocking=True)
      else:
        pass

    sub = rospy.Subscriber('joy', Joy, rotation)
    rospy.spin()