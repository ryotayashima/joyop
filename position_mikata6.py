#/usr/bin/python
from core_tool import *
import rospy
from sensor_msgs.msg import Joy

def Help():
  return '''Position contorl script for mikata6 arm.
  Add argument 'position' or 'rotation'.
  Usage: joyop.position'''

def Run(ct, *args):

  q = ct.robot.Q()
  def move(msg):
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
      # ct.robot.MoveToQ(q, 0.1, blocking=True)
    # else:
    #   pass

  sub = rospy.Subscriber('joy', Joy, move)
  rospy.spin()