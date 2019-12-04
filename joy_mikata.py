#! /usr/bin/python
from core_tool import *
import sensor_msgs.msg

def Help():
  return '''Velocity contorol using Joystick and Jacobian.
  Usage: joyop.joy'''

def Callback(state, steps, wsteps, gsteps, data):
  gsteps[0] = 0.0
  steps[:] = [0.0]*3
  wsteps[:] = [0.0]*3
  state[1] = 'no_cmd'
  state[3] = False 

  multipler = (1.0+data.axes[5])*0.5 + (1.0-data.axes[2])*2.0 #RT, LT

  axes = [(ax if abs(ax)>0.15 else 0.0) for ax in data.axes]

  if data.buttons[7] > 0:  # START
    state[0] = 'quit'
    return

  if data.buttons[1] > 0:  # B
    state[1] = 'cmd_B'

  if data.buttons[2] > 0:  # X
    state[1] = 'cmd_X'

  if data.buttons[3] > 0:  # Y
    state[1] = 'cmd_Y'

  dpad_btn = [1 if btn>0 else 0 for btn in data.buttons[11:15]] #cross key
  if any(dpad_btn):
    if sum(dpad_btn)==1:
      if data.buttons[11] > 0:  # LEFT
        state[1] = 'cmd_left'
      if data.buttons[12] > 0:  # RIGHT
        state[1] = 'cmd_right'
      if data.buttons[13] > 0:  # UP
        state[1] = 'cmd_up'
      if data.buttons[14] > 0:  # DOWN
        state[1] = 'cmd_down'

  if data.buttons[0] > 0:  # A
    state[1] = 'grip'
    gsteps[0] = multipler * axes[0]

  if data.buttons[5] <= 0:  #  not RB
    state[3] = False
  else:
    state[3] = True

  if state[3] and state[1]=='no_cmd':
    if data.buttons[4] <= 0:  # not LB
      state[1] = 'position'
      gsteps[0] = 0.0
      steps[0] = multipler * axes[1]
      steps[1] = multipler * axes[0]
      steps[2] = multipler * axes[4]
      wsteps[0] = 0.0
      wsteps[1] = 0.0
      wsteps[2] = multipler * axes[3]
    else:  # LB
      state[1] = 'orientation'
      gsteps[0] = 0.0
      steps[0] = 0.0
      steps[1] = 0.0
      steps[2] = multipler * axes[4]
      wsteps[0] = -multipler * axes[0]
      wsteps[1] = multipler * axes[1]
      wsteps[2] = multipler * axes[3]


def Run(ct,*args):
  if not any((ct.robot.Is('Baxter'),ct.robot.Is('Mikata'),ct.robot.Is('CraneX7'),ct.robot.Is('UR'))):     #,ct.robot.Is('Motoman')
    CPrint(4,'This program works only with Baxter, Mikata, CraneX7, and UR.')
    return

  speed_gain = 1.0
  if ct.robot.Is('UR'): speed_gain = 0.4

  arm = ct.robot.Arm

  is_dxlg= [ct.robot.EndEff(a) is not None and ct.robot.EndEff(a).Is('DxlGripper') for a in range(ct.robot.NumArms)]
  if any(is_dxlg):
    active_holding= [False]*ct.robot.NumArms

  steps = [0.0, 0.0, 0.0]
  wsteps = [0.0, 0.0, 0.0]
  gsteps = [0.0]
  state = ['run', 'no_cmd', arm, False] # run/quit, no_cmd/CMD, ARM,ACTIVE_BTN

  gstate_range = [ct.robot.GripperRange(a) for a in range(ct.robot.NumArms)]
  gstate = [ct.robot.GripperPos(a) if ct.robot.EndEff(a).IsInitialized else 0.0 for a in range(ct.robot.NumArms)]
  for a in range(ct.robot.NumArms):
    if ct.robot.EndEff(a).IsInitialized:
      ct.robot.MoveGripper(gstate[a], arm=a)

  ct.AddSub('joy', 'joy', sensor_msgs.msg.Joy, lambda msg: Callback(state, steps, wsteps, gsteps, msg))
