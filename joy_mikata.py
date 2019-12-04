#!/usr/bin/python
from core_tool import *
import sensor_msgs.msg

def Help():
  return '''Velocity control using Joystick and Jacobian.
  Usage: j'''

def Callback(state, steps, wsteps, gsteps, data):
  gsteps[0]= 0.0
  steps[:]= [0.0]*3
  wsteps[:]= [0.0]*3
  state[1]= 'no_cmd'
  state[3]= False

  multiplier= (1.0+data.axes[5])*0.5 + (1.0-data.axes[2])*2.0  #RT, LT

  axes= [(ax if abs(ax)>0.15 else 0.0) for ax in data.axes]

  if data.buttons[7] > 0:  #START
    state[0]= 'quit'
    return

  if data.buttons[1] > 0:  #B
    state[1]= 'cmd_B'
   
  if data.buttons[2] > 0:  #X
    state[1]= 'cmd_X'

  if data.buttons[3] > 0:  #Y
    state[1]= 'cmd_Y'

  dpad_btn= [1 if btn>0 else 0 for btn in data.buttons[11:15]]
  if any(dpad_btn):
    if sum(dpad_btn)==1:  #Need to press exactly one d-pad button at the same time
      if data.buttons[11] > 0:  #LEFT
        state[1]= 'cmd_left'
        #return
      if data.buttons[12] > 0:  #RIGHT
        state[1]= 'cmd_right'
        #return
      if data.buttons[13] > 0:  #UP
        state[1]= 'cmd_up'
        #return
      if data.buttons[14] > 0:  #DOWN
        state[1]= 'cmd_down'
        #return

  if data.buttons[0] > 0:  #A
    state[1]= 'grip'
    gsteps[0]= multiplier * axes[0]
    #return

  if data.buttons[5] <= 0:  #not RB
    state[3]= False
  else:
    state[3]= True

  if state[3] and state[1]=='no_cmd':
    if data.buttons[4] <= 0:  #not LB
      state[1]= 'position'
      gsteps[0]= 0.0
      steps[0]= multiplier * axes[1]
      steps[1]= multiplier * axes[0]
      steps[2]= multiplier * axes[4]
      wsteps[0]= 0.0
      wsteps[1]= 0.0
      wsteps[2]= multiplier * axes[3]
    else:  #LB
      state[1]= 'orientation'
      gsteps[0]= 0.0
      steps[0]= 0.0
      steps[1]= 0.0
      steps[2]= multiplier * axes[4]
      wsteps[0]= -multiplier * axes[0]
      wsteps[1]= multiplier * axes[1]
      wsteps[2]= multiplier * axes[3]

def Run(ct,*args):
  if not any((ct.robot.Is('Baxter'),ct.robot.Is('Mikata'),ct.robot.Is('CraneX7'),ct.robot.Is('UR'))):     #,ct.robot.Is('Motoman')
    CPrint(4,'This program works only with Baxter, Mikata, CraneX7, and UR.')
    return

  speed_gain= 1.0
  if ct.robot.Is('UR'):  speed_gain= 0.4

  arm= ct.robot.Arm

  is_dxlg= [ct.robot.EndEff(a) is not None and ct.robot.EndEff(a).Is('DxlGripper') for a in range(ct.robot.NumArms)]
  if any(is_dxlg):
    active_holding= [False]*ct.robot.NumArms

  steps= [0.0, 0.0, 0.0]
  wsteps= [0.0, 0.0, 0.0]
  gsteps= [0.0]
  state= ['run', 'no_cmd', arm, False]  #run/quit, no_cmd/CMD, ARM, ACTIVE_BTN

  gstate_range= [ct.robot.GripperRange(a) for a in range(ct.robot.NumArms)]
  gstate= [ct.robot.GripperPos(a) if ct.robot.EndEff(a).IsInitialized else 0.0 for a in range(ct.robot.NumArms)]
  for a in range(ct.robot.NumArms):
    if ct.robot.EndEff(a).IsInitialized:
      ct.robot.MoveGripper(gstate[a],arm=a)

  ct.AddSub('joy', 'joy', sensor_msgs.msg.Joy, lambda msg: Callback(state, steps, wsteps, gsteps, msg))

  velctrl= [ct.Run('velctrl',a) for a in range(ct.robot.NumArms)]
  suppress_velctrl= False  #Set this True when an external program use the velocity control.

  kbhit= TKBHit()
  try:
    while state[0]=='run':
      if kbhit.IsActive():
        key= kbhit.KBHit()
        if key=='q':
          break;
        elif key is not None:
          state[1]= 'key_'+str(key)
      else:
        break

      if state[1]=='key_h':
        print '''\
Command:
  Joystick:
    RB: Activation
    LB: Mode switch (linear/rotational)
    RT: Decelerator (analog; 1 to 0)
    LT: Accelerator (analog; 1 to 5)
    Start: Quit
    Left-stick (axes 0,1):
      Left/Right: +y/-y (linear mode), -wx/+wx (rotational mode)
      Up/Down:    +x/-x (linear mode), +wy/-wy (rotational mode)
    Right-stick (axes 0,1):
      Left/Right: +wz/-wz
      Up/Down:    +z/-z
    X: Switch arm to LEFT
    B: Switch arm to RIGHT
    Y: Run calib_x
    A: Gripper mode when keep pressing
      Left-stick (axes 0,1):
        Left/Right: open/close
    DPad:
      Left:  Run fv.open arm  (Cancel all tactile-based grasping and Open)
      Right: Run fv.grasp 'on'/'off' arm  (Gentle grasp)
      Up:    Run fv.hold' on'/'off' arm   (Holding = Slip avoidance)
      Down:  Run fv.openif 'on'/'off' arm (Handover)
  Keyboard:
    q: Quit
    h: Show help
    l/r: Switch arm to LEFT/RIGHT
    c: Run calib_x (Calibrate the external RGB-D sensor pose)
    i: Run fv.inhand 'on'/'off' arm  (In-hand manipulation)
    t: Run fv.trackf4 (or fv.trackf2) 'on'/'off' arm (Tai Chi)
    y: Run fv.trackf4 (or fv.trackf2) for BOTH ARM   (Tai Chi)
    o: Run fv.tracko 'on'/'off' arm  (Proximity vision-based tracking)
    p: Run fv.pickup2a 'on'/'off' arm (Slip-based automatic picking up) '''
        state[1]= 'no_cmd'

      elif state[1]=='grip':
        if is_dxlg[arm]:
          if not active_holding[arm]:
            ct.robot.EndEff(arm).StartHolding()
            active_holding[arm]= True
        #gstate[arm]+= 0.0002*gsteps[0]
        gstate[arm]= ct.robot.GripperPos(arm) + 0.005*gsteps[0]
        if gstate[arm]<gstate_range[arm][0]:  gstate[arm]= gstate_range[arm][0]
        if gstate[arm]>gstate_range[arm][1]:  gstate[arm]= gstate_range[arm][1]
        #print rospy.Time.now(),LRToStr(arm),ct.robot.GripperPos(arm),gstate[arm]
        ct.robot.MoveGripper(gstate[arm],max_effort=100.0,speed=100.0,arm=arm)
        if ct.robot.EndEff(arm).Is('BaxterEPG'):
          rospy.sleep(0.01)  #FIXME: Without this, ct.robot.GripperPos(arm) is not updated.

      if not state[1]=='grip':
        if is_dxlg[arm]:
          if active_holding[arm]:
            ct.robot.EndEff(arm).StopHolding()
            active_holding[arm]= False

      if state[3] and state[1] in ('position','orientation'):
        q= ct.robot.Q(arm=arm)
        vx= map(lambda x:speed_gain*0.4*x,steps)+map(lambda x:speed_gain*1.0*x,wsteps)
        if ct.robot.DoF(arm=arm)>=6:
          dq= ToList(la.pinv(ct.robot.J(q,arm=arm))*MCVec(vx))
        else:  #e.g. Mikata Arm
          #We use weighted pseudo inverse of Jacobian.
          #W: weights on pose error.
          if Norm(vx[:3])>=Norm(vx[3:]):
            W= np.diag(6.0*Normalize([1.0,1.0,1.0, 0.01,0.01,0.01]))
          else:
            W= np.diag(6.0*Normalize([1.0,1.0,1.0, 0.01,0.01,0.01]))
          dq= ToList(la.pinv(W*ct.robot.J(q,arm=arm))*W*MCVec(vx))
          #dq= ToList(0.5*(la.pinv(ct.robot.J(q,arm=arm)[:3,])*MCVec(vx[:3])+
                          #la.pinv(ct.robot.J(q,arm=arm))*MCVec(vx)))
      else:
        dq= [0.0]*ct.robot.DoF(arm)
      if not suppress_velctrl:
        #CPrint(1,'keyctrl3:',dq)
        velctrl[arm].Step(dq)

  finally:
    kbhit.Deactivate()
    for a in range(ct.robot.NumArms):
      velctrl[a].Finish()
    for a in range(ct.robot.NumArms):
      if is_dxlg[a]:
        if active_holding[arm]:
          ct.robot.EndEff(arm).StopHolding()
          active_holding[arm]= False
    for thread in filter(lambda th:th[:3]=='vs_', ct.thread_manager.thread_list):
      print 'Turn off:',thread
      ct.thread_manager.Stop(name=thread)

    ct.DelSub('joy')
    print 'Finished'
