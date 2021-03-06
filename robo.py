#
#
# Developed at Tisfoon by Amir Pirzadeh
#
import serial
import time
import threading
import os
import math
import tkinter
import numpy as np
import pickle
import random

import sys
sys.path.insert(0, "../myUtils")
from _tcp import TCPServer


rIdle             = 'idle'
rClearFeedback    = 'clear feedback'
rWaitCF           = 'wait clear feedback'
rCloseLoop        = 'close loop'

#simulation data
sim_data = np.zeros((0, 0), dtype=np.int)

#world representation
world_size = 500
world_res  = 20    #world_res * world_size is diameter of travel in mm
half_world = world_size // 2
grid_world = np.zeros((world_size, world_size), dtype=np.uint8)
robo_draw_info = 1
robo_draw_color = 'black'

robo_explore = False
explore_actions = []
robo_state = 'idle'
#these two are calculated by the firmware
robo_travel = 0
robo_angle = 0
#these are calculated from the encoder feedback
robo_vector_pol = [0, 0]
robo_vector_xy = [0, 0]
robo_orientation = 0
robo_theta = 0
last_PLeft = 0
last_PRight = 0
old_pleft = 0
old_pright = 0
last_left_enc = 0
last_right_enc = 0
PLeft = 0
PRight = 0

key = ''
keysdown = {}
init_feedback = 0
check_something_wrong = False
robo_stream_enabled = False
robo_stream_mode = True
robo_read_data = False
data_request_time = 0
update_info_time = 0
robo_button_press_time = 0
check_stuck_time = 0

pwm_R = 0
pwm_L = 0
pwm_norm = 70
pwm_max = 150
pwm_bias = 0
pwm_last_correction = 0
pwm_correction_time = 0

bumper = [0] * 6
crash = [0] * 2
cliff = [0] * 4
bumper_arc = [None] * 6    #for drawing on canvas
motor_current = [0] * 2

cModeStraight = 'straight'
cModeSpin     = 'spin'
cModeBack     = 'back'
cModeDummy    = 'dummy'
R_L_Offset = 0
R_Target = 0
L_Target = 0
C_Mode = cModeStraight
robo_straight_travel = 0

#
# robot wheel radius rw = 36 mm
# counts per revolution of wheel is CPR = 508.8 (from documentation)
# robot wheel separation is D = 232 mm (radius R = 116 mm)
# 
# polar coordinates travel: PLeft is left wheel counts and PRight is right wheel counts.
# r (travel distance mm) = (PLeft + PRight) / 2   * (2 pi rw) / CPR
# theta (orientation range 0 to 2 pi) = (PRight - PRight) / (D / 2)
#
rw = 36.2
CPR = 508.8
D = 232   #217 inside dim, 231 center dim, 247 outside dim
R = D/2
pi_rw = math.pi * rw
two_pi_rw_div_CPR = 2 * pi_rw / CPR
CPR_div_2_pi_rw = CPR / 2 / pi_rw
two_pi = math.pi * 2
radian_in_pos = two_pi / 100
k_mean = 1.74533e-4   #1e-10
k_scale = 2e-4        #0.22

radian_to_degrees = 180 / math.pi

counts_180 = R * CPR / 2 / rw
print('counts in 180 degrees: ', counts_180)

#alignment_error = 0  #5.7397e-5   #radians per mm.

counts_limit = world_size * world_res * CPR_div_2_pi_rw

# travel in mm
def polar_r(PLeft, PRight):
  # (PLeft + PRight) / 2 * (2 * pi * rw) / CPR
  # (PLeft + PRight) * pi * rw / CPR
  if abs(PLeft) < abs(PRight):
    return PLeft * two_pi_rw_div_CPR
  else:
    return PRight * two_pi_rw_div_CPR
  
#range is 0 to 2pi
#orientation in radians
#
# when the signs of r_travel and l_travel are the same,
# robot will orient around the whole diameter for the 
# difference
#
# when the signs of r_travel and l_travel are different,
# robot will turn about its center for the common time
# and about the whole diameter for the difference.
# 
def polar_theta(PLeft, PRight, last_PLeft, last_PRight):
  global robo_orientation, robo_theta
  
  l_travel = PLeft - last_PLeft
  r_travel = PRight - last_PRight
  
  #how far did one go more than the other?
  #double_angle = r_travel - l_travel
  
  robo_theta = (r_travel - l_travel) / 2 / counts_180 * math.pi
  
  if (r_travel * l_travel) > 0:
    drift = (r_travel + l_travel) * np.random.normal(loc=k_mean, scale=k_scale, size=(1))[0] / 18
    robo_theta += drift
    
    
  #print (l_travel, r_travel, robo_theta)
  
  
  #while robo_theta > two_pi:
  #  robo_theta -= two_pi
  #while robo_theta < 0:
  #  robo_theta += two_pi
  
  #print(PRight - PLeft)  
  robo_orientation += robo_theta  
                                
  while robo_orientation > two_pi:
    robo_orientation -= two_pi
  while robo_orientation < 0:
    robo_orientation += two_pi
      
  
  
def radians_to_deg(theta):
  return theta * 360 / two_pi
  
def polar_to_xy(polar):
  x = polar[0] * math.cos(polar[1])
  y = polar[0] * math.sin(polar[1])
  return (x,y)
  
def xy_to_polar(xy):
  rho = math.sqrt(xy[0]*xy[0] + xy[1]*xy[1])
  if abs(xy[0]) < 1e-5:
    if xy[1] > 0:
      phi = math.pi/2
    else:
      phi = math.pi*3/4
  else:
    phi = math.atan(xy[1]/xy[0])
    
  return (rho, phi)
  
def robo_add_tuple(t1, t2):
  x = t1[0] + t2[0]
  y = t1[1] + t2[1]
  return (x, y)
  
def robo_calc_pos(PLeft, PRight):
  global robo_vector_xy, robo_vector_pol
  global last_PLeft, last_PRight
  global robo_straight_travel
  
  #print('Calc Pos', PLeft, PRight)
  
  #phi = robo_orientation
  
  #update the new position in xy and polar coordinates
  #based on encoder feedback
  #movement since last time
  #rho = polar_r((PLeft - last_PLeft), (PRight - last_PRight))
  polar_theta(PLeft, PRight, last_PLeft, last_PRight)  #calculate new instant angle for next calcs
  
  r_travel = PRight - last_PRight
  l_travel = PLeft - last_PLeft

  this_travel = (r_travel + l_travel) / 2
  
  robo_vector_xy[0] += this_travel * two_pi_rw_div_CPR * math.cos(robo_orientation)
  robo_vector_xy[1] += this_travel * two_pi_rw_div_CPR * math.sin(robo_orientation)
  
  #print(rho, phi, PLeft, PRight, last_PLeft, last_PRight)

  robo_straight_travel += this_travel
  
  #(x, y) = polar_to_xy((rho, phi))   #use the old robo_theta for the calcs.
  
  #print(rho, phi, x, y, robo_vector_xy[0], robo_vector_xy[1])
  
  #add in cartesian
  #robo_vector_xy = robo_add_tuple(robo_vector_xy, (x, y))
  
  #print('vec', robo_vector_xy)
  
  #keep a position in polar coordinates
  robo_vector_pol = xy_to_polar(robo_vector_xy)
  
  last_PLeft = PLeft
  last_PRight = PRight
  
def rdata_check(data):
  global robo_state, data_request_time
  
  if data != 3:
    if check_something_wrong:
      print('------------ SOMETHING WRONG')
      data_request_time = time.time() + 10
      robo_state = rIdle
      robo_explore = False
      robo_close()
      time.sleep(1)
      robo_init()
      time.sleep(1)
      robo_full_control()
      time.sleep(1)
      #robo_drive(0, -1)        
      data_request_time = time.time() + 4
    return False
  else:
    return True
    
def rdata_button_press(data):
  global key, robo_state, robo_explore, robo_button_press_time
  
  if time.time() < robo_button_press_time:
    return
  
  if data & 1 > 0:    #clean button
    if not robo_explore:
      btnExploreClick()
    else:
      btnStopClick()
  #elif data & 2 > 0:  #spot button
  #  key = 's'
  elif data & 4 > 0:  #dock button
    print ('******************  STOP')
    btnStopClick()
  elif data & 64 > 0:    #schedule button
    robo_explore = False
    btnStopClick()
    btnClearClick()
    
  if data > 0:
    robo_button_press_time = time.time() + 1.5
    
def robo_safety():
  global explore_actions, robo_draw_info, robo_draw_color
  global pwm_L, pwm_R
  global R_L_Offset
  
  #check the bumper light strength
  if len(explore_actions) == 0 and robo_explore:
    for i in range(6):
      if bumper[i] > 80:
        robo_draw_info = 2
        robo_draw_color = 'red'
        
        if i in [0,1]:
          R_L_Offset -= 4   # turn right

        elif i in [4,5]:
          R_L_Offset += 4   # turn left

        else:
          explore_actions += [cModeSpin, cModeDummy]
          btnBackClick()
  
  #check the cliff light strength
  for i in range(4):
    if cliff[i] < 1200:
      robo_draw_info = 3
      robo_draw_color = 'lime'
      if len(explore_actions) == 0 and robo_explore:
        pwm_L = -35
        pwm_R = -35
        explore_actions += [cModeSpin, cModeDummy]
        btnBackClick()
        robo_sing()
        
  #check the crash bumps
  for i in range(2):
    if crash[i] > 0:
      robo_draw_info = 2
      robo_draw_color = 'red'
      if len(explore_actions) == 0 and robo_explore:
        explore_actions += [cModeSpin, cModeDummy]
        btnBackClick()
        
  if check_stuck():
    if len(explore_actions) == 0 and robo_explore:
      explore_actions += [cModeSpin, cModeDummy]
      btnBackClick()
      
def init_vars():
  global robo_state, last_left_enc, last_right_enc, PLeft, PRight, robo_vector_xy, robo_vector_pol
  global last_PLeft, last_PRight
  global old_pleft, old_pright
  global R_Target, L_Target, R_L_Offset
  global robo_orientation, robo_theta
  
  PLeft = 0
  PRight = 0
  R_Target = 0
  L_Target = 0
  R_L_Offset = 0
  last_PLeft = 0
  last_PRight = 0
  old_pleft = 0
  old_pright = 0
  robo_vector_xy = [0, 0]
  robo_vector_pol = [0, 0]
  robo_orientation = 0
  robo_theta = 0


def rdata_enc_feedback(data):
  global robo_state, last_left_enc, last_right_enc, PLeft, PRight, robo_vector_xy, robo_vector_pol
  global last_PLeft, last_PRight
  global old_pleft, old_pright
  global R_Target, L_Target, R_L_Offset
  global robo_orientation, robo_theta
  global grid_world, robo_explore, explore_actions
  global sim_data
  
  robo_left_enc = int.from_bytes(data[0:2], byteorder='big', signed=True)
  robo_right_enc = int.from_bytes(data[2:4], byteorder='big', signed=True)
  if robo_state == rWaitCF:
    last_left_enc = robo_left_enc
    last_right_enc = robo_right_enc
    PLeft = 0
    PRight = 0
    R_Target = 0
    L_Target = 0
    R_L_Offset = 0
    last_PLeft = 0
    last_PRight = 0
    old_pleft = 0
    old_pright = 0
    robo_vector_xy = [0, 0]
    robo_vector_pol = [0, 0]
    robo_orientation = 0
    robo_theta = 0
    sim_data = np.zeros((0, 0), dtype=np.int)
    grid_world[::] = 0
    robo_explore = False
    explore_actions = []
    _canvas_map.delete('all')
    print('INITIALIZE', robo_left_enc, robo_right_enc, robo_vector_xy[0], robo_vector_xy[1])
    robo_state = rIdle
  
  change_left = (robo_left_enc - last_left_enc)
  if change_left > 32767:
    change_left -= 65535
  elif change_left < -32768:
    change_left += 65535
  
  change_right = (robo_right_enc - last_right_enc)
  if change_right > 32767:
    change_right -= 65535
  elif change_right < -32768:
    change_right += 65535
    
  PLeft += change_left
  PRight += change_right
  
  last_left_enc = robo_left_enc
  last_right_enc = robo_right_enc
  
  if (PLeft != last_PLeft) or (PRight != last_PRight):
    sim_data = np.append(sim_data, [PLeft, PRight])
  
  robo_calc_pos(PLeft, PRight)

def rdata_bumpers(data):
  global bumper
  
  bumper[0] = int.from_bytes(data[0:2], byteorder='big', signed=False)
  bumper[1] = int.from_bytes(data[2:4], byteorder='big', signed=False)
  bumper[2] = int.from_bytes(data[4:6], byteorder='big', signed=False)
  bumper[3] = int.from_bytes(data[6:8], byteorder='big', signed=False)
  bumper[4] = int.from_bytes(data[8:10], byteorder='big', signed=False)
  bumper[5] = int.from_bytes(data[10:12], byteorder='big', signed=False)
  
def rdata_cliff(data):
  global cliff
  
  cliff[0] = int.from_bytes(data[0:2], byteorder='big', signed=False)
  cliff[1] = int.from_bytes(data[2:4], byteorder='big', signed=False)
  cliff[2] = int.from_bytes(data[4:6], byteorder='big', signed=False)
  cliff[3] = int.from_bytes(data[6:8], byteorder='big', signed=False)
  
def rdata_crash(data):
  global crash
  
  crash[0] = data & 2
  crash[1] = data & 1

def rdata_motor_current(data):
  global motor_current
  
  motor_current[0] = int.from_bytes(data[0:2], byteorder='big', signed=True)
  motor_current[1] = int.from_bytes(data[2:4], byteorder='big', signed=True)
  



def robo_read():
  global thread_run
  
  while thread_run:
    time.sleep(0.1)
    
    if robo_read_data:
      if robo_stream_enabled and robo_stream_mode:
        data = ser.read(8 * 84)
        offset = 0
        for i in range(len(data)):
          if data[i] == 19:
            offset = i
            break
            
      elif not robo_stream_mode:
        data = ser.read(80)
    else:
      continue
      
    #print(len(data), data[0+offset], data[84+offset], data[84+84+offset], offset)
    #every once in a while, the offset is 37... not in documentation...
    if offset > 0:
      garbage = ser.read(offset)
      
    #print(offset, data[504+offset])
      
    if len(data) > (offset + 84 + 84 + 84 + 84 + 84 + 84 + 84):
      if (data[0+offset] == 19) and (data[84+offset] == 19) and (data[168+offset] == 19) and       \
         (data[252+offset] == 19) and (data[336+offset] == 19) and (data[420+offset] == 19) and    \
         (data[504+offset] == 19):
        sub_data = data[0+offset+3:0+offset+83]
        act_on_data(sub_data)
      
        sub_data = data[84+offset+3:84+offset+83]
        act_on_data(sub_data)
      
        sub_data = data[168+offset+3:168+offset+83]
        act_on_data(sub_data)
      
        sub_data = data[252+offset+3:252+offset+83]
        act_on_data(sub_data)
      
        sub_data = data[336+offset+3:336+offset+83]
        act_on_data(sub_data)
      
        sub_data = data[420+offset+3:420+offset+83]
        act_on_data(sub_data)
      
        sub_data = data[504+offset+3:504+offset+83]
        act_on_data(sub_data)
      else:
        continue


def act_on_data(data):       
  global robo_state, robo_travel, robo_angle
  global key
  global data_request_time
  global pwm_R, pwm_L, pwm_norm, pwm_last_correction, pwm_correction_time
  
  #response to packet id 100  
  if len(data) == 80:
    if rdata_check(data[40]):             #packet 35 (oi mode)
      rdata_button_press(data[11])      #packet 18
      rdata_enc_feedback(data[52:56])   #packets 43 and 44
      rdata_bumpers(data[57:69])
      rdata_cliff(data[28:36])
      rdata_crash(data[0])              #packet 7
      rdata_motor_current(data[71:75])
      robo_safety()
      update_info()
      data_request_time = time.time() + 0.088
    else:
      print(data[40])
    
    
  #read the button presses on top of robot
  if len(data) == 1:
    key = rdata_button_press(data[0])
  
  #response to packet ID 101.  Read the raw encoder feedback    
  if len(data) == 28:
    rdata_enc_feedback(data[0:4])
  
  #response to packet ID 2  
  if len(data) == 6:
    rdata_travel_angle(data[2:6])
  
  if robo_state == rClearFeedback:
    robo_state = rWaitCF
         
  if robo_state == rCloseLoop:
    if abs(PRight - R_Target) < 100:
      #print('out of close loop')
      robo_pwm(0, 0)
      #robo_drive(0, 0)
      robo_state = rIdle
      pwm_last_correction = 0
    else:       
      
      if time.time() > pwm_correction_time:  

        pwm_correction_time = time.time() + 0.1
        
        error = error_function(PLeft, PRight)  
        correction = (error * 0.5) 
        
        #print(f'{pwm_R:.1f}, {pwm_L:.1f}, {PRight}, {PLeft}, {PRight-PLeft}')
          
        if C_Mode in {cModeStraight, cModeBack}:
          pwm_L = pwm_L + correction - pwm_last_correction
        else:
          pwm_L = pwm_L + correction - pwm_last_correction
          
        pwm_last_correction = correction * 0.95
        
          
        if pwm_L < -pwm_max:
          pwm_L = -pwm_max
        elif pwm_L > pwm_max:
          pwm_L = pwm_max
          
        #print(f'{pwm_R:.0f} {pwm_L:.0f} {radians_to_deg(robo_orientation):.2f} {PRight} {PLeft} {PRight-PLeft}')
      
        robo_pwm(pwm_R, pwm_L + pwm_bias)

      
    


def robo_send(cmds):
  data = bytearray();
  for i in cmds:
    data.extend(chr(i).encode('iso-8859-1'))
    
  ser.write(data)

#make a number out of 2's complement signed with high byte first  
def robo_num(num):
  return [num.to_bytes(2, byteorder='big', signed=True)[0], num.to_bytes(2, byteorder='big', signed=True)[1]]
  
def robo_init():
  #sends op code 128
  robo_send([128])
  
def robo_close():
  global robo_stream_enabled
  
  print('robo/stream closed')
  robo_stream_enabled = False
  robo_send([150, 0, 173])
  
def robo_sing():
  print('sing')
  robo_send([0x8c, 0x03, 0x01, 0x40, 0x10, 0x8d, 0x03, 0x40, 0x10]) 
  
def robo_drive(speed, radius):
  #print(f'drive {speed}mm/sec {radius}mm')
  robo_send([137] + robo_num(speed) + robo_num(radius))
  
def robo_reset():
  print('reset')
  robo_send([7])
  
def robo_full_control():
  print('full control')
  robo_send([132])
  
def robo_request_packet(packet):
  global data_request_time
  
  data_request_time = time.time() + 4
  
  robo_send([142, packet]);
  
def robo_stream(packet):
  global robo_stream_enabled
  
  if not robo_stream_enabled:
    print('enable streaming')
    robo_send([148, 1, packet]);
    robo_stream_enabled = True
  
def error_function(PLeft, PRight):
  if C_Mode in {cModeStraight, cModeBack}:
    return PRight - PLeft - R_L_Offset
  else:
    return -(PRight + PLeft) + R_L_Offset
    
def robo_pwm(R, L):
  R = round(R)
  L = round(L)
  #print(f'pwm {R} {L} {PLeft} {PRight} {error_function(PLeft, PRight)} theta: {radians_to_deg(robo_orientation):.1f}')
  robo_send([146] + robo_num(R) + robo_num(L))


def _keypress(event):
    global keysdown
    keysdown[event.keysym] = 1
    print(keysdown)

def robo_process():
  global robo_state
  global thread_run, init_feedback, key, keysdown, check_something_wrong
  global robo_read_data
  
  data_request_time = time.time() + 3
  
  try:
    while thread_run:
      time.sleep(0.1)
      
      #get robot packets
      if time.time() > data_request_time:
        if robo_stream_mode:
          robo_stream(100)          #streams 80 bytes + 4 bytes of overhead
        else:
          robo_request_packet(100)  #get 80 bytes of info back
          
        robo_read_data = True
           
        if init_feedback > 0:
          init_feedback -= 1
          if init_feedback == 0:
            check_something_wrong = True
          robo_state = rClearFeedback
      
      if 't' in keysdown:   #turn clockwise
        keysdown = {}
        robo_drive(50, -1)
      elif 'g' in keysdown:   #straight
        keysdown = {}
        robo_drive(20, 32767)
      elif 'p' in keysdown or key == 'p':   #stop
        robo_state = rIdle
        robo_drive(0, -1)        
        key = ''
        keysdown = {}
      elif 'w' in keysdown:   #pwm straight
        robo_state = rIdle
        btnGoClick()
        keysdown = {}
      elif 'i' in keysdown:   #show PLeft and PRight
        keysdown = {}
        print(PLeft, PRight)
      elif 'c' in keysdown:
        keysdown = {}
        robo_state = rClearFeedback
      elif 'q' in keysdown:   #quit
        thread_run = False
        keysdown = {}
        time.sleep(0.9)
        _root_window.destroy()
  
  except KeyboardInterrupt:
    pass
    
def reset_check_stuck():
  global old_pleft, old_pright, check_stuck_time
  
  check_stuck_time = time.time() + 2
  
  old_pleft = PLeft
  old_pright = PRight

def check_stuck():
  global old_pleft, old_pright, check_stuck_time
  
  res = False
  if time.time() > check_stuck_time:
    
    if robo_state != rIdle:
      if (abs(old_pleft - PLeft) < 5) and ((pwm_L > 5) or (pwm_L < -5)):
        res = True
        print('left wheel stuck', PLeft, old_pleft, pwm_L)
      if (abs(old_pright - PRight) < 5) and ((pwm_R > 5) or (pwm_R < -5)):
        res = True
        print('right wheel stuck', PRight, old_pright, pwm_R)
        
    old_pleft = PLeft
    old_pright = PRight
    check_stuck_time = time.time() + 2.3

  return res
  
def doOneSim(this_rw, this_CPR, this_D):
  global rw, CPR, D, counts_180, two_pi_rw_div_CPR
  
  init_vars()
  
  D = this_D
  rw = this_rw
  CPR = this_CPR
  
  R = D / 2    
  counts_180 = R * CPR / 2 / rw
  two_pi_rw_div_CPR = 2 * math.pi * rw / CPR
  #print(counts_180, pi_rw_div_CPR)
  
  i = 0
  while i < len(sim_data):
    left_pos = sim_data[i]
    right_pos = sim_data[i+1]
    i += 2    
    robo_calc_pos(left_pos, right_pos)
    
  return robo_vector_xy[0], robo_vector_xy[1], robo_orientation * radian_to_degrees
    

def doSimulation():
  global sim_data
  
  #open the pickled file  
  try:
    with open('sim_data.bin', 'rb') as fp:
      sim_data = pickle.load(fp)
  except:
      print('simulation data not found')
      exit()
  
  print('simulation...')    
  
  best_diff = 0
  best_first = True
  best_x = 0
  best_y = 0
  best_o = 0
  
  trials = 50
  while (trials > 0):
    try_rw = 36.2  #random.uniform(35, 36)
    try_D = 231  #4 * try_rw * random.uniform(790, 805) / CPR  #random.uniform(215,250)
    try_CPR = 508.8 #random.uniform(500, 517.6)
    
    xx, yy, oo = doOneSim(try_rw, try_CPR, try_D)
    
    #print(f'rw:{try_rw:.1f} D:{try_D:.1f} CPR:{try_CPR:.1f}   x,y,o {xx:.1f} {yy:.1f}    {oo:.1f} \n')
    
    dx = 0#abs(xx - 292)  
    dy = 0#abs(yy - 19)
    do = abs(oo - 320)*5
    if (dx+dy+do < best_diff) or best_first:
      best_diff = dx+dy+do
      best_rw = try_rw
      best_D = try_D
      best_CPR = try_CPR
      best_first = False
      best_x = xx
      best_y = yy
      best_o = oo
      print(f'best {best_rw:.3f} {best_D:.3f} {best_CPR:.1f}, {best_x:.1f}, {best_y:.1f}, {best_o:.1f}  {counts_180:.2f}')  
      
    trials -= 1
  
  print(f'best {best_rw:.3f} {best_D:.3f} {best_CPR:.1f}, {best_x:.1f}, {best_y:.1f}, {best_o:.1f}')  


  
try:
  print(os.name)
  if os.name == 'posix':
    ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=3, rtscts=0, xonxoff=0)
  else:
    ser = serial.Serial(port="COM37", baudrate=115200, timeout=3, rtscts=0, xonxoff=0)
  time.sleep(0.5)
  ser_port = True
  print('serial port open okay')
  
except:
  ser_port = False
  print('Robo not connected')
  doSimulation()

def formatColor(r, g, b):
    return '#%02x%02x%02x' % (int(r * 255), int(g * 255), int(b * 255))
    
def draw_robo():
  global grid_world, explore_actions
  global robo_draw_color, robo_draw_info
  global old_pleft, old_pright
  global robo_straight_travel
  
  map_x = (robo_vector_xy[0] // world_res) + half_world
  map_y = (-robo_vector_xy[1] // world_res) + half_world
  
  if robo_explore:
    #if map_x > world_size and len(explore_actions) == 0:
    #  explore_actions += [cModeSpin, cModeDummy]
    #  btnBackClick()

    #if map_x < 0 and len(explore_actions) == 0:
    #  explore_actions += [cModeSpin, cModeDummy]
    #  btnBackClick()
  
    #if map_y > world_size and len(explore_actions) == 0:
    #  explore_actions += [cModeSpin, cModeDummy]
    #  btnBackClick()

    #if map_y < 0 and len(explore_actions) == 0:
    #  explore_actions += [cModeSpin, cModeDummy]
    #  btnBackClick()
      
    if len(explore_actions) > 0:
      if robo_state == rIdle:
        if explore_actions[0] == cModeBack:
          explore_actions.pop(0)
          btnBackClick()
        elif explore_actions[0] == cModeSpin:
          explore_actions.pop(0)
          btnSpinClick()
        elif explore_actions[0] == cModeDummy:
          explore_actions.pop(0)
          robo_straight_travel = 0 
    elif robo_state == rIdle:
      btnExploreClick()
  
  map_x = max(min(map_x, world_size), 0)
  map_y = max(min(map_y, world_size), 0)
  
  deg = robo_orientation * radian_to_degrees
  
  for i in range(6):
    if bumper_arc[i] is not None:
      _canvas_map.delete(bumper_arc[i])
  
  #print(map_x, map_y, deg)
  try:
        
    if grid_world[int(map_x)][int(map_y)] == 0:    
      grid_world[int(map_x)][int(map_y)] = robo_draw_info
      _canvas_map.create_rectangle(map_x, map_y, map_x+1, map_y+1, outline = robo_draw_color)
      
    robo_draw_info = 1
    robo_draw_color = 'black'
  except:
    print('robo off canvas')
    
  
  size = 40
  for i in range(6):
    if bumper[i] > 50:
      bumper_arc[5-i] = _canvas_map.create_arc(map_x-size, map_y-size, map_x+size, map_y+size, start=deg-30+((5-i)*10), extent=12, outline="red", style=tkinter.ARC, width=2)
    else:
      bumper_arc[5-i] = _canvas_map.create_arc(map_x-size, map_y-size, map_x+size, map_y+size, start=deg-30+((5-i)*10), extent=12, outline="black", style=tkinter.ARC, width=2)
      
def update_info():
  global update_info_time
  
  if time.time() > update_info_time:
    info_status.set(f'{robo_state}  {C_Mode}  {explore_actions}')
    info_position.set(f'R: {PRight}->{R_Target:.0f}  L: {PLeft}  Diff: {PRight - PLeft}')
    info_pwm.set(f'pwm {pwm_R:.0f} {pwm_L:.0f} {error_function(PLeft, PRight)} R_L_Offset: {R_L_Offset}')
    info_xy.set(f'x,y  t,o {robo_vector_xy[0]:.1f} {robo_vector_xy[1]:.1f}    {robo_theta*radian_to_degrees:.1f} {robo_orientation*radian_to_degrees:.1f} \n')
    info_bumper.set(f' bumper 0 {bumper[0]} \n bumper 1 {bumper[1]} \n bumper 2 {bumper[2]} \n bumper 3 {bumper[3]} \n bumper 4 {bumper[4]} \n bumper 5 {bumper[5]}')
    info_current.set(f' motor current 0 {motor_current[0]} \n motor current 1 {motor_current[1]} ')
    info_cliff.set(f' cliff 0 {cliff[0]} \n cliff 1 {cliff[1]} \n cliff 2 {cliff[2]} \n cliff 3 {cliff[3]} ')

           
    draw_robo()
      
    update_info_time = time.time() + 0.75
  
    #_canvas.update()

def btnStopClick():
  global robo_state, robo_explore, explore_options
  
  robo_state = rIdle
  explore_options = []
  robo_explore = False
  robo_drive(0, -1)        

def btnGoClick():
  global C_Mode, R_L_Offset, R_Target, L_Target, pwm_R, pwm_L, pwm_last_correction
  global robo_state
  
  R_L_Offset = PRight - PLeft - error_function(PLeft, PRight)
  C_Mode = cModeStraight
  R_Target += (CPR * 2)
  L_Target += (CPR * 2)
  
  pwm_R = pwm_norm
  pwm_L = pwm_norm
  pwm_last_correction = 0
  reset_check_stuck()
  robo_state = rCloseLoop

def btnSpinClick():
  global C_Mode, R_L_Offset, R_Target, L_Target, pwm_R, pwm_L, pwm_last_correction
  global robo_state
  
  R_L_Offset = PRight + PLeft + error_function(PLeft, PRight)
  C_Mode = cModeSpin
  r = random.uniform(0.2, 0.8)
  R_Target += counts_180 * r
  L_Target -= counts_180 * r 
  
  pwm_R = pwm_norm
  pwm_L = -pwm_norm
  pwm_last_correction = 0
  reset_check_stuck()
  robo_state = rCloseLoop
  
def btnBackClick():
  global C_Mode, R_L_Offset, R_Target, L_Target, pwm_R, pwm_L, pwm_last_correction
  global robo_state
  
  R_L_Offset = PRight - PLeft - error_function(PLeft, PRight)
  C_Mode = cModeBack
  R_Target = PRight - (CPR * 1)
  L_Target = PLeft - (CPR * 1)
  
  pwm_R = -pwm_norm
  pwm_L = -pwm_norm
  pwm_last_correction = 0
  reset_check_stuck()
  robo_state = rCloseLoop

def btnExploreClick():
  global C_Mode, R_L_Offset, R_Target, L_Target, pwm_R, pwm_L, pwm_last_correction
  global robo_state, robo_explore, explore_actions
  
  R_L_Offset = PRight - PLeft - error_function(PLeft, PRight)
  C_Mode = cModeStraight
  R_Target += counts_limit
  L_Target += counts_limit
  
  pwm_R = pwm_norm
  pwm_L = pwm_norm
  pwm_last_correction = 0
  reset_check_stuck()
  robo_state = rCloseLoop
  explore_actions = []
  robo_explore = True

def btnClearClick():
  global robo_state
  
  robo_state = rClearFeedback
  
def btnSaveDataClick():
  try:
    with open('sim_data.bin', 'wb') as fp:
      pickle.dump(sim_data, fp)
    
    with open('sim_data.txt', 'w') as text_file:
      i = 0
      while i < len(sim_data):
        print(f'{sim_data[i]}, {sim_data[i+1]}', file=text_file)
        i += 2
  
  except:
    print('error saving data ')

def process_server(connection, data):
  global explore_actions
  global R_L_Offset
  global robo_straight_travel

  data = data.decode('iso-8859-1')
  param_list = data.split(",")
  response = ''
  #print(param_list)
  for param in param_list:
    #print(param)
    response += param + ',\0'

    if len(explore_actions) == 0 and robo_explore:
        if param == 'back':
          if robo_straight_travel > CPR:
            explore_actions += [cModeSpin, cModeDummy]
            btnBackClick()
          else:
            print('back ignored', robo_straight_travel)
        elif param == 'right':
            R_L_Offset -= 50
        elif param == 'left':
            R_L_Offset += 50
      

      
  #create a fixed length packet.  This will avoid the timeout
  #on the client side.  Say 40 chars?
  pad_len = 40 - len(response)
  if pad_len > 0:
    for i in range(pad_len):
      response += '\0'
  
  connection.sendall(bytearray(response, 'iso-8859-1'))

  

#main routine
#if connected to robot
#start a thread to read the serial port
#listen to keyboard
if ser_port:
  thread_run = True
  
  server = TCPServer('127.0.0.1', 1029, process_server) 
          
  t_rx = threading.Thread(name='rx_thread', target=robo_read, args=[])
  t_rx.setDaemon(True)
  t_rx.start()

  t_main = threading.Thread(name='main_thread', target=robo_process, args=[])
  t_main.setDaemon(True)
  t_main.start()

  # Create the root window
  _root_window = tkinter.Tk()
  #_root_window.protocol('WM_DELETE_WINDOW', _destroy_window)
  _root_window.geometry('505x900')
  _root_window.title('Tisfoon')
  _root_window.resizable(1, 1)
  _root_window.bind( "<KeyPress>", _keypress )
  #_canvas = tkinter.Canvas(_root_window, width=400, height=400)
  #_canvas.grid(row=0,column=0)
  _frame_left = tkinter.Frame(_root_window, width=300)
  _frame_left.grid(row=0, column=0)

  _frame_right = tkinter.Frame(_root_window, width=200)
  _frame_right.grid(row=0,column=1)
  
  _canvas_map = tkinter.Canvas(_root_window, width=world_size, height=world_size, bg="light gray")
  _canvas_map.grid(row=1, column=0, columnspan=2)

  btnStop = tkinter.Button(_frame_right, text="Stop", command=btnStopClick)
  btnGo = tkinter.Button(_frame_right, text="Go", command=btnGoClick)
  btnSpin = tkinter.Button(_frame_right, text="Spin", command=btnSpinClick)
  btnBack = tkinter.Button(_frame_right, text="Backup", command=btnBackClick)
  btnExplore = tkinter.Button(_frame_right, text="Explore", command=btnExploreClick)
  btnClear = tkinter.Button(_frame_right, text="Clear", command=btnClearClick)
  btnSaveData = tkinter.Button(_frame_right, text="Save Data", command=btnSaveDataClick)
  
  btnStop.grid(row=0, column=0, sticky="we")
  btnGo.grid(row=1, column=0, sticky="we")
  btnSpin.grid(row=2, column=0, sticky="we")
  btnBack.grid(row=3, column=0, sticky="we")
  btnExplore.grid(row=4, column=0, sticky="we")
  btnSaveData.grid(row=5, column=0, sticky="we")
  btnClear.grid(row=6, column=0, sticky="we")
  
  info_status = tkinter.StringVar(_frame_left)
  info_position = tkinter.StringVar(_frame_left)
  info_pwm = tkinter.StringVar(_frame_left)
  info_xy = tkinter.StringVar(_frame_left)
  info_bumper = tkinter.StringVar(_frame_left)
  info_current = tkinter.StringVar(_frame_left)
  info_cliff = tkinter.StringVar(_frame_left)
  
  lblStatus = tkinter.Label(_frame_left, textvariable=info_status, justify="left", width=35)
  lblPosition = tkinter.Label(_frame_left, textvariable=info_position, justify="left", width=35)
  lblPWM = tkinter.Label(_frame_left, textvariable=info_pwm, justify="left", width=35)
  lblXY = tkinter.Label(_frame_left, textvariable=info_xy, justify="left", width=35)
  lblBumper = tkinter.Label(_frame_left, textvariable=info_bumper, justify="left", width=35)
  lblCurrent = tkinter.Label(_frame_left, textvariable=info_current, justify="left", width=35)
  lblCliff = tkinter.Label(_frame_left, textvariable=info_cliff, justify="left", width=35)
  
  lblStatus.grid(row=0, column=0)
  lblPosition.grid(row=1, column=0)
  lblPWM.grid(row=2, column=0)
  lblXY.grid(row=3, column=0)
  lblBumper.grid(row=4, column=0)
  lblCurrent.grid(row=5, column=0)
  lblCliff.grid(row=6, column=0)

  
  #open the OI
  robo_init()
  time.sleep(1)
  robo_full_control()
  robo_request_time = time.time() + 3
  init_feedback = 3
  
  
  _root_window.mainloop()
  
  thread_run = False
  
  time.sleep(0.3)
  
  #close the OI
  robo_close()

  ser.flush()
  ser.close()


  
