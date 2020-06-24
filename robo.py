import serial
import time
import threading
#import keyboard
import os
import math
import tkinter

rIdle             = 'idle'
rClearFeedback    = 'clear feedback'
rWaitCF           = 'wait clear feedback'
rWaitCE           = 'wait clear encoder'
rForward          = 'forward'
rWaitForward      = 'wait forward'
rFaceBackward     = 'face backward'
rWaitFaceBackward = 'wait face backward'
rGoBack           = 'go back'
rWaitGoBack       = 'wait go back'
rFaceForward      = 'face forward'
rWaitFaceForward  = 'wait face forward'
rCloseLoop        = 'close loop'

robo_state = 'idle'
#these two are calculated by the firmware
robo_travel = 0
robo_angle = 0
#these are calculated from the encoder feedback
robo_vector_pol = [0, 0]
robo_vector_xy = [0, 0]
robo_orientation = 0
last_PLeft = 0
last_PRight = 0
last_left_enc = 0
last_right_enc = 0
PLeft = 0
PRight = 0

key = ''
keysdown = {}
init_feedback = 0
check_something_wrong = False
data_request_time = 0

pwm_R = 0
pwm_L = 0
pwm_norm = 70
pwm_max = 90
pwm_accel = 3
pwm_update_time = 0
pwm_last_error = 0

cModeStraight = 'straight'
cModeSpin     = 'spin'
R_L_Offset = 0
R_Target = 0
C_Mode = cModeStraight

#
# robot wheel radius rw = 36 mm
# counts per revolution of wheel is CPR = 508.8 (from documentation)
# robot wheel separation is D = 232 mm (radius R = 116 mm)
# 
# polar coordinates travel: PLeft is left wheel counts and PRight is right wheel counts.
# r (travel distance mm) = (PLeft + PRight) / 2   * (2 pi rw) / CPR
# theta (orientation range 0 to 2 pi) = (PRight - PRight) / (D / 2)
#
rw = 36
CPR = 508.5
D = 252   #217 inside dim, 232 center dim, 247 outside dim
R = D/2
pi_rw = math.pi * rw
pi_rw_div_CPR = pi_rw / CPR
two_pi = math.pi * 2
radian_in_pos = two_pi / 100


# travel in mm
def polar_r(PLeft, PRight):
  # (PLeft + PRight) / 2 * (2 * pi * rw) / CPR
  # (PLeft + PRight) * pi * rw / CPR
  return (PLeft + PRight) * pi_rw_div_CPR
  
#range is 0 to 2pi
#orientation in radians                
def polar_theta(PLeft, PRight):
  global robo_orientation
  
  # (PRight - PLeft) / R
  theta = (PRight - PLeft) / R / 4
    
  
  while theta > two_pi:
    theta -= two_pi
  while theta < 0:
    theta += two_pi
  
  #print(PRight - PLeft)  
  robo_orientation += theta  
    
  while robo_orientation > two_pi:
    robo_orientation -= two_pi
  while robo_orientation < 0:
    robo_orientation += two_pi
  
  return theta
  
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
  
  #print('Calc Pos', PLeft, PRight)
  
  #update the new position in xy and polar coordinates
  #based on encoder feedback
  #movement since last time
  rho = polar_r((PLeft - last_PLeft), (PRight - last_PRight))
  phi = polar_theta((PLeft - last_PLeft), (PRight - last_PRight))
  
  #print(rho, phi, PLeft, PRight, last_PLeft, last_PRight)
  
  (x, y) = polar_to_xy((rho, robo_orientation))
  
  #print(rho, phi, x, y, robo_vector_xy[0], robo_vector_xy[1])
  
  #add in cartesian
  robo_vector_xy = robo_add_tuple(robo_vector_xy, (x, y))
  
  #print('vec', robo_vector_xy)
  
  #keep a position in polar coordinates
  robo_vector_pol = xy_to_polar(robo_vector_xy)
  
  last_PLeft = PLeft
  last_PRight = PRight
  
def rdata_check(data):
  global robo_state, data_request_time
  
  if data != 3:
    print('------------ SOMETHING WRONG')
    if check_something_wrong:
      data_request_time = time.time() + 10
      robo_state = rIdle
      robo_close()
      time.sleep(1)
      robo_init()
      time.sleep(1)
      robo_full_control()
      time.sleep(1)
      robo_drive(0, -1)        
      data_request_time = time.time() + 4
    return False
  else:
    return True
    
def rdata_button_press(data):
  global key, robo_state
  
  if data & 1 > 0:    #clean button
    key = 'b'
  elif data & 2 > 0:  #spot button
    key = 's'
  elif data & 4 > 0:  #dock button
    key = 'p'
    print ('******************  STOP')
  elif data & 64 > 0:    #schedule button
    robo_state = rClearFeedback

def rdata_enc_feedback(data):
  global robo_state, last_left_enc, last_right_enc, PLeft, PRight, robo_vector_xy, robo_vector_pol
  global last_PLeft, last_PRight
  global robo_orientation
  
  robo_left_enc = int.from_bytes(data[0:2], byteorder='big', signed=True)
  robo_right_enc = int.from_bytes(data[2:4], byteorder='big', signed=True)
  if robo_state == rWaitCE:
    last_left_enc = robo_left_enc
    last_right_enc = robo_right_enc
    PLeft = 0
    PRight = 0
    last_PLeft = 0
    last_PRight = 0
    robo_vector_xy = [0, 0]
    robo_vector_pol = [0, 0]
    robo_orientation = 0
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
  
  robo_calc_pos(PLeft, PRight)

  #print(f'polar r: {polar_r(PLeft, PRight)} deg: {radians_to_deg(polar_theta(PLeft, PRight))}')
  
def rdata_travel_angle(data):
  global robo_state, robo_travel, robo_angle
  
  robo_travel += int.from_bytes(data[0:2], byteorder='big', signed=True)
  robo_angle  += int.from_bytes(data[2:4], byteorder='big', signed=True)

  if robo_state == rWaitCF:
    robo_travel = 0
    robo_angle = 0
    robo_state = rWaitCE
  
  if (robo_state != rIdle)and (robo_state != rCloseLoop):
    #print(f'travel: {robo_travel} polar r: {robo_vector_pol[0]}    angle: {robo_angle} deg: {radians_to_deg(robo_vector_pol[1])}')
    #print(f'travel: {robo_travel:.1f} x: {robo_vector_xy[0]:.1f}    angle: {robo_angle:.1f} theta: {radians_to_deg(robo_orientation):.1f}  y: {robo_vector_xy[1]:.1f}  PLeft: {PLeft}  PRight: {PRight}')
    print(PLeft, PRight)
  elif (robo_state != rCloseLoop):
    print(PLeft, PRight)




def robo_read():
  global thread_run
  global robo_state, robo_travel, robo_angle
  global key
  global data_request_time
  global pwm_R, pwm_L, pwm_update_time, pwm_last_error, pwm_norm
  
  while thread_run:
    time.sleep(0.1)
    data = ser.read(80)
    
    #response to packet id 100  
    if len(data) == 80:
      if rdata_check(data[40]):             #packet 35 (oi mode)
        rdata_button_press(data[11])      #packet 18
        rdata_enc_feedback(data[52:56])   #packets 43 and 44
        rdata_travel_angle(data[12:16])   #packets 19 and 20
        data_request_time = time.time() + 0.1
      else:
        print(data[40])
      
    #if len(data) > 0:
      #print(len(data), data.decode('iso-8859-1'))
      #print(len(data))
      #data_rcv = True
      
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
      
    if robo_state == rForward:
      robo_state = rWaitForward
      robo_drive(50, 32767)
      
    if robo_state == rWaitForward:
      if robo_vector_xy[0] >= 500:
        robo_drive(0, 0)
        robo_state = rFaceBackward
        
    if robo_state == rFaceBackward:
      robo_state = rWaitFaceBackward
      robo_drive(50, 1)
      
    if robo_state == rWaitFaceBackward:
      if abs(robo_orientation - math.pi) <= radian_in_pos:
        robo_drive(0, 0)
        robo_state = rGoBack
      
    if robo_state == rGoBack:
      robo_state = rWaitGoBack
      robo_drive(50, 32767)
    
    if robo_state == rWaitGoBack:
      if robo_vector_xy[0] <= 0:
        robo_drive(0, 0)
        robo_state = rFaceForward
        
    if robo_state == rFaceForward:
      robo_state = rWaitFaceForward
      robo_drive(50, -1)
      
    if robo_state == rWaitFaceForward:
      if abs(robo_orientation - 0) <= radian_in_pos or abs(robo_orientation - two_pi) <= radian_in_pos:
        robo_drive(0, 0)
        robo_state = rIdle
        
    if robo_state == rCloseLoop:
      if PRight >= R_Target:
        print('out of close loop')
        robo_pwm(0, 0)
        #robo_drive(0, 0)
        robo_state = rIdle
      else:
        if abs(PRight - R_Target) < 500:
          pwm_norm = 25
        else:
          pwm_norm = pwm_max * 0.9
          
        if pwm_R < pwm_norm:
          pwm_R = max(35, pwm_R + pwm_accel)
        elif pwm_R > pwm_norm:
          pwm_R = max(35, pwm_R - pwm_accel)
          
        if time.time() > pwm_update_time:
          error = (error_function(PLeft, PRight) * 1.8) - pwm_last_error
          pwm_L = pwm_L + error
          if pwm_L < -pwm_max:
            pwm_L = -pwm_max
          elif pwm_L > pwm_max:
            pwm_L = pwm_max
          pwm_update_time = time.time() + 0.2
          pwm_last_error = error + (pwm_last_error * 0.8)
        
          robo_pwm(pwm_R, pwm_L)

      
    


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
  print('robo closed')
  robo_send([173])
  
def robo_sing():
  print('sing')
  robo_send([0x8c, 0x03, 0x01, 0x40, 0x10, 0x8d, 0x03, 0x40, 0x10]) 
  
def robo_drive(speed, radius):
  print(f'drive {speed}mm/sec {radius}mm')
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
  
def robo_run():
  global robo_state
  
  print('begin run')
  if robo_state == rIdle:
    robo_sing()
    robo_state = rForward
    
def error_function(PLeft, PRight):
  if C_Mode == cModeStraight:
    return PRight - PLeft - R_L_Offset
  else:
    return -(PRight + PLeft) + R_L_Offset
    
def robo_pwm(R, L):
  R = round(R)
  L = round(L)
  print(f'pwm {R} {L} {PLeft} {PRight} {error_function(PLeft, PRight)} theta: {radians_to_deg(robo_orientation):.1f}')
  robo_send([146] + robo_num(R) + robo_num(L))


def _keypress(event):
    global keysdown
    keysdown[event.keysym] = 1
    print(keysdown)

def robo_process():
  global robo_state
  global thread_run, init_feedback, key, keysdown, check_something_wrong
  global C_Mode, R_L_Offset, R_Target, pwm_R, pwm_L

  
  try:
    while thread_run:
      time.sleep(0.1)
      
      #get robot packets
      if time.time() > data_request_time:
        robo_request_packet(100)  #get 80 bytes of info back
        if init_feedback > 0:
          init_feedback -= 1
          if init_feedback == 0:
            check_something_wrong = True
          robo_state = rClearFeedback
      
      if 'b' in keysdown or key == 'b':     #begin the program
        keysdown = {}
        robo_run()
        key = ''
      #elif keyCode == 'r':   #reset
      #  keysdown = {}
      #  robo_reset()
      #elif keyboard.is_pressed('s') or key == 's':   #beep
      #  keysdown = {}
      #  robo_sing()
      #  key = ''
      elif 't' in keysdown:   #turn clockwise
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
        if True or C_Mode == cModeStraight:
          C_Mode = cModeSpin
          R_Target = PRight + (CPR * 1.555)
          R_L_Offset = PRight + PLeft
        else:
          C_Mode = cModeStraight
          R_Target = PRight + (CPR * 15)
          R_L_Offset = PRight - PLeft
        
        pwm_R = 0
        pwm_L = 0
        robo_state = rCloseLoop
        keysdown = {}
      elif 'i' in keysdown:   #show PLeft and PRight
        keysdown = {}
        print(PLeft, PRight)
      elif 'c' in keysdown:
        keysdown = {}
        robo_state = rClearFeedback
      elif 'q' in keysdown:   #quit
        thread_run = False
        ser.flush()
        keysdown = {}
        time.sleep(0.9)
        _root_window.destroy()
  
  except KeyboardInterrupt:
    pass
  
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


#main routine
#if connected to robot
#start a thread to read the serial port
#listen to keyboard
if ser_port:
  thread_run = True
  
  t_rx = threading.Thread(name='rx_thread', target=robo_read, args=[])
  t_rx.setDaemon(True)
  t_rx.start()

  t_main = threading.Thread(name='main_thread', target=robo_process, args=[])
  t_main.setDaemon(True)
  t_main.start()

  # Create the root window
  _root_window = tkinter.Tk()
  #_root_window.protocol('WM_DELETE_WINDOW', _destroy_window)
  _root_window.title('iRobot')
  _root_window.resizable(0, 0)
  _root_window.bind( "<KeyPress>", _keypress )
  
  #open the OI
  robo_init()
  time.sleep(1)
  robo_full_control()
  robo_request_time = time.time() + 3
  init_feedback = 3
  
  _root_window.mainloop()

  #close the OI
  robo_close()

  ser.close()


  
