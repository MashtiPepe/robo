import time
import serial
import threading
import keyboard
import os
import math

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

robo_state = 'idle'
#these two are calculated by the firmware
robo_travel = 0
robo_angle = 0
#these are calculated from the encoder feedback
robo_vector_pol = [0, 0]
robo_vector_xy = [0, 0]
robo_orientation = 0
last_P1 = 0
last_P2 = 0
last_left_enc = 0
last_right_enc = 0
P1 = 0
P2 = 0

key = ''
data_request_time = 0
data_rcv = False

#
# robot wheel radius rw = 36 mm
# counts per revolution of wheel is CPR = 508.8 (from documentation)
# robot wheel separation is D = 232 mm (radius R = 116 mm)
# 
# polar coordinates travel: P1 is left wheel counts and P2 is right wheel counts.
# r (travel distance mm) = (P1 + P2) / 2   * (2 pi rw) / CPR
# theta (orientation range 0 to 2 pi) = (p2 - p2) / (D / 2)
#
rw = 36
CPR = 508.8
D = 232
R = D/2
pi_rw = math.pi * rw
pi_rw_div_CPR = pi_rw / CPR
two_pi = math.pi * 2

# travel in mm
def polar_r(P1, P2):
  # (P1 + P2) / 2 * (2 * pi * rw) / CPR
  # (P1 + P2) * pi * rw / CPR
  return (P1 + P2) * pi_rw_div_CPR
  
#range is 0 to 2pi
#orientation in radians                
def polar_theta(P1, P2):
  global robo_orientation
  
  # (P2 - P1) / R
  theta = (P2 - P1) / R / 4
    
  
  while theta > two_pi:
    theta -= two_pi
  while theta < -two_pi:
    theta += two_pi
  
  #print(P2 - P1)  
  robo_orientation += theta  
    
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
  
def robo_calc_pos(P1, P2):
  global robo_vector_xy, robo_vector_pol
  global last_P1, last_P2
  
  #print('Calc Pos', P1, P2)
  
  #update the new position in xy and polar coordinates
  #based on encoder feedback
  #movement since last time
  rho = polar_r((P1 - last_P1), (P2 - last_P2))
  phi = polar_theta((P1 - last_P1), (P2 - last_P2))
  
  #print(rho, phi, P1, P2, last_P1, last_P2)
  
  (x, y) = polar_to_xy((rho, robo_orientation))
  
  #print(rho, phi, x, y, robo_vector_xy[0], robo_vector_xy[1])
  
  #add in cartesian
  robo_vector_xy = robo_add_tuple(robo_vector_xy, (x, y))
  
  #print('vec', robo_vector_xy)
  
  #keep a position in polar coordinates
  robo_vector_pol = xy_to_polar(robo_vector_xy)
  
  last_P1 = P1
  last_P2 = P2
  

def rdata_button_press(data):
  global ket, robo_state
  
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
  global robo_state, last_left_enc, last_right_enc, P1, P2, robo_vector_xy, robo_vector_pol
  global last_P1, last_P2
  global robo_orientation
  
  robo_left_enc = int.from_bytes(data[0:2], byteorder='big', signed=True)
  robo_right_enc = int.from_bytes(data[2:4], byteorder='big', signed=True)
  if robo_state == rWaitCE:
    last_left_enc = robo_left_enc
    last_right_enc = robo_right_enc
    P1 = 0
    P2 = 0
    last_P1 = 0
    last_P2 = 0
    robo_vector_xy = [0, 0]
    robo_vector_pol = [0, 0]
    robo_orientation = 0
    print('INITIALIZE', robo_left_enc, robo_right_enc, robo_vector_xy[0], robo_vector_xy[1])
    robo_state = rIdle
    
  #check for roll over
  if last_left_enc > 10000 and robo_left_enc < -10000:
    p1_add = (32767 - last_left_enc) + (32768 + robo_left_enc)    #65535 - last_left_enc + robo_left_enc
  elif last_left_enc < -10000 and robo_left_enc > 10000:
    p1_add = -(32768 + last_left_enc) - (32767 - robo_left_enc)   #-65535 - last_left_enc + robo_left_enc
  else:
    p1_add = 0
    
  #check for roll over
  if last_right_enc > 10000 and robo_right_enc < -10000:
    p2_add = (32767 - last_right_enc) + (32768 + robo_right_enc)    #65535 - last_right_enc + robo_right_enc
  elif last_right_enc < -10000 and robo_right_enc > 10000:
    p2_add = -(32768 + last_right_enc) - (32767 - robo_right_enc)   #-65535 - last_right_enc + robo_right_enc
  else:
    p2_add = 0
    
  P1 += (robo_left_enc - last_left_enc + p1_add)
  P2 += (robo_right_enc - last_right_enc + p2_add)
  
  last_left_enc = robo_left_enc
  last_right_enc = robo_right_enc
  
  robo_calc_pos(P1, P2)

  #print(f'polar r: {polar_r(P1, P2)} deg: {radians_to_deg(polar_theta(P1, P2))}')
  
def rdata_travel_angle(data):
  global robo_state, robo_travel, robo_angle
  
  robo_travel += int.from_bytes(data[0:2], byteorder='big', signed=True)
  robo_angle  += int.from_bytes(data[2:4], byteorder='big', signed=True)

  if robo_state == rWaitCF:
    robo_travel = 0
    robo_angle = 0
    robo_state = rWaitCE
  
  if robo_state != rIdle:    
    #print(f'travel: {robo_travel} polar r: {robo_vector_pol[0]}    angle: {robo_angle} deg: {radians_to_deg(robo_vector_pol[1])}')
    print(f'travel: {robo_travel:.1f} x: {robo_vector_xy[0]:.1f}    angle: {robo_angle:.1f} theta: {radians_to_deg(robo_orientation):.1f}  y: {robo_vector_xy[1]:.1f}')
    #print(P1, P2)


try:
  print(os.name)
  if os.name == 'posix':
    ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=0, rtscts=0, xonxoff=0)
  else:
    ser = serial.Serial(port="COM37", baudrate=115200, timeout=0, rtscts=0, xonxoff=0)
  time.sleep(0.5)
  ser_port = True
  print('serial port open okay')
  
except:
  ser_port = False
  print('Robo not connected')

def robo_read():
  global thread_run
  global robo_state, robo_travel, robo_angle
  global key
  global data_rcv
  
  while thread_run:
    time.sleep(0.1)
    data = ser.read(80)
    if len(data) > 0:
      #print(len(data), data.decode('iso-8859-1'))
      #print(len(data))
      data_rcv = True
    
    #response to packet id 100  
    if len(data) == 80:
      rdata_button_press(data[11])      #packet 18
      rdata_enc_feedback(data[52:56])   #packets 43 and 44
      rdata_travel_angle(data[12:16])   #packets 19 and 20
      
      
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
      if robo_travel >= 500:
        robo_drive(0, 0)
        robo_state = rFaceBackward
        
    if robo_state == rFaceBackward:
      robo_state = rWaitFaceBackward
      robo_drive(50, 1)
      
    if robo_state == rWaitFaceBackward:
      if robo_angle >= 140:
        robo_drive(0, 0)
        robo_state = rGoBack
      
    if robo_state == rGoBack:
      robo_state = rWaitGoBack
      robo_drive(50, 32767)
    
    if robo_state == rWaitGoBack:
      if robo_travel >= 1000:
        robo_drive(0, 0)
        robo_state = rFaceForward
        
    if robo_state == rFaceForward:
      robo_state = rWaitFaceForward
      robo_drive(50, -1)
      
    if robo_state == rWaitFaceForward:
      if robo_angle <= 1:
        robo_drive(0, 0)
        robo_state = rIdle
      
    


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
  
  data_rcv = False
  data_request_time = time.time() + 1
  
  robo_send([142, packet]);
  
def robo_run():
  global robo_state
  
  print('begin run')
  if robo_state == rIdle:
    robo_sing()
    robo_state = rForward


#main routine
#if connected to robot
#start a thread to read the serial port
#listen to keyboard
if ser_port:
  thread_run = True
  
  t_rx = threading.Thread(name='rx_thread', target=robo_read, args=[])
  t_rx.setDaemon(True)
  t_rx.start()

  #open the OI
  robo_init()
  time.sleep(1)
  robo_full_control()
  robo_request_time = time.time() + 3
  init_feedback = 3

  try:
    while True:
      time.sleep(0.2)
      
      #get robot packets
      if data_rcv or (time.time() > data_request_time):
        robo_request_packet(100)  #get 80 bytes of info back
        if init_feedback > 0:
          init_feedback -= 1
          robo_state = rClearFeedback
      
      if keyboard.is_pressed('b') or key == 'b':     #begin the program
        robo_run()
        key = ''
      elif keyboard.is_pressed('r'):   #reset
        robo_reset()
      elif keyboard.is_pressed('s') or key == 's':   #beep
        robo_sing()
        key = ''
      elif keyboard.is_pressed('t'):   #turn clockwise
        robo_drive(50, -1)
      elif keyboard.is_pressed('g'):   #straight
        robo_drive(20, 32767)
      elif keyboard.is_pressed('p') or key == 'p':   #stop
        robo_state = rIdle
        robo_drive(0, -1)        
        key = ''
      elif keyboard.is_pressed('i'):   #show p1 and p2
        print(P1, P2)
      elif keyboard.is_pressed('c'):
        robo_state = rClearFeedback
      elif keyboard.is_pressed('q'):   #quit
        thread_run = False
        time.sleep(0.9)
        break
  
  except KeyboardInterrupt:
    pass
  
  #close the OI
  robo_close()

  ser.close()


  
