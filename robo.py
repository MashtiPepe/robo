import serial
import time
import threading
import keyboard
import os

rIdle             = 'idle'
rClearFeedback    = 'clear feedback'
rWaitCF           = 'wait clear feedback'
rForward          = 'forward'
rWaitForward      = 'wait forward'
rFaceBackward     = 'face backward'
rWaitFaceBackward = 'wait face backward'
rGoBack           = 'go back'
rWaitGoBack       = 'wait go back'
rFaceForward      = 'face forward'
rWaitFaceForward  = 'wait face forward'

robo_state = 'idle'
robo_travel = 0
robo_angle = 0
robo_states = [rIdle, rClearFeedback, rWaitCF, rForward, rWaitForward, rFaceBackward, rWaitFaceBackward, rGoBack, rWaitGoBack, rFaceForward, rWaitFaceForward]

key = ''

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
  
  while thread_run:
    time.sleep(0.1)
    data = ser.read(50)
    if len(data) > 0:
      #print(len(data), data.decode('iso-8859-1'))
      print(len(data), data)
      
    if len(data) == 1:
      if data[0] & 1 > 0:
        key = 'b'
      elif data[0] & 2 > 0:
        key = 's'
      elif data[0] & 4 > 0:
        key = 'p'
      elif data[0] & 64 > 0:
        robo_state = rClearFeedback
      
    if robo_state == rWaitCF:
      if len(data) == 6:
        robo_travel = 0
        robo_angle = 0
        robo_state = rIdle
      elif len(data)==0:
        robo_sensors(2)
        
    if robo_state == rWaitForward:
      if len(data) == 6:
        robo_travel += int.from_bytes(data[2:4], byteorder='big', signed=True)
        robo_angle  += int.from_bytes(data[4:6], byteorder='big', signed=True)
        
        print(robo_travel, robo_angle)
        if robo_travel >= 500:
          robo_drive(0, 0)
          robo_state = rFaceBackward
      elif len(data)==0:
        robo_sensors(2)
        
    if robo_state == rWaitFaceBackward:
      if len(data) == 6:
        robo_travel += int.from_bytes(data[2:4], byteorder='big', signed=True)
        robo_angle  += int.from_bytes(data[4:6], byteorder='big', signed=True)
        
        print(robo_travel, robo_angle)
        if robo_angle >= 140:
          robo_drive(0, 0)
          robo_state = rGoBack
      elif len(data)==0:
        robo_sensors(2)
      
    if robo_state == rWaitGoBack:
      if len(data) == 6:
        robo_travel -= int.from_bytes(data[2:4], byteorder='big', signed=True)
        robo_angle  += int.from_bytes(data[4:6], byteorder='big', signed=True)
        
        print(robo_travel, robo_angle)
        if robo_travel <= 1:
          robo_drive(0, 0)
          robo_state = rFaceForward
      elif len(data)==0:
        robo_sensors(2)
        
    if robo_state == rWaitFaceForward:
      if len(data) == 6:
        robo_travel += int.from_bytes(data[2:4], byteorder='big', signed=True)
        robo_angle  += int.from_bytes(data[4:6], byteorder='big', signed=True)
        
        print(robo_travel, robo_angle)
        if robo_angle <= 1:
          robo_drive(0, 0)
          robo_state = rIdle
      elif len(data)==0:
        robo_sensors(2)
      
    if robo_state == rClearFeedback:
      robo_state = rWaitCF
      robo_travel = 0
      robo_angle = 0
      robo_sensors(2)
      
    if robo_state == rForward:
      robo_state = rWaitForward
      robo_drive(50, 32767)
      
    if robo_state == rWaitForward:
      robo_sensors(2)
      
    if robo_state == rFaceBackward:
      robo_state = rWaitFaceBackward
      robo_drive(50, 1)
      
    if robo_state == rGoBack:
      robo_state = rWaitGoBack
      robo_drive(50, 32767)
    
    if robo_state == rFaceForward:
      robo_state = rWaitFaceForward
      robo_drive(50, -1)
      
    


def robo_send(cmds):
  data = bytearray();
  for i in cmds:
    data.extend(chr(i).encode('iso-8859-1'))
    
  ser.write(data)
  
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
  #robo_send([140, 3, 0, 100]) 
  #robo_send([141, 1])
  
def robo_drive(speed, radius):
  print(f'drive {speed}mm/sec {radius}mm')
  robo_send([137] + robo_num(speed) + robo_num(radius))
  
def robo_reset():
  print('reset')
  robo_send([7])
  
def robo_full_control():
  print('full control')
  robo_send([132])
  
def robo_sensors(packet):
  #print('sensors')
  robo_send([142, packet]);
  
def robo_run():
  global robo_state
  
  print('begin run')
  if robo_state == rIdle:
    robo_sing()
    robo_state = rForward


if ser_port:
  thread_run = True
  
  t_rx = threading.Thread(name='rx_thread', target=robo_read, args=[])
  t_rx.setDaemon(True)
  t_rx.start()

  #open the OI
  robo_init()
  time.sleep(1)
  robo_full_control()
  robo_state = rClearFeedback

  try:
    while True:
      time.sleep(1)
      
      #get any key presses
      robo_sensors(18)
      
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
      elif keyboard.is_pressed('i'):   #feedback
        robo_sensors(18)
      elif keyboard.is_pressed('c'):
        if robo_state == rIdle:
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


  
