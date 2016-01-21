#!/usr/bin/env python
import socket
import struct
import sys
import os
import time
import binascii
import math
from random import randint


#PC_IP = "192.168.0.145"    #127.0.0.1"
PC_IP = "192.168.0.101"    #127.0.0.1"
LIS_PORT = 64417

CONTROL_IP = "192.168.0.145"   #"127.0.0.1"
CONTROL_PORT = 41010


# current Millisecond function
def current_milli_time():
    """Get current timestamp in millisecond"""
    return int(round(time.time() * 1000))



UDPSOCKL = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP Lis
UDPSOCKL.settimeout(0.4)
UDPSOCKL.bind((PC_IP, LIS_PORT))

UDPSOCKS = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP Send
#UDPSOCKS.bind((PC_IP, PC_SEND_PORT))

gps_log = open("localize"+time.strftime("%Y-%m-%d-%H%M%S")+".csv","w")


prev_ticks_left = 0
prev_ticks_right = 0

prev_x, prev_y, prev_theta, prev_head = 0.0, 0.0, math.radians(0.0), 0.0
new_x, new_y, new_theta = 0.0, 0.0, 0.0 

prev_time = current_milli_time()

tot_d_left_wheel = 0
tot_d_right_wheel = 0
R = 0.2413

MAX_COUNT =  2147483647 #65535 #4096


utm_x = []
utm_y = []

vl, vr = 0.0, 0.0

INIT = 0

def update_odometry(_Tleft, _Tright):
    global prev_time 
    global new_x
    global new_y 
    global new_theta
    global prev_x
    global prev_y
    global prev_theta
    global prev_ticks_left
    global prev_ticks_right
    global utm_x
    global utm_y
    global prev_head
    global vl, vr
    global INIT
    
    
    _ts = current_milli_time()

    deltaT = current_milli_time() - prev_time
    print deltaT
    if deltaT == 0:
        deltaT = 0.0001
    prev_time = _ts
    
    robot_wheel_base_length = 0.6096
    LN = float( 3150.0 )
    RN = float( 3150.0 )
    # read the wheel encoder values
    ticks_left = _Tleft
    ticks_right = _Tright
    
    # get the difference in ticks since the last iteration
    if INIT == 0:
        prev_ticks_left = ticks_left
        prev_ticks_right = ticks_right
        INIT = 1
        
    d_ticks_left = (ticks_left - prev_ticks_left)
    #if d_ticks_left < 0:
    #    d_ticks_left += MAX_COUNT
    
    d_ticks_right = (ticks_right - prev_ticks_right)
    #if d_ticks_right < 0:
    #    d_ticks_right += MAX_COUNT
    
    print "ticks", d_ticks_left, d_ticks_right
    
    # estimate the wheel movements
    d_left_wheel = round(((2*math.pi)*R*( d_ticks_left / LN )), 4)
    d_right_wheel = round(((2*math.pi)*R*( d_ticks_right / RN )), 4)
    d_center = round((0.5 * ( d_left_wheel + d_right_wheel )), 4)
    
    # calculate the new pose
    print "distance", d_left_wheel, d_right_wheel, d_center
    
    vl = d_left_wheel * 1000 /(deltaT);
    vr = d_right_wheel * 1000 /(deltaT);
    
    print "prev_theta",  prev_theta
    
    new_x = prev_x + ( d_center * math.cos( prev_theta ) )
    new_y = prev_y + ( d_center * math.sin( prev_theta ) )
    new_theta = prev_theta + round(((( d_right_wheel - d_left_wheel ) / robot_wheel_base_length )),4)
    #new_theta =  math.atan2(new_y, new_x)
    print "dt", (( d_right_wheel - d_left_wheel ) / robot_wheel_base_length )
    
    
    new_theta = round((new_theta % (2*math.pi)), 4)
    
   
    
    # update the pose estimate with the new values
   
    
    
    _head = math.degrees(new_theta)
    
    if _head > 180:
        _head = _head - 360
    
    prev_head = _head
    #print "pose :", round(new_x, 4), round(new_y, 4), new_theta 
    #print "vel :", round(vl, 2), round(vr, 2), round(((vl + vr)/2.),2)
    
    utm_x.append(new_x)
    utm_y.append(new_y)
                                    
    
    # save the current tick count for the next iteration
    prev_ticks_left = ticks_left
    prev_ticks_right = ticks_right
    prev_x, prev_y, prev_theta = new_x, new_y, new_theta
    
    gps_log.write('%f,%f,%f,%f, %f \n' %(new_x, new_y, _head, vl, vr))

    
    return new_x, new_y, _head, vl, vr




while True:
    try:
        _data, _ = UDPSOCKL.recvfrom(1024)
        print "len", len(_data)
        if len(_data) == 32:  # ecoms packed capture
                _structure = struct.Struct('> ' + '1s 1s H I I h h h h I h 1s 1s h h')
                unpacked_data = _structure.unpack(_data)
                print unpacked_data
        elif len(_data) == 36:  # ecoms packed capture
                _structure = struct.Struct('< ' + '1s 1s H I I i i i h I h h h')
                unpacked_data = _structure.unpack(_data)
                print unpacked_data
                
                
                X, Y, Z, _vl, _vr = update_odometry(unpacked_data[5], unpacked_data[7])
                
                _left = (_vl)/R
                _right = (_vr)/R
                
        elif len(_data) == 14:  # ecoms packed capture
            _structure = struct.Struct('> ' + '2B h h 8B')
            unpacked_data = _structure.unpack(_data)
            print unpacked_data
        else:
            print "Undefined UDP Packed length", len(_data)
    except socket.error:
            print "Socket Error. No message received"
    
    except KeyboardInterrupt:
        print "Keyboard press"
        UDPSOCKL.close()
        sys.exit(0)
    
    _fvalues = (int('F1', 16), int('A1', 16), prev_x, prev_y, prev_head, vl, vr)
    _fstructure = struct.Struct('< ' + '2B d d d d d')
    _fdata = _fstructure.pack(*_fvalues)
    UDPSOCKS.sendto(_fdata, (CONTROL_IP, CONTROL_PORT))
    print "sending", prev_x, prev_y, prev_head, vl, vr
    
    
    
        
    
