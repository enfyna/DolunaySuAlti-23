from __future__ import print_function
from statistics import mode
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil
import argparse  
from enum import Enum
import RPi.GPIO as GPIO

pin_role1 = 16
pin_role2 = 13
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin_role1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(pin_role2, GPIO.OUT, initial=GPIO.LOW)
# p1 = GPIO.PWM(pin_role1, 500)
# p2 = GPIO.PWM(pin_role2, 500)
# p1.start(0)
# p2.start(0)
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', default='/dev/serial/by-id/usb-ArduPilot_Pixhawk1_19003D000551353532333634-if00')
#parser.add_argument('--connect', default='/dev/serial/by-id/usb-ArduPilot_Pixhawk1_2B004D000B51373232393438-if00')

args = parser.parse_args()

connection_string = args.connect
sitl = None

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

class Modes(Enum):
    MANUAL   = 0
    ACRO     = 1
    UNKNOWN  = 2
    STEERING = 3
    HOLD     = 4
    LOITER   = 5
    FOLLOW   = 6
    SIMPLE   = 7
    AUTO     = 8
    RTL      = 9
    SRTL     = 10
    GUIDED   = 11

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)



@vehicle.on_message('HEARTBEAT')
def listener(self, name, message):
    if(message.base_mode > 0):
        if Modes(message.custom_mode) == Modes.MANUAL:
            GPIO.output(pin_role2, GPIO.LOW)
            GPIO.output(pin_role1,GPIO.HIGH)
            # p1.ChangeDutyCycle(0)
            # p2.ChangeDutyCycle(100)
        else: #elif Modes(message.custom_mode) == Modes.HOLD:
            GPIO.output(pin_role1,GPIO.LOW)
            GPIO.output(pin_role2, GPIO.HIGH)
            # p1.ChangeDutyCycle(100)
            # p2.ChangeDutyCycle(0)
        print("Base: {}, Custom: {}\tMODE: {}".format((message.base_mode), (message.custom_mode), Modes(message.custom_mode)))

while(1):
    pass
