#!/usr/bin/python

#Import GPIO library
import RPi.GPIO as GPIO  

# import time
# from time import sleep

#Set GPIO pin numbering
GPIO.setmode(GPIO.BOARD)                       

GPIO.setwarnings(False)

########################################### IR Sensor Content ###########################################

# GPIO pins that IR sensors are connected to
IR1_INPUT_PIN = 7
IR2_INPUT_PIN = 12

DARK = 1        # When there is not enough reflected IR light, GPIO reads as 1
LIGHT = 0       # When there is enough reflected IR light, GPIO reads as 0

INVALID = -1

# Configure IR sensor Pins to be inputs
GPIO.setup(IR1_INPUT_PIN, GPIO.IN)
GPIO.setup(IR2_INPUT_PIN, GPIO.IN)

def get_ir_state(inputPin):
    if inputPin not in [IR1_INPUT_PIN, IR2_INPUT_PIN]:
        print("get_ir_state was called with an invalid input pin number: " + str(inputPin))
        print("valid pin numbers include: " + str(IR1_INPUT_PIN) + ", " + str(IR2_INPUT_PIN) + ".")
        return -1
    else:
        pinValue = GPIO.input(inputPin)
        return pinValue


########################################### Motor Content ###########################################

class Motor:
    ''' Class to handle interaction with the motor pins
    Supports redefinition of "forward" and "backward" depending on how motors are connected
    Use the supplied Motorshieldtest module to test the correct configuration for your project.
    
    Arguments:
    motor = string motor pin label (i.e. "MOTOR1","MOTOR2","MOTOR3","MOTOR4") identifying the pins to which
            the motor is connected.
    config = int defining which pins control "forward" and "backward" movement.
    '''
    motorpins = {"MOTOR4":{"config":{1:{"e":32,"f":24,"r":26},2:{"e":32,"f":26,"r":24}},"arrow":1},
                 "MOTOR3":{"config":{1:{"e":19,"f":21,"r":23},2:{"e":19,"f":23,"r":21}}, "arrow":2},
                 "MOTOR2":{"config":{1:{"e":22,"f":16,"r":18},2:{"e":22,"f":18,"r":16}}, "arrow":3},
                 "MOTOR1":{"config":{1:{"e":11,"f":15,"r":13},2:{"e":11,"f":13,"r":15}},"arrow":4}}
    
    def __init__(self, motor, config):
        self.testMode = False
        self.arrow = Arrow(self.motorpins[motor]["arrow"])
        self.pins = self.motorpins[motor]["config"][config]
        GPIO.setup(self.pins['e'],GPIO.OUT)
        GPIO.setup(self.pins['f'],GPIO.OUT)
        GPIO.setup(self.pins['r'],GPIO.OUT)
        self.PWM = GPIO.PWM(self.pins['e'], 50)  # 50Hz frequency
        self.PWM.start(0)
        GPIO.output(self.pins['e'],GPIO.HIGH)
        GPIO.output(self.pins['f'],GPIO.LOW)
        GPIO.output(self.pins['r'],GPIO.LOW)

    def test(self, state):
        ''' Puts the motor into test mode
        When in test mode the Arrow associated with the motor receives power on "forward"
        rather than the motor. Useful when testing your code. 
        
        Arguments:
        state = boolean
        '''
        self.testMode = state

    def forward(self, speed):
        ''' Starts the motor turning in its configured "forward" direction.

        Arguments:
        speed = Duty Cycle Percentage from 0 to 100.
        0 - stop and 100 - maximum speed
        '''    
        print("Forward")
        if self.testMode:
            self.arrow.on()
        else:
            self.PWM.ChangeDutyCycle(speed)
            GPIO.output(self.pins['f'],GPIO.HIGH)
            GPIO.output(self.pins['r'],GPIO.LOW)

    def reverse(self,speed):
        ''' Starts the motor turning in its configured "reverse" direction.

        Arguments:
        speed = Duty Cycle Percentage from 0 to 100.
        0 - stop and 100 - maximum speed
     '''
        print("Reverse")
        if self.testMode:
            self.arrow.off()
        else:
            self.PWM.ChangeDutyCycle(speed)
            GPIO.output(self.pins['f'],GPIO.LOW)
            GPIO.output(self.pins['r'],GPIO.HIGH)

    def stop(self):
        ''' Stops power to the motor,
     '''
        print("Stop")
        self.arrow.off()
        self.PWM.ChangeDutyCycle(0)
        GPIO.output(self.pins['f'],GPIO.LOW)
        GPIO.output(self.pins['r'],GPIO.LOW)

    def speed(self):
        ''' Control Speed of Motor,
     '''
        
class Arrow():
    ''' Defines an object for controlling one of the LED arrows on the Motorshield.
    
        Arguments:
        which = integer label for each arrow. The arrow number if arbitrary starting with:
            1 = Arrow closest to the Motorshield's power pins and running clockwise round the board
            ...
            4 = Arrow closest to the motor pins.
    '''
    arrowpins={1:33,2:35,3:37,4:36}

    def __init__(self, which):
        self.pin = self.arrowpins[which]
        GPIO.setup(self.pin,GPIO.OUT)
        GPIO.output(self.pin, GPIO.LOW)

    def on(self):
        GPIO.output(self.pin,GPIO.HIGH)

    def off(self):
        GPIO.output(self.pin,GPIO.LOW)