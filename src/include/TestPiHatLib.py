#!/usr/bin/python

from PiHatLib import *
import time

# Assign each motor 
m1 = Motor("MOTOR1",1)
m2 = Motor("MOTOR2",1)
m3 = Motor("MOTOR3",1)
m4 = Motor("MOTOR4",1)

# Assign each Arrow
back_arrow = Arrow(1)
left_arrow = Arrow(2)
front_arrow = Arrow(3) 
right_arrow = Arrow(4)

try:
    while True:

#----------- Check IR1 Value ------------# 
        print("Reading IR1")
        ir1Value = get_ir_state(IR1_INPUT_PIN)
        if(ir1Value == DARK):
            print("IR1 is Dark")
        elif(ir1Value == LIGHT):
            print("IR1 is LIGHT")
        elif(ir1Value == INVALID):
            print("INVALID pin")
        time.sleep(2)
#--------------------------------------#

#----------- Check IR2 Value ------------# 
        print("Reading IR2")
        ir2Value = get_ir_state(IR2_INPUT_PIN)
        if(ir2Value == DARK):
            print("IR2 is Dark")
        elif(ir2Value == LIGHT):
            print("IR2 is LIGHT")
        elif(ir2Value == INVALID):
            print("INVALID pin")
        time.sleep(2)
#--------------------------------------#

#------- Check Invalid pin Value --------# 
        print("Reading Invalid pin")
        irValue = get_ir_state(1)
        if(irValue == DARK):
            print("IR is Dark")
        elif(irValue == LIGHT):
            print("IR is LIGHT")
        elif(irValue == INVALID):
            print("INVALID pin")
        time.sleep(2)
#--------------------------------------#
        
#---------- M1 Forward at 100% speed -------------#
        print("M1 Forward at 100\% speed")
        m1.forward(100)
        time.sleep(3)
#-------------------------------------------------#

#---------- M2 Forward at 50% speed -------------#
        print("M1 Forward at 50\% speed")
        m1.forward(50)
        time.sleep(3)
#-------------------------------------------------#

#-------------------- M1 Stop --------------------#
        print("M1 stop")
        m1.stop()
        time.sleep(3)
#-------------------------------------------------#
        
#---------- M1 Reverse at 100% speed -------------#
        print("M1 Reverse at 100\% speed")
        m1.reverse(100)
        time.sleep(3)
#-------------------------------------------------#

#---------- M2 Reverse at 50% speed -------------#
        print("M1 Reverse at 50\% speed")
        m1.reverse(50)
        time.sleep(3)
#-------------------------------------------------#

#-------------------- M1 Stop --------------------#
        print("M1 stop")
        m1.stop()
        time.sleep(3)
#-------------------------------------------------#

#-------------------- Front Arrow ----------------#
        print("Front Arrow")
        front_arrow.on()
        back_arrow.off()
        left_arrow.off()
        right_arrow.off()
        time.sleep(3)
#-------------------------------------------------#

#-------------------- Back Arrow -----------------#
        print("Back Arrow")
        front_arrow.off()
        back_arrow.on()
        left_arrow.off()
        right_arrow.off()
        time.sleep(3)
#-------------------------------------------------#

#-------------------- Left Arrow -----------------#
        print("Left Arrow")
        front_arrow.off()
        back_arrow.off()
        left_arrow.on()
        right_arrow.off()
        time.sleep(3)
#-------------------------------------------------#

#-------------------- Right Arrow -----------------#
        print("Right Arrow")
        front_arrow.off()
        back_arrow.off()
        left_arrow.off()
        right_arrow.on()
        time.sleep(3)
#-------------------------------------------------#

#-------------------- Arrows off -----------------#
        print("Arrows off")
        front_arrow.off()
        back_arrow.off()
        left_arrow.off()
        right_arrow.off()
        time.sleep(3)
#-------------------------------------------------#

except KeyboardInterrupt:
    GPIO.cleanup()
