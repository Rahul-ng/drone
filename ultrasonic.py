import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
#define ultrasonic sensor pin 
PIN_TRIG_FRONT = 23
PIN_TRIG_BACK = 24
PIN_TRIG_LEFT = 25
PIN_TRIG_RIGHT = 8
PIN_ECHO_FRONT = 12
PIN_ECHO_BACK = 16
PIN_ECHO_LEFT = 20
PIN_ECHO_RIGHT = 21
#end

 
def measure_front():
    GPIO.setup(PIN_TRIG_FRONT,GPIO.OUT)
    GPIO.setup(PIN_ECHO_FRONT,GPIO.IN)
    GPIO.output(PIN_TRIG_FRONT,False)    #SET TO 0 OR FALSE TO SETTLE
    time.sleep(0.2)
    GPIO.output(PIN_TRIG_FRONT,True)
    time.sleep(0.00001)
    GPIO.output(PIN_TRIG_FRONT,False)
    while GPIO.input(PIN_ECHO_FRONT)==0:
        pulse_start=time.time()
    while GPIO.input(PIN_ECHO_FRONT)==1:
        pulse_end=time.time()
    pulse_duration=pulse_end-pulse_start
    distance=pulse_duration*17150
    distance=round(distance,2)
    print("sensor front distance:",distance,"cm")
    time.sleep(0.5)    
    return distance

def measure_back():
    GPIO.setup(PIN_TRIG_BACK,GPIO.OUT)
    GPIO.setup(PIN_ECHO_BACK,GPIO.IN)
    GPIO.output(PIN_TRIG_BACK,False)    #SET TO 0 OR FALSE TO SETTLE
    time.sleep(0.2)
    GPIO.output(PIN_TRIG_BACK,True)
    time.sleep(0.00001)
    GPIO.output(PIN_TRIG_BACK,False)
    while GPIO.input(PIN_ECHO_BACK)==0:
        pulse_start=time.time()
    while GPIO.input(PIN_ECHO_BACK)==1:
        pulse_end=time.time()
    pulse_duration=pulse_end-pulse_start
    distance=pulse_duration*17150
    distance=round(distance,2)
    print("sensor back distance:",distance,"cm")
    time.sleep(0.5)    
    return distance


def measure_left():
    GPIO.setup(PIN_TRIG_LEFT,GPIO.OUT)
    GPIO.setup(PIN_ECHO_LEFT,GPIO.IN)
    GPIO.output(PIN_TRIG_LEFT,False)    #SET TO 0 OR FALSE TO SETTLE
    time.sleep(0.2)
    GPIO.output(PIN_TRIG_LEFT,True)
    time.sleep(0.00001)
    GPIO.output(PIN_TRIG_LEFT,False)
    while GPIO.input(PIN_ECHO_LEFT)==0:
        pulse_start=time.time()
    while GPIO.input(PIN_ECHO_LEFT)==1:
        pulse_end=time.time()
    pulse_duration=pulse_end-pulse_start
    distance=pulse_duration*17150
    distance=round(distance,2)
    print("sensor left distance:",distance,"cm")
    time.sleep(0.5)    
    return distance

def measure_right():
    GPIO.setup(PIN_TRIG_RIGHT,GPIO.OUT)
    GPIO.setup(PIN_ECHO_RIGHT,GPIO.IN)
    GPIO.output(PIN_TRIG_RIGHT,False)    #SET TO 0 OR FALSE TO SETTLE
    time.sleep(0.2)
    GPIO.output(PIN_TRIG_RIGHT,True)
    time.sleep(0.00001)
    GPIO.output(PIN_TRIG_RIGHT,False)
    while GPIO.input(PIN_ECHO_RIGHT)==0:
        pulse_start=time.time()
    while GPIO.input(PIN_ECHO_RIGHT)==1:
        pulse_end=time.time()
    pulse_duration=pulse_end-pulse_start
    distance=pulse_duration*17150
    distance=round(distance,2)
    print("sensor right distance:",distance,"cm")
    time.sleep(0.5)    
    return distance