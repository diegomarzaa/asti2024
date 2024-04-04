# from gpiozero import AngularServo
# import RPi.GPIO as GPIO
# from time import sleep

# SERVO_MIN_PULSE = 500
# SERVO_MAX_PULSE = 2500
# Servo = 23


# def main(args=None):
#   servo = AngularServo(10, min_pulse_width=0.0006, max_pulse_width=0.0023)
#   while True:
#     servo.angle = 90
#     sleep(2)
#     servo.angle = 0
#     sleep(2)
#     servo.angle = -90
#     sleep(2)
#     servo.angle = 0
#     sleep(2)

# if __name__ == '__main__':
#   main()










import RPi.GPIO as GPIO
import time

SERVO_MIN_PULSE = 500
SERVO_MAX_PULSE = 2500
Servo = 23

def map(value, inMin, inMax, outMin, outMax):
    return (outMax - outMin) * (value - inMin) / (inMax - inMin) + outMin

def setup():
    global p
    GPIO.setmode(GPIO.BCM)       
    GPIO.setup(Servo, GPIO.OUT)  
    GPIO.output(Servo, GPIO.LOW) 
    p = GPIO.PWM(Servo, 50)     
    p.start(0)                    

def setAngle(angle):      
    angle = max(0, min(180, angle))
    pulse_width = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE)
    pwm = map(pulse_width, 0, 20000, 0, 100)
    p.ChangeDutyCycle(pwm)

def loop():
    while True:
        for i in range(0, 181, 5):   
            setAngle(i)    
            time.sleep(0.002)
        time.sleep(1)
        for i in range(180, -1, -5): 
            setAngle(i)
            time.sleep(0.001)
        time.sleep(1)

def destroy():
    p.stop()
    GPIO.cleanup()

if __name__ == '__main__':    
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        destroy()
