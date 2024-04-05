import RPi.GPIO as GPIO
import time

PIN_SERVO = 11

def setup_servo(pin):
    GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
    GPIO.setup(pin, GPIO.OUT)
    servo = GPIO.PWM(pin, 50)  # 50Hz pulse for servo
    servo.start(0)  # Initialization with 0 duty cycle
    return servo

def servo_angle_to_duty_cycle(angle):
    """Convert an angle to the suitable duty cycle."""
    return 2 + (angle / 18)

def gradually_move_servo(servo, current_angle, target_angle, step=1, delay=0.05):
    """Gradually move the servo to target_angle in smaller steps and with a slight delay."""
    # Determine the current angle based on the duty cycle (assuming starting from 0 for simplicity)
    current_duty_cycle = servo_angle_to_duty_cycle(target_angle)
    #current_angle = (current_duty_cycle - 2) * 18

    # Calculate steps required to reach the target angle
    steps = int(abs(target_angle - current_angle) / step)
    print(f'There are {steps} steps. Target angle: {target_angle}. Current angle: {current_angle}')

    for _ in range(steps):
        # Increment or decrement the current angle towards the target angle
        current_angle += step if target_angle > current_angle else -step
        duty_cycle = servo_angle_to_duty_cycle(current_angle)
        servo.ChangeDutyCycle(duty_cycle)
        time.sleep(delay)
    
    # Ensures the servo moves to the exact target angle in case it wasn't divisible by the step
    servo.ChangeDutyCycle(servo_angle_to_duty_cycle(target_angle))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)  # Stop sending signal to hold position

# Main section
if __name__ == '__main__':
    servo_pin = PIN_SERVO
    servo = setup_servo(servo_pin)

    try:
        while True:
            angle = float(input('Enter angle between 0 & 180: '))
            if 0 <= angle <= 40:
                gradually_move_servo(servo, 0, angle)      # TODO: Ajustar bien parametros
            else:
                print("Angle must be between 0 and 180 degrees.")
    finally:
        servo.stop()
        GPIO.cleanup()
        print("Goodbye!")










# # Import libraries
# import RPi.GPIO as GPIO
# import time

# # Set GPIO numbering mode
# GPIO.setmode(GPIO.BOARD)

# # Set pin 11 as an output, and set servo1 as pin 11 as PWM
# GPIO.setup(11,GPIO.OUT)
# servo1 = GPIO.PWM(11,50) # Note 11 is pin, 50 = 50Hz pulse

# #start PWM running, but with value of 0 (pulse off)
# servo1.start(0)
# print ("Waiting for 2 seconds")
# time.sleep(2)

# #Let's move the servo!
# print ("Rotating 180 degrees in 10 steps")

# # Define variable duty
# duty = 2

# # Loop for duty values from 2 to 12 (0 to 180 degrees)
# while duty <= 12:
#     servo1.ChangeDutyCycle(duty)
#     time.sleep(1)
#     duty = duty + 1

# # Wait a couple of seconds
# time.sleep(2)

# # Turn back to 90 degrees
# print ("Turning back to 90 degrees for 2 seconds")
# servo1.ChangeDutyCycle(7)
# time.sleep(2)

# #turn back to 0 degrees
# print ("Turning back to 0 degrees")
# servo1.ChangeDutyCycle(2)
# time.sleep(0.5)
# servo1.ChangeDutyCycle(0)

# #Clean things up at the end
# servo1.stop()
# GPIO.cleanup()
# print ("Goodbye")





















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










# import RPi.GPIO as GPIO
# import time

# SERVO_MIN_PULSE = 500
# SERVO_MAX_PULSE = 2500
# Servo = 23

# def map(value, inMin, inMax, outMin, outMax):
#     return (outMax - outMin) * (value - inMin) / (inMax - inMin) + outMin

# def setup():
#     global p
#     GPIO.setmode(GPIO.BCM)       
#     GPIO.setup(Servo, GPIO.OUT)  
#     GPIO.output(Servo, GPIO.LOW) 
#     p = GPIO.PWM(Servo, 50)     
#     p.start(0)                    

# def setAngle(angle):      
#     angle = max(0, min(180, angle))
#     pulse_width = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE)
#     pwm = map(pulse_width, 0, 20000, 0, 100)
#     p.ChangeDutyCycle(pwm)

# def loop():
#     while True:
#         for i in range(0, 181, 5):   
#             setAngle(i)    
#             time.sleep(0.002)
#         time.sleep(1)
#         for i in range(180, -1, -5): 
#             setAngle(i)
#             time.sleep(0.001)
#         time.sleep(1)

# def destroy():
#     p.stop()
#     GPIO.cleanup()

# if __name__ == '__main__':    
#     setup()
#     try:
#         loop()
#     except KeyboardInterrupt:
#         destroy()
