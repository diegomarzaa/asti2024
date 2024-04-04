from gpiozero import AngularServo
from time import sleep

def main(args=None):
  servo = AngularServo(10, min_pulse_width=0.0006, max_pulse_width=0.0023)
  while True:
    servo.angle = 90
    sleep(2)
    servo.angle = 0
    sleep(2)
    servo.angle = -90
    sleep(2)
    servo.angle = 0
    sleep(2)

if __name__ == '__main__':
  main()
