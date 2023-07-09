import RPi.GPIO as GPIO
from time import sleep

if __name__ == "__main__":

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(21, GPIO.IN)

try:  
    while True:           
        if GPIO.input(21): 
            print("Y")
        else:  
            print("N")  
        sleep(0.1)          
  
finally:                 
    GPIO.cleanup()
