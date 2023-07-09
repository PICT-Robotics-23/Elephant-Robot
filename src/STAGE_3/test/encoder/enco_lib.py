import Encoder
import RPi.GPIO as GPIO

def constrain(value, minn, maxn):
    return min(max(value, minn), maxn)

if __name__ == "__main__":
    enc = Encoder.Encoder(17, 27)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # pick motor 1
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(18, GPIO.OUT)
    motorA = GPIO.PWM(18, 1000)
    motorA.start(0)

    High_count = int(input())
    setpoint = High_count
    err = setpoint - int(enc.read())
    kp = 2.0

    # general
    pwm = 0
    direction = 0

    while(err > 10):
        err = setpoint - int(enc.read())
        print(err)
        pwm = kp * err
        if (pwm > 0):
            GPIO.output(23, GPIO.HIGH)
        else:
            GPIO.output(23, GPIO.LOW)
        pwm = constrain(abs(pwm), 0, 100)
        motorA.ChangeDutyCycle(pwm)

    motorA.ChangeDutyCycle(0)

            


    