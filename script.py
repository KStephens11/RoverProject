# GPIOZERO_PIN_FACTORY=pigpio python3
# GPIO.cleanup()
from threading import Thread
import RPi.GPIO as GPIO
from gpiozero import AngularServo
GPIO.setmode(GPIO.BCM)

from flask import Flask, render_template, request
app = Flask(__name__)

in1 = 6
in2 = 12
in3 = 26
in4 = 16
enA = 20
enB = 21

def motorSetup():
    #Right Motor
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.setup(enA, GPIO.OUT)
    motor1 = GPIO.PWM(enA, 100)
    motor1.start(0)
    
    #Left Motor
    GPIO.setup(in3, GPIO.OUT)
    GPIO.setup(in4, GPIO.OUT)
    GPIO.setup(enB, GPIO.OUT)
    motor2 = GPIO.PWM(enB, 100)
    motor2.start(0)

def motorStop():
    GPIO.output(in1, False)
    GPIO.output(in2, False)
    GPIO.output(in3, False)
    GPIO.output(in4, False)
    GPIO.output(enA, False)
    GPIO.output(enB, False)

def motorForward():
    GPIO.output(in1, True)
    GPIO.output(in2, False)
    GPIO.output(in3, True)
    GPIO.output(in4, False)
    GPIO.output(enA, True)
    GPIO.output(enB, True)

def motorBackward():
    GPIO.output(in1, False)
    GPIO.output(in2, True)
    GPIO.output(in3, False)
    GPIO.output(in4, True)
    GPIO.output(enA, True)
    GPIO.output(enB, True)

def motorLeft():
    GPIO.output(in1, True)
    GPIO.output(in2, False)
    GPIO.output(in3, False)
    GPIO.output(in4, True)
    GPIO.output(enA, True)
    GPIO.output(enB, True)

def motorRight():
    GPIO.output(in1, False)
    GPIO.output(in2, True)
    GPIO.output(in3, True)
    GPIO.output(in4, False)
    GPIO.output(enA, True)
    GPIO.output(enB, True)

def servoSetup():
    GPIO.setup(27, GPIO.OUT)
    GPIO.setup(22, GPIO.OUT)
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)
    armVert = AngularServo(27, min_pulse_width=0.0006, max_pulse_width=0.0023)
    armVert.angle=90
    armClaw = AngularServo(22, min_pulse_width=0.0006, max_pulse_width=0.0023)
    armClaw.angle=90
    armHorz = AngularServo(23, min_pulse_width=0.0006, max_pulse_width=0.0023)
    armHorz.angle=0
    camNeck = AngularServo(24, min_pulse_width=0.0006, max_pulse_width=0.0023)
    camNeck.angle=0

@app.route('/', methods=['POST', 'GET'])
def index():
    return render_template('index.html')

@app.route('/<num>', methods=['POST'])
def button(num):
    if num == "1":
        return "1" + motorForward()
    if num == "2":
        return "2" + motorStop()

def startSite():
    app.run(debug=True, use_reloader=False, host='0.0.0.0')

if __name__ == '__main__':
    t = Thread(target=startSite)
    t.start()
    motorSetup()

GPIO.cleanup()
