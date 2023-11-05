# GPIOZERO_PIN_FACTORY=pigpio python3
# GPIO.cleanup()
from threading import Thread
from rover import *

from flask import Flask, render_template, request
app = Flask(__name__)

@app.route('/', methods=['POST', 'GET'])
def index():
    return render_template('index.html')

@app.route('/<num>', methods=['POST'])
def button(num):
    if num == "1":
        return "1"
    if num == "2":
        return "2"

def startSite():
    app.run(debug=True, use_reloader=False, host='0.0.0.0')

if __name__ == '__main__':
    t = Thread(target=startSite)
    t.start()
    motorSetup()

GPIO.cleanup()
