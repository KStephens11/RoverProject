from threading import Thread
import asyncio
import websockets
from flask import Flask, render_template, request, Response
import cv2
import time
from gpiozero import AngularServo, Motor, PWMOutputDevice
from picamera2 import Picamera2
import board
import adafruit_dht
import adafruit_mpu6050

app = Flask(__name__)

camera = Picamera2()
camera.start()

dhtDevice = adafruit_dht.DHT11(board.D4)
mpu = adafruit_mpu6050.MPU6050(board.I2C())

motorBR = Motor(21, 26)
motorBL = Motor(20, 16)
motorTR = Motor(22, 25)
motorTL = Motor(10, 9)
pwmBR = PWMOutputDevice(13, initial_value=0, frequency=100)
pwmBL = PWMOutputDevice(12, initial_value=0, frequency=100)
pwmTL = PWMOutputDevice(11, initial_value=0, frequency=100)
pwmTR = PWMOutputDevice(27, initial_value=0, frequency=100)

BRservo = AngularServo(19)
TRservo = AngularServo(6)
BLservo = AngularServo(5)
TLservo = AngularServo(7)

pwmTR.value = 0.5
pwmTL.value = 0.5
pwmBR.value = 0.5
pwmBL.value = 0.5


# Function to get frame from camera and return as byte
def getPiFrame():
    # Capture frame data as array
    frame = camera.capture_array()
    # Flip frame 180 degrees
    rot_frame = cv2.rotate(frame, cv2.ROTATE_180)
    # Convert frame to from BGR to RGB (Uses XBGR8888)
    rgb_frame = cv2.cvtColor(rot_frame, cv2.COLOR_BGR2RGB)
    # Encode frame into memory buffer
    jpeg = cv2.imencode('.jpg', rgb_frame)[1]
    # Convert frame to bytes
    return jpeg.tobytes()

def videoFrames():
    while True:
        frame = getPiFrame()
        #concats frames one by one and shows result
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/videoStream')
def videoStream():
    return Response(videoFrames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/map')
def mapPage():
    return render_template('map.html')

async def handler(websocket, idk): 
    try:
        while True:
            message = await websocket.recv()
            if message:
                code, value = message.split(":")
                match code:
                    case "1":
                        motorTR.stop()
                        motorTL.stop()
                        motorBR.stop()
                        motorBL.stop()
                    case "2":
                        motorTR.forward()
                        motorTL.forward()
                        motorBR.forward()
                        motorBL.forward()
                    case "3":
                        motorTR.backward()
                        motorTL.backward()
                        motorBR.backward()
                        motorBL.backward()
                    case "4":
                        motorTR.forward()
                        motorTL.backward()
                        motorBR.backward()
                        motorBL.forward()
                    case "5":
                        motorTR.backward()
                        motorTL.forward()
                        motorBR.forward()
                        motorBL.backward()
                    case "6":
                        motorTR.backward()
                        motorTL.forward()
                        motorBR.backward()
                        motorBL.forward()
                    case "7":
                        motorTR.forward()
                        motorTL.backward()
                        motorBR.forward()
                        motorBL.backward()
                    case "8":
                        BRservo.angle(value)
                        sleep(0.5)
                        BRservo.detach()
                    case "9":
                        BLservo.angle(value)
                        sleep(0.5)
                        BLservo.detach()
                    case "10":
                        TLservo.angle(value)
                        sleep(0.5)
                        TLservo.detach()
                    case "11":
                        TRservo.angle(value)
                        sleep(0.5)
                        TRservo.detach()
                    case "12":
                        value = value/10
                        pwmTR.value = value
                        pwmTL.value = value
                        pwmBR.value = value
                        pwmBL.value = value
                    case "20":
                        await websocket.send("X:%.2f, Y:%.2f, Z:%.2f m/s^2" % (mpu.acceleration) + ";" + "\nX:%.2f, Y:%.2f, Z:%.2f rad/s" % (mpu.gyro) + ";" + str(dhtDevice.temperature) + ";" + str(dhtDevice.humidity))
                        
                print(message)
    except websockets.ConnectionClosedOK:
            print("CONNECTION CLOSED")


async def main():
    async with websockets.serve(handler, "192.168.0.174", 8080):
        await asyncio.Future()

def startSite():
    app.run(debug=True, threaded=True, use_reloader=False, host='0.0.0.0')

if __name__ == "__main__":
    t = Thread(target=startSite)
    t.start()
    asyncio.run(main())

camera.release()
