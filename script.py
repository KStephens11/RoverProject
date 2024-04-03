# Librarys
from threading import Thread
import asyncio
import websockets
from flask import Flask, render_template, request, Response
import cv2
from time import sleep
from gpiozero import AngularServo, Motor, PWMOutputDevice
from picamera2 import Picamera2
import board
import adafruit_dht
import adafruit_mpu6050
import serial
import adafruit_gps
from ultralytics import YOLO
app = Flask(__name__)

model = YOLO('yolov9c.pt')

# Create camera
camera = Picamera2()
videoConfig = camera.create_video_configuration()
stillConfig = camera.create_still_configuration()


# Create DHT object from library
dhtDevice = adafruit_dht.DHT11(board.D4)
# Create MPU object from library
mpu = adafruit_mpu6050.MPU6050(board.I2C())
# Initialize and configure UART serial connection
uart = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=10)
# Create GPS
gps = adafruit_gps.GPS(uart, debug=False)
# Send command to set GPS NMEA output frequencies, Set to output Longitude and Latitude only
gps.send_command(b'PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Set NMEA port update rate, set to 1 second
gps.send_command(b'PMTK220,1000')

# Initialize motor outputs
motorBR = Motor(21, 26)
motorBL = Motor(20, 16)
motorTR = Motor(22, 25)
motorTL = Motor(10, 9)
# Initialize PWM outputs
pwmBR = PWMOutputDevice(13, initial_value=0, frequency=100)
pwmBL = PWMOutputDevice(8, initial_value=0, frequency=100)
pwmTL = PWMOutputDevice(11, initial_value=0, frequency=100)
pwmTR = PWMOutputDevice(27, initial_value=0, frequency=100)
# Set PWM value to 0.5 by default
pwmTR.value = 0.5
pwmTL.value = 0.5
pwmBR.value = 0.5
pwmBL.value = 0.5

# Initialize servos
ArmXServo = AngularServo(19)
ArmYServo = AngularServo(7)
ClawServo = AngularServo(6)
CamServo = AngularServo(5)

# Detach Servos immediately
ArmYServo.detach()
ArmXServo.detach()
ClawServo.detach()
CamServo.detach()

# Function to get frame from camera and process it for streaming
def getFrame():
    # Capture frame data as array
    frame = camera.capture_array()
    # Flip frame 180 degrees
    rot_frame = cv2.rotate(frame, cv2.ROTATE_180)
    # Convert frame to from BGR to RGB (Uses XBGR8888)
    rgb_frame = cv2.cvtColor(rot_frame, cv2.COLOR_BGR2RGB)
    return rgb_frame

# Function
def yieldFrame():
    while True:
        frame = getFrame()
        # Encode frame into memory buffer
        jpeg = cv2.imencode('.jpg', frame)[1]
        # Concats frames one by one and yields results
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

def processImage():
    frame = getFrame()
    results = model(frame)
    output = results[0].plot()
    jpeg = cv2.imencode('.jpg', output)[1]
    while True:
        # Concats frames one by one and yields results
        yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

# Displays video frames onto route /videoStream
@app.route('/videoStream')
def videoStream():
    # mimetype set to multipart/x-mixed-replace to indicate streaming content
    return Response(yieldFrame(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/procImage')
def procImage():
    camera.stop()
    camera.configure(stillConfig)
    camera.start()
    return Response(processImage(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route for Main Dashboard
@app.route('/')
def index():
    camera.stop()
    camera.configure(videoConfig)
    camera.start()
    return render_template('index.html')

# Route for Map
@app.route('/map')
def mapPage():
    return render_template('map.html')

# Route for Image Processing page
@app.route('/process')
def imagePage():
    return render_template('process.html')

async def handler(websocket, idk): 
    try:
        while True:
            message = await websocket.recv()
            if message:
                code, value = message.split(":")
                value = int(value)
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
                        ArmXServo.angle = value
                        sleep(0.2)
                        ArmXServo.detach()
                    case "9":
                        ArmYServo.angle = value
                        sleep(0.2)
                        ArmYServo.detach()
                    case "10":
                        ClawServo.angle = value
                        sleep(0.2)
                        ClawServo.detach()
                    case "11":
                        CamServo.angle = value
                        sleep(0.2)
                        CamServo.detach()
                    case "12":
                        value = value/10
                        pwmTR.value = value
                        pwmTL.value = value
                        pwmBR.value = value
                        pwmBL.value = value
                    case "20":
                        try:
                            temp = dhtDevice.temperature
                            humidity = dhtDevice.humidity
                            acceleration = mpu.acceleration
                            gyro = mpu.gyro
                        except (RuntimeError, OSError):
                            temp = 0
                            humidity = 0
                            acceleration = 0
                        await websocket.send("X:%.2f, Y:%.2f, Z:%.2f m/s^2" % (acceleration) + ";" + "\nX:%.2f, Y:%.2f, Z:%.2f rad/s" % (gyro) + ";" + str(temp) + ";" + str(humidity))
                    case "21":
                        gps.update()
                        if gps.latitude == None or gps.longitude == None:
                            gps.update()
                        else:
                            await websocket.send(str(gps.latitude) + "," + str(gps.longitude))
                print(message)
    except websockets.ConnectionClosedOK:
            print("CONNECTION CLOSED")
            motorTR.stop()
            motorTL.stop()
            motorBR.stop()
            motorBL.stop()


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
