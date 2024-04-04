# Libraries
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

# Object detection model
model = YOLO('yolov9c.pt')

# Create camera
camera = Picamera2()
# Configurations for camera
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

# Reset servos positions and detach immediately
ArmXServo.angle = 0
sleep(0.2)
ArmXServo.detach()

ArmYServo.angle = 0
sleep(0.2)
ArmYServo.detach()

ClawServo.angle = 0
sleep(0.2)
ClawServo.detach()

CamServo.angle = 0
sleep(0.2)
CamServo.detach()


# Function to get frame from camera then flip and covert it to RGB
def getFrame():
    # Capture frame data as array
    frame = camera.capture_array()
    # Flip frame 180 degrees
    rot_frame = cv2.rotate(frame, cv2.ROTATE_180)
    # Convert frame to from BGR to RGB (Uses XBGR8888)
    rgb_frame = cv2.cvtColor(rot_frame, cv2.COLOR_BGR2RGB)
    # Return frame
    return rgb_frame

# Function for streaming video, Generator
def yieldFrame():
    while True:
        # Get frame from camera
        frame = getFrame()
        # Encode frame as jpeg
        jpeg = cv2.imencode('.jpg', frame)[1]
        # Concats frames one by one and yields results as bytes
        # First String represents start of frame and last string represents end of frame
        # Content type is set to jpeg
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

# Function for object recogntion
def processImage():
    # Get frame from camera
    frame = getFrame()
    # Insert frame into object recognition model(yoloV9c) and save into results
    results = model(frame)
    # Set output as the returned annotated frame
    output = results[0].plot()
    # Encode frame as jpeg
    jpeg = cv2.imencode('.jpg', output)[1]
    # Return jpeg as bytes
    return (jpeg.tobytes())

# Displays video frames onto route /videoStream
@app.route('/videoStream')
def videoStream():
    # mimetype set to multipart/x-mixed-replace to indicate streaming content, displays stream
    return Response(yieldFrame(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/procImage')
def procImage():
    # Stop and start camera to change configuration for high resolution still images
    camera.stop()
    camera.configure(stillConfig)
    camera.start()
    # mimetype set to image/jpeg and display jpeg
    return Response(processImage(), mimetype='image/jpeg')

# Route for Main Dashboard
@app.route('/')
def index():
    # Stop and start camera to change configuration for lower resolution video
    camera.stop()
    camera.configure(videoConfig)
    camera.start()
    # Return index page
    return render_template('index.html')

# Route for Map
@app.route('/map')
def mapPage():
    # Return map page
    return render_template('map.html')

# Route for Image Processing page
@app.route('/process')
def imagePage():
    # return processing page
    return render_template('process.html')

# Handler for websockets, uses asynchronous function
async def handler(websocket):
    # Try and except for when the websocket closes
    try:
        # While Loop for message handling
        while True:
            # set message to recieved data from websocket
            message = await websocket.recv()
            # If there is a message
            if message:
                # Split the message into two using :
                code, value = message.split(":")
                # set value as an integer
                value = int(value)
                # Switch that compares code
                match code:
                    # Case to stop rover
                    case "1":
                        motorTR.stop()
                        motorTL.stop()
                        motorBR.stop()
                        motorBL.stop()
                    # Case to move rover forward
                    case "2":
                        motorTR.forward()
                        motorTL.forward()
                        motorBR.forward()
                        motorBL.forward()
                    # Case to move rover backward
                    case "3":
                        motorTR.backward()
                        motorTL.backward()
                        motorBR.backward()
                        motorBL.backward()
                    # Case to move rover left
                    case "4":
                        motorTR.forward()
                        motorTL.backward()
                        motorBR.backward()
                        motorBL.forward()
                    # Case to move rover right
                    case "5":
                        motorTR.backward()
                        motorTL.forward()
                        motorBR.forward()
                        motorBL.backward()
                    # Case to turn rover right
                    case "6":
                        motorTR.backward()
                        motorTL.forward()
                        motorBR.backward()
                        motorBL.forward()
                    # Case to turn rover left
                    case "7":
                        motorTR.forward()
                        motorTL.backward()
                        motorBR.forward()
                        motorBL.backward()
                    # Case to move arm on the x axis
                    case "8":
                        ArmXServo.angle = value*-1
                        sleep(0.2)
                        ArmXServo.detach()
                    # Case to move arm on the y axis
                    case "9":
                        ArmYServo.angle = value
                        sleep(0.2)
                        ArmYServo.detach()
                    # Case to open and close claw
                    case "10":
                        ClawServo.angle = value
                        sleep(0.2)
                        ClawServo.detach()
                    # Case to move camera
                    case "11":
                        CamServo.angle = value
                        sleep(0.2)
                        CamServo.detach()
                    # Case to change rover speed
                    case "12":
                        value = value/10
                        pwmTR.value = value
                        pwmTL.value = value
                        pwmBR.value = value
                        pwmBL.value = value
                    # Case to get and send sensor data
                    case "20":
                        try:
                            # Get data
                            temp = dhtDevice.temperature
                            humidity = dhtDevice.humidity
                            acceleration = mpu.acceleration
                            gyro = mpu.gyro
                        # If there has been an error getting the data
                        except (RuntimeError, OSError):
                            # Set all to zero
                            temp = 0
                            humidity = 0
                            acceleration = (0, 0, 0)
                            gyro = (0, 0, 0)
                        # Send all data
                        await websocket.send("X:%.2f, Y:%.2f, Z:%.2f m/s^2" % (acceleration) + ";" + "\nX:%.2f, Y:%.2f, Z:%.2f rad/s" % (gyro) + ";" + str(temp) + ";" + str(humidity))
                    # Case to update and send GPS data
                    case "21":
                        # Update GPS
                        gps.update()
                        # Check if gps has returned coordinates
                        if gps.latitude == None or gps.longitude == None:
                            # If not print warning
                            print("GPS NO DATA")
                        else:
                            # Send GPS latitude and longitude
                            await websocket.send(str(gps.latitude) + "," + str(gps.longitude))
                print(message)
    except websockets.ConnectionClosedOK:
            # Stops motors when connection closes
            print("CONNECTION CLOSED")
            motorTR.stop()
            motorTL.stop()
            motorBR.stop()
            motorBL.stop()

# Function for websocket
async def main():
    # Create websokcet server passing the function handler and the ip and port to use
    async with websockets.serve(handler, "192.168.0.174", 8080):
        # Waits forever
        await asyncio.Future()
# Function to start flask
def startSite():
    # Set as threaded, disable reloader and set host to 0.0.0.0 to bind on all interfaces, making it externally accessible
    app.run(debug=True, threaded=True, use_reloader=False, host='0.0.0.0')

# Run if the script is executed directly
if __name__ == "__main__":
    # Create thread for website
    t = Thread(target=startSite)
    # Start thread
    t.start()
    # Start async function main which is for websockets
    asyncio.run(main())