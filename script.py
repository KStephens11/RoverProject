# GPIOZERO_PIN_FACTORY=pigpio python3

from threading import Thread
from rover import *
import asyncio
import websockets
from flask import Flask, render_template, request, Response
import cv2
            
app = Flask(__name__)

def getFrame(cam):
    ret, frame = cam.read()
    if ret:
        ret, jpeg = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()
                
def videoFrames():
    camera = cv2.VideoCapture(0)
    while True:
        frame = getFrame(camera)        
        if frame == None:
            camera.release()
            camera = cv2.VideoCapture(0)
            frame = getFrame(camera)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
        else:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
            
@app.route('/videoStream')
def videoStream():
    return Response(videoFrames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html')

async def handler(websocket, idk):
    state = True
    while True:
        message = await websocket.recv()
        if message == "1" and state:
            lastMessage = message
            state = False
            motorStop()
        if message == "2"and state:
            lastMessage = message
            state = False
            motorForward()
        if message == "3" and state:
            lastMessage = message
            state = False
            motorBackward()
        if message == "4" and state:
            lastMessage = message
            state = False
            motorLeft()
        if message == "5" and state:
            lastMessage = message
            state = False
            motorRight()
        if message != lastMessage:
            lastMessage = message
            state=True
        print(message)

async def main():
    async with websockets.serve(handler, "192.168.1.5", 8080):
        await asyncio.Future()

def startSite():
    app.run(debug=True, threaded=True, use_reloader=False, host='0.0.0.0')

if __name__ == "__main__":
    motorSetup()
    t = Thread(target=startSite)
    t.start()
    asyncio.run(main())

GPIO.cleanup()
camera.release()
