# GPIOZERO_PIN_FACTORY=pigpio python3
# GPIO.cleanup()
from threading import Thread
from rover import *

import asyncio
import websockets

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


if __name__ == "__main__":
    motorSetup()
    asyncio.run(main())
    GPIO.cleanup()
