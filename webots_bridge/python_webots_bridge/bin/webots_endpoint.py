#!/usr/bin/env python3

import sys
import threading

from webots_bridge import connection, messages
from webots_bridge.devices import devices

from controller import Robot

host, port = sys.argv[1:]
port = int(port)

robot = Robot()
timeStep = int(robot.getBasicTimeStep())

# get references to our devices
left_wheel = robot.getDevice("left wheel")
right_wheel = robot.getDevice("right wheel")
camera = robot.getDevice("camera")

# turn on velocity control for both motors
for wheel in (left_wheel, right_wheel):
    wheel.setPosition(float('inf'))  
    wheel.setVelocity(0)
    
camera.enable(timeStep)

def create_motor_setter(motor):
    def f(msg):
        motor.setVelocity(msg.velocity)
    return f

conn = connection.connect(host, port)

# set up the reading side
handlers = {
    devices["LEFT_MOTOR"]: create_motor_setter(left_wheel),
    devices["RIGHT_MOTOR"]: create_motor_setter(right_wheel)
}
def receiver():
    while not conn.closed():
        device, message = conn.receive()
        handlers[device](message)
t = threading.Thread(target=receiver, daemon=True)
t.start()

# send over a CameraInfo
cam_info = messages.CameraInfo(
	width=camera.getWidth(),
	height=camera.getHeight()
)
conn.send(devices["CAMERA"], cam_info)

frame = 0
while robot.step(timeStep) != -1:
    frame += 1
    print("frame", frame)
    img = camera.getImage()
    conn.send(devices["CAMERA"], messages.CameraImage(image=img))