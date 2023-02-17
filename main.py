#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()
angle_sensor = Motor(Port.C)
light_sensor = ColorSensor(Port.S4)
obstacle_sensor = UltrasonicSensor(Port.S1)
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
head_motor = Motor(Port.C)

robot = DriveBase(left_motor, right_motor, wheel_diameter=35, axle_track=110)

ev3.speaker.beep()

def head_rotation(liste_distance, luminosite, angle):
    head_motor.run_angle(100, 20)
    time.sleep(0.6)
    liste_distance.append(obstacle_sensor.distance())
    angle.append(angle_sensor.angle())
    luminosite.append(light_sensor.ambient())

def check_obstacle():
    liste_distance = []
    luminosite = []
    angle=[]
    head_motor.run_angle(100, -80)
    for i in range(8):
        head_rotation(liste_distance, luminosite,angle)
    return (liste_distance,luminosite,angle)
    
    head_motor.run_angle(100, -80)



def decide_rotation_lumiere():
    lumiere=0
    angle_de_rotation_lumiere=0
    
    for y in range(len(luminosite))
        if luminosite[y] > lumiere:
            lumiere=luminosite[y]
            angle_de_rotation_lumiere=angle[y]    
    return angle_de_rotation_lumiere

def decide_rotation_distance():
    distance=0
    angle_de_rotation_distance=0
    for i in range(len(liste_distance))
        if liste_distance[i] > distance:
            distance = liste_distance[i]
            angle_de_rotation_distance=angle[i]
    return angle_de_rotation_distance=angle

def decide_rotation_angle():
    valeur_angle_rotation=dicide_rotation_lumiere()



def move():
    robot.straight(500)
    while True:
        if obstacle_sensor.distance() < 200:
            robot.stop()
            check_obstacle()
            robot.straight(500)




    

