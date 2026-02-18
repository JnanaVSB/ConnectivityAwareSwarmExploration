#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  8 13:11:02 2025

@author: vedasaipolisetty
"""

from controller import Robot
import random
import math

TIME_STEP = 32
MAX_SPEED = 6.28

robot = Robot()

# Enable proximity sensors
ps_names = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
sensors = []
for name in ps_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    sensors.append(sensor)

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Small random bias to avoid identical motion
bias = (random.random() - 0.5) * 0.5

def compute_avoidance():
    values = [s.getValue() for s in sensors]

    front = values[0] > 80 or values[7] > 80
    left  = values[5] > 80 or values[6] > 80
    right = values[1] > 80 or values[2] > 80

    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    if front:
        left_speed  = -MAX_SPEED
        right_speed =  MAX_SPEED
    elif left:
        left_speed  =  MAX_SPEED
        right_speed =  MAX_SPEED * 0.3
    elif right:
        left_speed  =  MAX_SPEED * 0.3
        right_speed =  MAX_SPEED

    return left_speed + bias, right_speed - bias

while robot.step(TIME_STEP) != -1:
    l, r = compute_avoidance()
    left_motor.setVelocity(l)
    right_motor.setVelocity(r)
