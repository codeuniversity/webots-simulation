import math
import time
from controller import Robot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import threading
import asyncio
import websockets

async def point_cloud_socket(websocket, path):
    global point_cloud
    while True:
        await websocket.send(str(point_cloud))
        await asyncio.sleep(1)

start_server = websockets.serve(point_cloud_socket, 'localhost', 4000)

asyncio.get_event_loop().run_until_complete(start_server)
pc_thread = threading.Thread(target=asyncio.get_event_loop().run_forever)
pc_thread.start()


def degree_to_radiant(degree):
    return degree * (math.pi/180)

TIME_STEP = 128
MAX_SPEED = 2.76
START_POSITION = 0
MIN_MOTOR_POSITION = -57.2958
MAX_MOTOR_POSITION = 57.2958

point_cloud = []

class MSEB(Robot):

    def initialize(self):
        global START_POSITION
        global MAX_SPEED
        global TIME_STEP
        # Naming Schemes for Webots-motors
        components = ['body', 'leg', 'foot']
        sides = ['r','l']
        leg_locations = ['top', 'mid', 'bot']

        self.leg_motors = {}
        for s in sides:
            for l in leg_locations:
                leg_name = '_'.join([s, l])
                self.leg_motors[leg_name] = {}
                for c in components:
                    motor_name = c + '_' + leg_name
                    motor = self.getMotor(motor_name)
                    motor.setPosition(START_POSITION)
                    motor.setVelocity(MAX_SPEED)
                    self.leg_motors[leg_name][c] = motor

        self.sensors = {}

        sensor = self.getDistanceSensor('lidar')
        sensor.enable(TIME_STEP)
        self.sensors['lidar'] = sensor
        sensor = self.getPositionSensor('lidar_base_position_sensor')
        sensor.enable(TIME_STEP)
        self.sensors['lidar_base_position_sensor'] = sensor
        sensor = self.getPositionSensor('lidar_vertical_position_sensor')
        sensor.enable(TIME_STEP)
        self.sensors['lidar_vertical_position_sensor'] = sensor

        self.other_motors = {}
        motor = self.getMotor('lidar_base_motor')
        motor.setPosition(float("inf"))
        motor.setVelocity(6)
        self.other_motors['lidar_base_motor'] = motor
        motor = self.getMotor('lidar_motor_vertical')
        motor.setVelocity(6)
        motor.setPosition(0)
        self.other_motors['lidar_motor_vertical'] = motor

        self.position = [0,0,0]

    def set_knee_joint(self, leg_name, degree):
        global MIN_MOTOR_POSITION
        global MAX_MOTOR_POSITION
        if degree > MAX_MOTOR_POSITION:
            #self.leg_motors[leg_name]['leg'].setPosition(degree_to_radiant(MAX_MOTOR_POSITION))
            self.leg_motors[leg_name]['leg'].setPosition(1)
        elif degree < MIN_MOTOR_POSITION:
            self.leg_motors[leg_name]['leg'].setPosition(degree_to_radiant(MIN_MOTOR_POSITION))
        else:
            self.leg_motors[leg_name]['leg'].setPosition(degree_to_radiant(degree))

    def set_ankle(self, leg_name, degree):
        global MIN_MOTOR_POSITION
        global MAX_MOTOR_POSITION
        if degree > MAX_MOTOR_POSITION:
            self.leg_motors[leg_name]['foot'].setPosition(degree_to_radiant(MAX_MOTOR_POSITION))
        elif degree < MIN_MOTOR_POSITION:
            self.leg_motors[leg_name]['foot'].setPosition(degree_to_radiant(MIN_MOTOR_POSITION))
        else:
            self.leg_motors[leg_name]['foot'].setPosition(degree_to_radiant(degree))

    def set_hip_joint(self, leg_name, degree):
        global MIN_MOTOR_POSITION
        global MAX_MOTOR_POSITION
        if degree > MAX_MOTOR_POSITION:
            self.leg_motors[leg_name]['body'].setPosition(degree_to_radiant(MAX_MOTOR_POSITION))
        elif degree < MIN_MOTOR_POSITION:
            self.leg_motors[leg_name]['body'].setPosition(degree_to_radiant(MIN_MOTOR_POSITION))
        else:
            self.leg_motors[leg_name]['body'].setPosition(degree_to_radiant(degree))

    def reset(self):
        for leg_name in self.leg_motors:
            self.set_hip_joint(leg_name, 0)
            self.set_knee_joint(leg_name, 45)
            self.set_ankle(leg_name, -45)

    def tripod_walk(self):
        LEG_RAISING_ANGLE = 60
        LEG_STANDING_ANGLE = 45
        HIP_ROTATION_ANGLE = 30
        while True:
            self.set_knee_joint('l_mid', LEG_RAISING_ANGLE)
            self.set_knee_joint('r_top', LEG_RAISING_ANGLE)
            self.set_knee_joint('r_bot', LEG_RAISING_ANGLE)
            yield 1
            yield 1
            yield 1

            self.set_hip_joint('l_mid', HIP_ROTATION_ANGLE)
            self.set_hip_joint('r_top', HIP_ROTATION_ANGLE)
            self.set_hip_joint('r_bot', HIP_ROTATION_ANGLE)
            self.set_hip_joint('l_top', -HIP_ROTATION_ANGLE)
            self.set_hip_joint('l_bot', -HIP_ROTATION_ANGLE)
            self.set_hip_joint('r_mid', -HIP_ROTATION_ANGLE)
            yield 1
            yield 1
            yield 1
            yield 1
            yield 1
            yield 1

            self.set_knee_joint('l_mid', LEG_STANDING_ANGLE)
            self.set_knee_joint('r_top', LEG_STANDING_ANGLE)
            self.set_knee_joint('r_bot', LEG_STANDING_ANGLE)
            yield 1
            yield 1
            yield 1

            self.set_knee_joint('r_mid', LEG_RAISING_ANGLE)
            self.set_knee_joint('l_top', LEG_RAISING_ANGLE)
            self.set_knee_joint('l_bot', LEG_RAISING_ANGLE)
            yield 1
            yield 1
            yield 1

            self.set_hip_joint('r_mid', HIP_ROTATION_ANGLE)
            self.set_hip_joint('l_top', HIP_ROTATION_ANGLE)
            self.set_hip_joint('l_bot', HIP_ROTATION_ANGLE)
            self.set_hip_joint('r_top', -HIP_ROTATION_ANGLE)
            self.set_hip_joint('r_bot', -HIP_ROTATION_ANGLE)
            self.set_hip_joint('l_mid', -HIP_ROTATION_ANGLE)
            yield 1
            yield 1
            yield 1
            yield 1
            yield 1
            yield 1

            self.set_knee_joint('r_mid', LEG_STANDING_ANGLE)
            self.set_knee_joint('l_top', LEG_STANDING_ANGLE)
            self.set_knee_joint('l_bot', LEG_STANDING_ANGLE)
            yield 1
            yield 1
            yield 1

    def update_map(self):
        global point_cloud
        length = self.sensors['lidar'].getValue()
        if length < 3:
            yaw = self.sensors['lidar_base_position_sensor'].getValue()
            pitch = self.sensors['lidar_vertical_position_sensor'].getValue()
            point = []
            point.append(math.sin(yaw) * math.cos(pitch) * length )
            point.append(math.sin(pitch) * length)
            point.append(math.cos(yaw) * math.cos(pitch) * length )
            point_cloud.append(point)

    def update_lidar_position(self):
        yaw = self.sensors['lidar_base_position_sensor'].getValue()
        if yaw > 2*math.pi * self.lidar_rotations:
            self.lidar_rotations += 1
            old_vertical_position = self.sensors['lidar_vertical_position_sensor'].getValue()
            print(old_vertical_position)
            if self.wait_for_motor and old_vertical_position == 0:
                self.wait_for_motor = False
            if old_vertical_position >= 0.2:
                self.wait_for_motor = True
                self.other_motors['lidar_motor_vertical'].setPosition(0)
            elif not self.wait_for_motor:
                self.other_motors['lidar_motor_vertical'].setPosition(old_vertical_position + 0.01)

    def run(self):
        global TIME_STEP
        global pc_thread
        i = 0
        self.reset()
        move = self.tripod_walk()
        self.lidar_rotations = 0
        self.wait_for_motor = False
        while(self.step(TIME_STEP)) != -1:
            if i > 20:
                next(move)
            self.update_map()
            self.update_lidar_position()
            i += 1
            print(point_cloud)
        pc_thread.join(0.3)


bot = MSEB()
bot.initialize()
bot.run()
