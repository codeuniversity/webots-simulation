import math
import time
from controller import Robot

def degree_to_radiant(degree):
    return degree * (math.pi/180)
TIME_STEP = 64
MAX_SPEED = 2.76
START_POSITION = 0
MIN_MOTOR_POSITION = -57.2958
MAX_MOTOR_POSITION = 57.2958

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

    def run(self):
        global TIME_STEP
        i = 0
        self.reset()
        move = self.tripod_walk()
        while(self.step(TIME_STEP)) != -1:
            if i > 20:
                next(move)
            i += 1

bot = MSEB()
bot.initialize()
bot.run()
