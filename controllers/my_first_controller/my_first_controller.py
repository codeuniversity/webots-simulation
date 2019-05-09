from controller import Robot, Motor

# time in [ms] of a simulation step
TIME_STEP = 64

#MAX_SPEED = 6.28

def move_leg(body_motor, leg_motor, foot_motor, degree):
    leg_motor.setPosition() 

# create the Robot instance.
robot = Robot()

# initialize devices
motor_names = ['leg_r_mid', 'foot_r_mid']
motors = []
for name in motor_names:
    motor = robot.getMotor(name)
    print(name)
    motor.setPosition(0)
    motor.setVelocity(4.76)
    motors.append(motor)
   
ps = robot.getPositionSensor('ps_body_r_mid')
ps.enable(TIME_STEP)
# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    motors[0].setPosition(0.785398)
    motors[1].setPosition(-0.785398)