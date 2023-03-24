from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B,OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import GyroSensor

left_motor_port = OUTPUT_C
right_motor_port = OUTPUT_B

left_motor = LargeMotor(left_motor_port)
right_motor = LargeMotor(right_motor_port)
tank_drive = MoveTank(left_motor_port, right_motor_port)
gyro = GyroSensor(INPUT_1)