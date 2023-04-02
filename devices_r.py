from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B,OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import GyroSensor, ColorSensor

left_motor_port = OUTPUT_C
right_motor_port = OUTPUT_B
lift_motor_port = OUTPUT_D

left_motor = LargeMotor(left_motor_port)
right_motor = LargeMotor(right_motor_port)
lift_motor = MediumMotor(lift_motor_port)
tank_drive = MoveTank(left_motor_port, right_motor_port)
gyro = GyroSensor(INPUT_1)
color_reader = ColorSensor(INPUT_2)