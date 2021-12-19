#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from threading import Thread

class Snake:

    def __init__(self, ir_sensor_port: Port, ir_beacon_channel: int, driving_motor_port: Port, steering_motor_port: Port, striking_motor_port: Port):
        self.ev3_brick = EV3Brick()
        self.ir_sensor = InfraredSensor(ir_sensor_port)
        self.ir_beacon_channel = ir_beacon_channel
        self.driving_motor = Motor(driving_motor_port, Direction.CLOCKWISE)
        self.steering_motor = Motor(steering_motor_port, Direction.CLOCKWISE)
        self.striking_motor = Motor(striking_motor_port, Direction.CLOCKWISE)

        self.behaviour_state = AutonomousState()

        self.to_wiggle = True

        self.left_end = self.steering_motor.run_until_stalled(-200, then=Stop.HOLD)
        self.right_end = self.steering_motor.run_until_stalled(200, then=Stop.HOLD)

        print(self.left_end)
        print(self.right_end)

        self.limit = (self.right_end - self.left_end) // 2

        print(self.limit)
        self.steering_motor.reset_angle(self.limit)
        self.steering_motor.run_target(speed=200, target_angle=0, then=Stop.COAST)


    def run(self):
        self.behaviour_state.run(self)


    def hiss(self):
        self.ev3_brick.speaker.play_file(file=SoundFile.SNAKE_HISS)

    def strike(self):
        self.striking_motor.run_time(220, 1100)
        self.striking_motor.run_time(-220, 1100)

    def wiggle_behaviour(self):
        while True:
            if self.to_wiggle:
                self.steering_motor.run_until_stalled(-200, then=Stop.HOLD)
                wait(1000)
                self.steering_motor.run_until_stalled(200, then=Stop.HOLD)
                wait(1000)

    def bitting_action_behaviour(self):
        queue = []

        print("gg")
        for i in range(5):
            queue.append(self.ir_sensor.distance())
        
        while True:
            dist = self.ir_sensor.distance()
            avg = 0
            for value in queue:
                avg += value
            avg /= len(queue)

            print(avg)
            print(queue)

            if abs(avg - dist) > 70:
                strike(self)
                hiss(self)
                queue.clear()
                for i in range(10):
                    queue.append(self.ir_sensor.distance())
            else:
                queue.pop(0)
                queue.append(value)
            


class ManualState:
    def run(self, snake: Snake):
        while True:
            ir_beacons_pressed = set(snake.ir_sensor.buttons(snake.ir_beacon_channel))

            if ir_beacons_pressed == {Button.LEFT_UP, Button.RIGHT_UP}:
                snake.driving_motor.run(1000)

            elif ir_beacons_pressed == {Button.LEFT_DOWN, Button.RIGHT_DOWN}:
                snake.driving_motor.run(-1000)

            elif ir_beacons_pressed == {Button.LEFT_UP}:
                snake.steering_motor.run(-500)
                snake.driving_motor.run(1000)

            elif ir_beacons_pressed == {Button.RIGHT_UP}:
                snake.steering_motor.run(500)
                snake.driving_motor.run(1000)

            elif ir_beacons_pressed == {Button.LEFT_DOWN}:
                snake.steering_motor.run(-500)
                snake.driving_motor.run(-1000)

            elif ir_beacons_pressed == {Button.RIGHT_DOWN}:
                snake.steering_motor.run(500)
                snake.driving_motor.run(-1000)

            else:
                snake.steering_motor.hold()
                snake.driving_motor.stop()


class AutonomousState:
    def run(self, snake: Snake):
        snake.driving_motor.run(500)
        while True:
            if snake.ir_sensor.distance() <= 50:
                snake.to_wiggle = False
                snake.steering_motor.run_until_stalled(-200, then=Stop.HOLD)
                left_distance = snake.ir_sensor.distance()    
                snake.steering_motor.run_until_stalled(200, then=Stop.HOLD)
                right_distance = snake.ir_sensor.distance()
                if left_distance < right_distance:
                    snake.driving_motor.run(500)
                else:
                    snake.steering_motor.run_until_stalled(-200, then=Stop.HOLD)
                    snake.driving_motor.run(500)   
                snake.to_wiggle = True
                
                
snake = Snake(Port.S4, 1, Port.A, Port.C, Port.B)
t = Thread(target=snake.wiggle_behaviour)
t.start()
snake.run()