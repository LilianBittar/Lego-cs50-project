#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from threading import Thread
import random

class Snake:

    def __init__(self, ir_sensor_port: Port, ir_beacon_channel: int, driving_motor_port: Port, steering_motor_port: Port, striking_motor_port: Port):
        self.ev3_brick = EV3Brick()
        self.ir_sensor = InfraredSensor(ir_sensor_port)
        self.ir_beacon_channel = ir_beacon_channel
        self.driving_motor = Motor(driving_motor_port, Direction.CLOCKWISE)
        self.steering_motor = Motor(steering_motor_port, Direction.CLOCKWISE)
        self.striking_motor = Motor(striking_motor_port, Direction.CLOCKWISE)

        self.behaviour_state = None

        self.is_auto_moving = False
        self.is_auto_wiggling = False
        self.is_auto_bitting = False

        self.moving_direction = 0

        self.moving_thread = Thread(target=self.move_behaviour).start()
        self.wiggling_thread = Thread(target=self.wiggle_behaviour).start()
        #self.bitting_thread = Thread(target=self.bitting_action_behaviour).start()

        self.steering_motor.run_target(speed=200, target_angle=0, then=Stop.COAST)

        self.striking_motor.run_time(-220, 1100)
    

    def run(self):
        self.switch_behaviour()
        self.behaviour_state.run(self)


    def switch_behaviour(self):
            wait(500)
            ir_beacons_pressed = set(self.ir_sensor.buttons(2))
            if ir_beacons_pressed == {Button.LEFT_UP}:
                if isinstance(self.behaviour_state, AutonomousState):
                    self.moving_direction = 0
                    self.is_auto_moving = False
                    self.is_auto_wiggling = False
                    self.is_auto_bitting = False
                    self.behaviour_state = ManualState()
                    print("Switched to manual mode.")
                else:
                    self.behaviour_state = AutonomousState()
                    self.is_auto_moving = True
                    self.is_auto_wiggling = False
                    self.is_auto_bitting = True
                    print("Switched to automatic mode.")
            elif ir_beacons_pressed == {Button.RIGHT_UP}:
                self.hiss()
                self.strike()


    def hiss(self):
        self.ev3_brick.speaker.play_file(file=SoundFile.SNAKE_HISS)


    def strike(self):
        self.striking_motor.run_time(220, 1100)
        self.striking_motor.run_time(-220, 1100)


    def move_behaviour(self):
        while True:
            if self.is_auto_moving:
                if self.moving_direction == 1:  
                    self.driving_motor.run(random.random() * 200 + 300)
                elif self.moving_direction == 0:
                    self.driving_motor.stop()
                else:
                    self.driving_motor.run(-(random.random() * 200 + 300))


    def wiggle_behaviour(self):
        while True:
            if self.is_auto_wiggling:
                wait(3000)
                self.steering_motor.run_until_stalled(random.random() * 100 + 70, then=Stop.HOLD)
                wait(1000)
                self.steering_motor.run_until_stalled(-(random.random() * 100 + 70), then=Stop.HOLD)
                wait(1000)


    def bitting_action_behaviour(self):
        queue = []

        # for i in range(5):
        #     queue.append(self.ir_sensor.distance())
        
        while True:
            dist = 0
            avg = 0
            for value in queue:
                avg += value
            avg /= len(queue)
            if abs(avg - dist) > 40:
                self.strike()
                self.hiss()
                queue.clear()
                for i in range(10):
                    queue.append(self.ir_sensor.distance())
            else:
                queue.pop(0)
                queue.append(value)
             

class ManualState:
    def run(self, snake: Snake):
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

        snake.moving_direction = 1
               
        wait(500)
        if snake.ir_sensor.distance() <= 50:
            snake.moving_direction = 0
            snake.is_auto_wiggling = False
            snake.steering_motor.run_until_stalled(-200, then=Stop.HOLD)
            left_distance = snake.ir_sensor.distance()
            snake.steering_motor.run_until_stalled(200, then=Stop.HOLD)
            right_distance = snake.ir_sensor.distance()
            if right_distance < 40 and left_distance < 40:
                snake.moving_direction = -1
                wait(5000)
                snake.moving_direction = 0
            if left_distance > right_distance:
                snake.steering_motor.run_until_stalled(-200, then=Stop.HOLD)
            snake.moving_direction = 1
        else:
            snake.is_auto_wiggling = True
                
                
snake = Snake(Port.S4, 1, Port.A, Port.C, Port.B)
snake.behaviour_state = AutonomousState()
while True:
    snake.run()