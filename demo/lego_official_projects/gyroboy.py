#!/usr/bin/env python3

import time

import ev3dev.ev3 as ev3


class Timer(object):
    def __init__(self):
        self.reset()

    @property
    def time(self):
        return time.perf_counter() - self._start_time

    def reset(self):
        self._start_time = time.perf_counter()


class GyroBoy(object):

    OFFSET_DAMPING_FACTOR = 0.0005

    def __init__(self):
        # Declare sensors
        self.color_sensor = ev3.ColorSensor(address="in1")
        self.gyro_sensor = ev3.GyroSensor(address="in2")
        self.touch_sensor = ev3.TouchSensor(address="in3")
        self.ultrasonic_sensor = ev3.UltrasonicSensor(address="in4")

        self.gyro_sensor.rate()

        # Declare motors
        self.left_motor = ev3.LargeMotor(address="outA")
        self.right_motor = ev3.LargeMotor(address="outD")
        self.arms_motor = ev3.MediumMotor(address="outC")

        # calling these here so that attribute cache is setup outside of main loop
        # otherwise the first loop takes a very long time and throws off the balancing
        self.left_motor.position
        self.left_motor.duty_cycle_sp = 0
        self.right_motor.position
        self.right_motor.duty_cycle_sp = 0

        # Declare timers
        self.timer1 = Timer()
        self.timer2 = Timer()

        # Declare variables
        self.gyro_angle = 0  # gAng
        self.state = 0  # st
        self.fell_over = False  # ok
        self.motor_sum = 0  # mSum
        self.motor_difference = 0  # mDiff
        self.motor_position = 0  # mPos
        self.motor_delta = 0  # mD
        self.motor_delta_pos_1 = 0  # mDP1
        self.motor_delta_pos_2 = 0  # mDP2
        self.motor_delta_pos_3 = 0  # mDP3
        self.motor_speed = 0  # mSpd
        self.drive_control = 0  # Cdrv
        self.loop_count = 0  # Clo
        self.power = 0  # pwr
        self.steering_control = 0  # Cstr
        self.gyro_offset = 0  # gOS
        self.integral_time = 0  # tInt
        self.gyro_speed = 0  # gSpd

    def run(self):
        # Main (M) loop
        while True:
            self.reset()
            # TODO: display sleeping eyes image
            self.calc_gyro_offset()
            self.gyro_angle = -0.25
            # TODO: play speed up sound
            # TODO: play awake sound
            self.state = 1
            # Balance (BAL) loop
            print("loop")
            while True:
                p1 = time.perf_counter()
                self.GT()
                p2 = time.perf_counter()
                start_time = self.timer1.time
                p3 = time.perf_counter()
                self.GG()
                p4 = time.perf_counter()
                self.GM()
                p5 = time.perf_counter()
                self.EQ()
                p6 = time.perf_counter()
                self.left_motor.duty_cycle_sp, self.right_motor.duty_cycle_sp = self.cntrl()
                p7 = time.perf_counter()
                self.CHK()
                p8 = time.perf_counter()
                end_time = self.timer1.time
                time.sleep(max(0, 0.005 - (end_time - start_time)))
                p9 = time.perf_counter()
                if self.fell_over:
                    break
            print("oops")
            print("integral_time:", round(self.integral_time * 1000))
            print("GT:", round((p2-p1)*1000))
            print("time:", round((p3-p2)*1000))
            print("GG:", round((p4-p3)*1000))
            print("GM:", round((p5-p4)*1000))
            print("EQ:", round((p6-p5)*1000))
            print("cntrl:", round((p7-p6)*1000))
            print("CHK:", round((p8-p7)*1000))
            print("sleep:", round((p9-p8)*1000))
            self.left_motor.stop(stop_command='hold')
            self.right_motor.stop(stop_command='hold')
            self.state = 0
            # TODO: make LEDs red, flashing
            # TODO: show knocked out eyes
            # TODO: play power down sound
            while not self.touch_sensor.is_pressed():
                time.sleep(0.01)
            while self.touch_sensor.is_pressed():
                time.sleep(0.01)
            # TODO: reset LEDs

    def stop(self):
        self.left_motor.reset()
        self.right_motor.reset()
        self.arms_motor.reset()
        # TODO: these stops are not needed once reset is fixed in motor drivers
        self.left_motor.stop(stop_command='coast')
        self.right_motor.stop(stop_command='coast')

    def reset(self):
        """Resets motors, sensors and variables. This is like the RST My Block."""
        print("reset")
        self.left_motor.reset()
        self.right_motor.reset()
        # calling run_direct here so we don't waste time repeating it in the loop
        self.left_motor.run_direct()
        self.right_motor.run_direct()
        # TODO: reset gyro sensor - probably doesn't actually make a difference though
        self.timer2.reset()
        self.motor_sum = 0
        self.motor_position = 0
        self.motor_delta = 0
        self.motor_delta_pos_1 = 0
        self.motor_delta_pos_2 = 0
        self.motor_delta_pos_3 = 0
        self.drive_control = 0
        self.loop_count = 0
        self.gyro_angle = 0
        self.fell_over = False
        self.power = 0
        self.state = 0
        self.steering_control = 0
        self.drive_control = 0  # copying mistake from official program - cDrv is initialized twice

    def calc_gyro_offset(self):
        """Calculate gyro offset to account for drift in the gyro sensor. This is like the gOS My Block"""
        print("calc_gyro_offset")
        # OSL loop
        while True:
            gyro_min = 1000  # gMn
            gyro_max = -1000  # gMx
            gyro_sum = 0  # gSum
            # gChk loop
            count = 200
            for i in range(0, count):
                # this function takes to long because it is setting the mode every iteration
                # gyro_rate = self.gyro_sensor.rate()  # gyro
                # just read the value instead
                gyro_rate = self.gyro_sensor.value(0)
                gyro_sum += gyro_rate
                if gyro_rate > gyro_max:
                    gyro_max = gyro_rate
                if gyro_rate < gyro_min:
                    gyro_min = gyro_rate
                time.sleep(0.004)
            if gyro_max - gyro_min < 2:
                break
        self.gyro_offset = gyro_sum / count
        print("gyro_offset: {}".format(self.gyro_offset))

    def GT(self):
        if self.loop_count == 0:
            self.integral_time = 0.014
            self.timer1.reset()
        else:
            self.integral_time = self.timer1.time / self.loop_count
        self.loop_count += 1

    def GG(self):
        # this function sets the mode in addition to reading the value, so it takes too long.
        # gyro_rate = self.gyro_sensor.rate()  # gyro
        # just reading the value instead
        gyro_rate = self.gyro_sensor.get_attr_int('value0')
        self.gyro_offset = self.OFFSET_DAMPING_FACTOR * gyro_rate + (1 - self.OFFSET_DAMPING_FACTOR) * self.gyro_offset
        self.gyro_speed = gyro_rate - self.gyro_offset
        self.gyro_angle += self.gyro_speed * self.integral_time

    def GM(self):
        prev_motor_sum = self.motor_sum
        left_motor_pos = self.left_motor.position
        right_motor_pos = self.right_motor.position
        self.motor_sum = right_motor_pos + left_motor_pos
        self.motor_difference = right_motor_pos - left_motor_pos
        self.motor_delta = self.motor_sum - prev_motor_sum
        self.motor_position += self.motor_delta
        avg_delta = (self.motor_delta_pos_3 + self.motor_delta_pos_2 + self.motor_delta_pos_1 + self.motor_delta) / 4
        self.motor_speed = avg_delta / self.integral_time
        self.motor_delta_pos_3 = self.motor_delta_pos_2
        self.motor_delta_pos_2 = self.motor_delta_pos_1
        self.motor_delta_pos_1 = self.motor_delta

    def EQ(self):
        self.motor_position -= self.integral_time * self.drive_control
        self.power = -0.01 * self.drive_control
        self.power += 0.08 * self.motor_speed
        self.power += 0.12 * self.motor_position
        self.power += 0.8 * self.gyro_speed
        self.power += 15 * self.gyro_angle
        if self.power > 100:
            self.power = 100
        if self.power < -100:
            self.power = -100

    def cntrl(self):
        self.motor_position -= self.drive_control * self.integral_time
        steering = self.steering_control * 0.1
        return self.power - steering, self.power + steering

    def CHK(self):
        if abs(self.power) < 100:
            self.timer2.reset()
        if self.timer2.time > 1:
            self.fell_over = True

if __name__ == '__main__':
    gyro_boy = GyroBoy()
    try:
        gyro_boy.run()
    finally:
        gyro_boy.stop()
