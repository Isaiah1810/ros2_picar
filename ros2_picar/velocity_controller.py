import rpyc
import os
import serial
from math import pi
class DriveInterface:
    def __init__(self, pwm_freq=20000000, pwm_pin=0, 
                 forward_pin=15, backward_pin=16):
        self.connection = rpyc.connect("localhost", 18811)
        self.pwm_freq = pwm_freq
        self.max_vel = 1 #meters per second. This value still needs to be measured
        self.f_pin = forward_pin
        self.r_pin = backward_pin
        self.pwm_pin = pwm_pin
    def get_duty_cycle(self, duty):
        if (duty > 1):
            print("Duty cycle greater than 1, setting to 1")
            return self.pwm_freq
        if (duty < 0):
            print("Duty cycle less than 0, setting to 0")
            return 0
        return self.pwm_freq*duty
    def write_dir(self, dir):
        if (dir == 1):
            os.system(f"gpio write {self.f_pin} 1")
            os.system(f"gpio write {self.r_pin} 0")
            return
        else:
            os.system(f"gpio write {self.f_pin} 0")
            os.system(f"gpio write {self.r_pin} 1")
            return
    def set_motor_velocity(self, vel):
        vel_dir = 1 if vel >= 0 else -1
        abs_vel = abs(vel)
        if (abs_vel > self.max_vel):
            print("Velocity Command Greater than max, setting to max")
            self.connection.root.pwm(self.pwm_pin, self.pwm_freq, self.pwm_freq)#Change this first one depending on pins used
            self.write_dir(vel_dir)
            return
        dir_ratio = vel/self.max_vel
        duty = self.get_duty_cycle(dir_ratio)
        self.connection.root.pwm(self.pwm_pin, self.pwm_freq, duty)
        return

class SteeringInterface:
    def __init__(self, port='/dev/ttyS3', baud_rate=9600):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = serial.Serial(port=self.port,
                                 baudrate=self.baud_rate,
                                 timeout=5.0,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE)

    def set_freq(self, freq):
        if(freq > 999): return 1
        self.ser.write(f"F{freq}".encode('ASCII'))
    
    def set_duty(self, duty):
        self.ser.write(f"F{str(duty).zfill(3)}".encode('ASCII'))

    def read(self):
        self.ser.write("read".encode("ASCII"))
        msg = self.ser.read_until("%".encode("ASCII"))
        print(str(msg))