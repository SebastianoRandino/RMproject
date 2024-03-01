import numpy as np
import RPi.GPIO as GPIO
import smbus2
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import busio
import time
import pickle
import board

class ExpSim:
    """
    A class to interface with the experimental setup:
    this will create a step response with a certain resistance, connecting and disconnecting those
    """
    def __init__(self,
        len_a: int,
        n_step_res: int,
        ADC_fs: int = 250,
        press_tr1: str = 'Calibration1Data',
        press_tr2: str = 'Calibration2Data'
        ):

        # Initializing time data of the simulation
        self.len_a = len_a
        self.n_step_res = n_step_res
        self.len_step_res = int(self.len_a / self.n_step_res)

        # Opening data of the pressure transducer for computing the wind speed
        self.press_tr1 = press_tr1
        self.press_tr2 = press_tr2
        with open(self.press_tr1, 'rb') as file:
            self.data = pickle.load(file)
        with open(self.press_tr2, 'rb') as file:
            self.data2 = pickle.load(file)
        self.m1, self.q1 = self.data2['fitting parameters']
        self.m2, self.q2 = self.data['fitting parameters']

        # Set up pins for acting on the transistors
        self.transPin = 17
        GPIO.setmode(GPIO.BCM)       # use bcm GPIO Numbering
        GPIO.setup(self.transPin, GPIO.OUT)   # set the transistorPin to OUTPUT mode
        GPIO.output(self.transPin, GPIO.LOW)  # make transPin output LOW level at the benning
        time.sleep(1) # Wait 3 second for transients

        # Initialize the encoder read by Arduino --- Initialize I2C connection with Arduino
        channel = 1
        self.address = 0x3F
        self.bus = smbus2.SMBus(channel)
        self.rx_bytes = [0,0,0]

        # Initialize the I2C connection with the ADC board
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ADC_fs = ADC_fs
        self.ads = ADS.ADS1115(self.i2c, gain=0.6666666666666666, data_rate=self.ADC_fs)
        self.channel1 = AnalogIn(self.ads, ADS.P0)
        self.channel2 = AnalogIn(self.ads, ADS.P1)
        self.channel3 = AnalogIn(self.ads, ADS.P2)
        self.channel4 = AnalogIn(self.ads, ADS.P3)

    def simulation(self):

        flag = True
        self.i_step = 1 

        # Initializing data to be passed
        rot_speed = np.zeros(self.len_a)
        gen_volt = np.zeros(self.len_a)
        tr1_volt = np.zeros(self.len_a)
        tr2_volt = np.zeros(self.len_a)
        temp_volt = np.zeros(self.len_a)
        time_speed = np.zeros(self.len_a)
        time_ch1 = np.zeros(self.len_a)
        time_ch2 = np.zeros(self.len_a)
        time_ch3 = np.zeros(self.len_a)
        time_ch4 = np.zeros(self.len_a)

        # For loop for the simulation
        for i in range(self.len_a):
            if i == self.i_step * self.len_step_res:
                if flag:
                    GPIO.output(self.transPin, GPIO.HIGH)
                    flag = False
                else:
                    GPIO.output(self.transPin, GPIO.LOW)
                    flag = True
                self.i_step = self.i_step + 1

            if i == 0:
                t0 = time.time()

            # Read rotational speed from Arduino 
            time_speed[i] = time.time() - t0
            try:
                self.rx_bytes = self.bus.read_i2c_block_data(self.address, 0, 4)
                value = self.rx_bytes[0] + (self.rx_bytes[1] << 8) + (self.rx_bytes[2] << 16)
            except:
                print('Reading error')
            v = float(value) / 1000
            rot_speed[i] = v

            # Read values from the ADC board
            time_ch1[i] = time.time() - t0
            gen_volt[i] = self.channel1.voltage
            time_ch2[i] = time.time() - t0
            tr1_volt[i] = self.channel2.voltage * 5
            time_ch3[i] = time.time() - t0
            tr2_volt[i] = self.channel3.voltage * 5
            time_ch4[i] = time.time() - t0
            temp_volt[i] = self.channel4.voltage

        for i in range(self.len_a):
            if i == 0:
                continue
            if abs(rot_speed[i] - rot_speed[i-1]) > 100:
                rot_speed[i] = rot_speed[i-1]

        rho = 101300 / ((np.mean(temp_volt) * 100 + 273.15) * 287)

        wind_speed1 = np.sqrt(2 * (self.m1 * tr1_volt + self.q1) / rho)
        wind_speed2 = np.sqrt(2 * (self.m2 * tr2_volt + self.q2) / rho)

        return rot_speed, gen_volt, wind_speed1, wind_speed2, rho , time_speed , time_ch1, time_ch2, time_ch3

		
	
        
        
        

		
		 
        
        
	    
	    

		
