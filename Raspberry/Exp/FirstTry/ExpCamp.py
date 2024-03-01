import smbus2
from time import sleep
import time
import numpy as np
import matplotlib.pyplot as plt
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import pickle 
import RPi.GPIO as GPIO
from scipy import signal

with open('Calibration1Data' , 'rb') as file:
	data = pickle.load(file)

with open('Calibration2Data' , 'rb') as file:
	data2 = pickle.load(file)
	
m2 = data['fitting parameters'][0]
q2 = data['fitting parameters'][1]

m1 = data2['fitting parameters'][0]
q1 = data2['fitting parameters'][1]

print(m1 , q1)

transistorPin = 17 # Gpio pin of the transistor

def setup():
 GPIO.setmode(GPIO.BCM)       # use PHYSICAL GPIO Numbering
 GPIO.setup(transistorPin, GPIO.OUT)   # set the transistorPin to OUTPUT mode
 GPIO.output(transistorPin, GPIO.LOW)  # make ledPin output LOW level at the benning
 sleep(3)
 print ('using pi transitor: %d'%transistorPin)

setup()

dt = 0.1
# ------- ENCODER INITIALIZATION 
# I2C channel 1 is connected to the GPIO pins
channel = 1
address = 0x3F

# Initialize I2C (SMBus)
bus = smbus2.SMBus(channel)
bus.timeout = 0.02 

rx_bytes = [0,0,0]

lenght = 2400

vel = []

# ------- ADC INITIALIZATION 
# Initialize the I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
 
# Create an ADS1115 object
ads = ADS.ADS1115(i2c , gain = 0.6666666666666666 , data_rate = 250)
# data_rate  = [8, 16, 32, 64, 128, 250, 475, 860] samples for second -- it defines the frequency (the more the noise :( )
 
# Define the analog input channel
channel = AnalogIn(ads, ADS.P0)
channel2 = AnalogIn(ads, ADS.P1)
channel3 = AnalogIn(ads, ADS.P2)

voltag = []
voltag2 = []
voltag3 = []

time_enc = []
time_adc = []

for i in range(lenght):
    if i ==0:
        t0 = time.time()
    if i == 400:
        GPIO.output(transistorPin, GPIO.HIGH)
    if i== 800:
        GPIO.output(transistorPin, GPIO.LOW)
    if i == 1200:
        GPIO.output(transistorPin, GPIO.HIGH)
    if i== 1600:
        GPIO.output(transistorPin, GPIO.LOW)
    if i == 2000:
        GPIO.output(transistorPin, GPIO.HIGH)
    #t1 = time.time()
    # Data from the encoder
    #try:
        #print ("Reading data")
        #rx_bytes[0] = bus.read_byte(address);
        #rx_bytes[1] = bus.read_byte(address);
        #rx_bytes[2] = bus.read_byte(address);
        #rx_bytes[3] = bus.read_byte(address);
    #except Exception as e:
        #print ("Read Error "+str(e))
    try:
        rx_bytes = bus.read_i2c_block_data(address , 0 , 4)
    except:
        print('Reading error')
    time_enc.append(time.time()-t0)
    print(rx_bytes)
    value = rx_bytes[0] + (rx_bytes[1] << 8) + (rx_bytes[2] << 16) #+ (rx_bytes[3] << 24)
    v = float(value) / 1000
    print ("Read value " + str(v))
    vel.append(v)
    
    # Data from the ADC
    voltag.append(channel.voltage)
    voltag2.append(channel2.voltage)
    voltag3.append(channel3.voltage)
    time_adc.append(time.time()-t0)
    #t2 = time.time()
    #print ("Reading time: " + str(t2-t1))
    #print('Speed: ' , np.round(np.array(vel[-1]) , 3), 'Voltage: ', np.round(np.array(voltage[-1]) , 3), 'Current: ', np.round((np.array(voltage2[-1]) - 2.604247869443707)*1000 * (1/185) * 1000) , 3)
    #sleep(dt - t2 + t1)
    
speed = np.array(vel)
v = np.array(voltag)
v2 = np.array(voltag2)  * 5
v3 = np.array(voltag3) * 5 

for i in range(len(speed)):
    if i == 0:
        continue
    if abs(speed[i] - speed[i-1]) > 100:
        speed[i] = speed[i-1]

f_s = 1/np.mean(np.diff(np.array(time_enc)))
f_c = 0.8

a, b = signal.butter(9 , f_c , 'lp' , analog=False , fs = f_s)
speed_filt = signal.filtfilt(a , b, speed)

v_filt = signal.filtfilt(a , b, v)

wind1 = np.sqrt( 2 * (m1 * v2 + q1)/ 1.225)
wind2 = np.sqrt( 2 * (m2 * v3 + q2)/ 1.225)

wind1_filt = signal.filtfilt(a , b, wind1)
wind2_filt = signal.filtfilt(a , b, wind2)

plt.figure()
plt.plot(speed) 
plt.plot(speed_filt) 
plt.ylabel('Speed')
plt.show()
 
plt.figure()
plt.plot(v)
plt.ylabel('Voltage')
plt.show()

plt.figure()
plt.plot(wind1)
plt.plot(wind1_filt) 
plt.ylabel('Voltage 2')
plt.show()

plt.figure()
plt.plot(wind2)
plt.plot(wind2_filt) 
plt.ylabel('Voltage 3'  )
plt.show()

plt.figure()
plt.plot(speed_filt , v , 'o')
plt.ylabel('Speed and voltage')
plt.show()

plt.plot(np.diff(np.array(time_enc)))
plt.show()

#plt.figure()
#plt.plot((v2 - 2.604247869443707)*1000 * (1/185) * 1000)
#plt.ylabel('Current')
#plt.show()

data = {
    "speed [rad/s]": speed,
    "voltage [V]": v,
    "voltage press1 [V]": v2,
    "voltage press2 [V]": v3,
    "dt [s]": dt
}

data_name = input('Namedata: ')
with open(data_name , "wb") as f:
    pickle.dump(data, f )
