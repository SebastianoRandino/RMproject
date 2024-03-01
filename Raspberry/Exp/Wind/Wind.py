import numpy as np
import pickle
import matplotlib.pyplot as plt
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import board

with open('Calibration1Data' , 'rb') as file:
	data = pickle.load(file)

with open('Calibration2Data' , 'rb') as file:
	data2 = pickle.load(file)
	
m2 = data['fitting parameters'][0]
q2 = data['fitting parameters'][1]

m1 = data2['fitting parameters'][0]
q1 = data2['fitting parameters'][1]

#x = np.linspace(-5,5)
#plt.plot(x , m1*x +q1)
#plt.show()

v = np.array([5,6,7,8,9,10, 0])
dp = 0.5 * 1.225 * v**2

v1 = (dp - q1) / m1
v2 = (dp - q2) / m2

print(v1)
print(v2)

input()
 
t_s = 5 #[s]
n_s = 250 # only [8, 16, 32, 64, 128, 250, 475, 860] samples/second are allowed 
N_s = t_s * n_s

flag = True
voltages1 = []
voltages2 = []
volt_raw = np.zeros([N_s , 7])
volt_raw2 = np.zeros([N_s , 7])

print(volt_raw.shape)

# Initialize the I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
 
# Create an ADS1115 object
ads = ADS.ADS1115(i2c , gain = 0.6666666666666666 , data_rate = n_s)
 
# Define the analog input channel
channel = AnalogIn(ads, ADS.P0)
channel2 = AnalogIn(ads, ADS.P1)
j= 0
 
while flag == True:
    message = input('Press eenter to acquire:' )
    if message == 'stop':
        flag = False
        continue
    else:
        voltag = []
        voltag2 = []
        for i in range(N_s):
            voltag.append(channel.voltage)
            voltag2.append(channel2.voltage)
        voltag = np.array(voltag)
        voltag2 = np.array(voltag2)
        print(voltag.shape)
        volt_raw[: , j] =  voltag
        volt_raw2[: , j] =  voltag2
        print((np.mean(volt_raw[: , j])-5.2) * 5)
        print(np.mean(volt_raw2[: , j]) * 5)
        j = j + 1
        #voltages.append(np.mean(np.array(voltag)))
    #print(pressures[-1])
    #print(voltages[-1])

volt_raw = (volt_raw - 5.2) * 5
volt_raw2 = volt_raw2 * 5 

for i in range(7):
	print(np.mean(volt_raw[: , i]))
	
for i in range(7):
	print(np.mean(volt_raw2[: , i]))

print(v1)
print(v2)
	
data = {
'Volt_raw1': volt_raw,
'Volt_raw2': volt_raw2
}      

name_file = input('Text name for saving the file: ')  

with open(name_file , "wb") as f:
    pickle.dump(data, f )
    
