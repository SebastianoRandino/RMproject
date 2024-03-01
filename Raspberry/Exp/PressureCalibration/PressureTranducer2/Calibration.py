import board
import time
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import pickle

flag = True
pressures = []
voltages = []

t_s = 1 #[s]
n_s = 250 # only [8, 16, 32, 64, 128, 250, 475, 860] samples/second are allowed 
N_s = t_s * n_s

# Initialize the I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
 
# Create an ADS1115 object
ads = ADS.ADS1115(i2c , gain = 0.6666666666666666 , data_rate = n_s)
 
# Define the analog input channel
channel = AnalogIn(ads, ADS.P0)

while flag == True:
    message = input('Insert pressure value in mmH2O [press stop to end]:' )
    if message == 'stop':
        flag = False
        continue
    else:
        voltag = []
        pressures.append(float(message) * 9.80665) #apppend data and convert in Pascal
        for i in range(N_s):
            voltag.append(channel.voltage)
        voltages.append(np.mean(np.array(voltag)))
    print(pressures[-1])
    print(voltages[-1])

pressures = np.array(pressures)
voltages = np.array(voltages) * 5

def linear(x,m,c):
    return m*x + c
    
par, cov = curve_fit(linear , voltages , pressures)

data = {
'pressures': pressures,
'voltages': voltages,
'fitting parameters': par,
'fitting covariance': cov
}        

with open('Calibration2Data' , 'wb') as file:
    pickle.dump(data , file)

x = np.linspace(np.min(voltages) , np.max(voltages))
y = linear(x , par[0] , par[1])
plt.plot(voltages , pressures , 'r*' , label= 'Data')
plt.plot(x , y , 'k' , label = 'fit')
plt.xlabel(r'$\Delta V$ [V]')
plt.ylabel(r'$\Delta p$ [Pa]')
plt.legend()
plt.show()
plt.savefig('Calibration2')
