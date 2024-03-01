import board
import time
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import matplotlib.pyplot as plt
import numpy as np
 
# Initialize the I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
 
# Create an ADS1115 object 0.6666666666666666
ads = ADS.ADS1115(i2c , gain = 0.6666666666666666 , data_rate = 250)
# data_rate  = [8, 16, 32, 64, 128, 250, 475, 860] samples for second -- it defines the frequency (the more the noise :( )
 
# Define the analog input channel
channel = AnalogIn(ads, ADS.P0)
 
data = []
flag= True
i=0
# Loop to read the analog input continuously
while flag:
    print("Analog Value: ", channel.value, "Voltage: ", channel.voltage)
    t1 = time.time()
    data.append(channel.voltage)
    t2 = time.time()
    #time.sleep(0.01)
    print(t2-t1)
    i = i +1
    if i > 1000:
        flag = False

plt.plot(np.array(data) )
plt.show()

print(np.mean(np.array(data))) 
#print((np.mean(np.array(data)) - 2.60380967978)*1000 * 1/185 )
