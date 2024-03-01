import smbus2
from time import sleep
import time
import numpy as np
import matplotlib.pyplot as plt

# I2C channel 1 is connected to the GPIO pins
channel = 1
address = 0x3F

# Initialize I2C (SMBus)
bus = smbus2.SMBus(channel)

rx_bytes = [0,0,0,0]

lenght = 10

vel = []

bytes_read = bus.read_i2c_block_data(address , 0 , 4)
print(bytes_read)
value = bytes_read[0] + (bytes_read[1] << 8) + (bytes_read[2] << 16) + (bytes_read[3] << 24)
print(value)

with smbus2.SMBus(1) as bus:
    mess = smbus2.i2c_msg.read(address , 4)

print(mess)

for i in range(lenght):
    t1 = time.time()
    try:
        print ("Reading data")
        rx_bytes[0] = bus.read_byte(address);
        rx_bytes[1] = bus.read_byte(address);
        rx_bytes[2] = bus.read_byte(address);
        #rx_bytes[3] = bus.read_byte(address);
    except Exception as e:
        print ("Read Error "+str(e))
    value = rx_bytes[0] + (rx_bytes[1] << 8) + (rx_bytes[2] << 16) #+ (rx_bytes[3] << 24)
    v = float(value) / 1000
    print ("Read value " + str(v))
    vel.append(v)
    t2 = time.time()
    print ("Reading time: " + str(t2-t1))
    #sleep(0.1 - t2 + t1)
    
data = np.array(vel)

plt.plot(data) 
plt.show()
    
