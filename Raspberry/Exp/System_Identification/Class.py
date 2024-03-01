from ExpSim import ExpSim
import matplotlib.pyplot as plt
import numpy as np
import pickle

len = 2800
step = 16

sim_exp = ExpSim(len , step)

w, V , v1 , v2, rho , t_w, t_V , t_v1 , t_v2 = sim_exp.simulation()

plt.plot(t_w , w)
plt.show() 

plt.plot(t_V , V)
plt.show() 

plt.plot(t_v1 , v1)
plt.show()

plt.plot(t_v2 , v2)
plt.show()  

plt.plot(w , V)
plt.show()

data = {
    "t_w" : t_w,
    "w" : w,
    "t_V" : t_V,
    "V" : V,
    "t_v1" : t_v1,
    "v1" : v1,
    "t_v2" : t_v2,
    "v2" : v2,
    "rho" : rho 
}

data_name = input('Simulation name: ')

with open(data_name , "wb") as f:
	pickle.dump(data , f)
