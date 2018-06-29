#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

State = np.asarray([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

def Derivatives(State):
	m = 1.0
	k = 3.0
	c = 2.0

	ucontrol = 0.0

	A = np.asarray([[0.0,1.0],[-k/m,-c/m]])
	B = np.asarray([0.0,1.0/m])

	Statedot = np.dot(A,State) + B*ucontrol
	return Statedot

Statedot = Derivatives(State)

tfinal = 10.0
tInit = 0.0

timestep = 0.001

time = np.arange(tInit,tfinal+timestep,timestep)

StateOut = 	np.zeros((2,len(time)))

for idx in range (0,len(time)):
	print ('Simulation', time[idx]/tfinal*100,'percent complete')
	StateOut[:,idx] = State
	Statedot = Derivatives(State)
	State += Statedot*timestep

position = StateOut[0,:]
velocity = StateOut[1,:]

plt.figure()
plt.subplot(121)
plt.plot(time, position)
plt.xlabel('Time(sec')
plt.ylabel('Position(m)')

plt.subplot(122)
plt.plot(time, velocity)
plt.xlabel('Time(sec')
plt.ylabel('velocity(m/s)')

plt.grid()
plt.show()

