#!/usr/bin/env python3

# code to simulate a pid loop under various models for plant/process
# and various PID parameter tunings


from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt

kp = 0.0
ki = .54/.001
kd = 0.0

m = 1 # nT/A

bext = 0.0
bfield = 0.0
current = 0.0
Setpoint = 0.0
errSum = 0.0
lastErr = 0.0

lastTime = datetime.now()
now = lastTime

timeChange = 0.0

def Compute():
    global lastTime, now, error, errSum, lastErr
    global bfield, current, Setpoint
    global timeChange

    now = datetime.now()
    timeChange = (now - lastTime).total_seconds()


    error = Setpoint - bfield
    errSum += (error * timeChange)
    dErr = (error - lastErr) / timeChange
  
    current = kp * error + ki * errSum + kd * dErr
  
    # Remember some variables for next time
    lastErr = error;
    lastTime = now;

def get_bext():
    global seconds_since_start, bext
    if(seconds_since_start < 1):
        bext = 0
    elif(seconds_since_start < 5):
        bext = 1
    else:
        bext = 0
    noise = np.random.normal(0,.1,100)
    bext = bext + noise[0]

whichtime = 0
oldcurrent = 0

def Correct():
    times = 1 # allows to correct on a slower timescale than sample time.
    global bfield, bext, m, current, whichtime, oldcurrent
    whichtime = whichtime + 1
    if (whichtime == times):
        bfield = bext + m*current
        whichtime = 0
        oldcurrent = current
    else:
        bfield = bext + m*oldcurrent

startTime = now
seconds_since_start = 0.0

tarray = np.array([])
darray = np.array([])
iarray = np.array([])
oarray = np.array([])
earray = np.array([])
tcarray = np.array([])

while(seconds_since_start < 10):
    seconds_since_start = (now-startTime).total_seconds()
    get_bext() # driving the system

    Compute()

    Correct()

    print(seconds_since_start, bext, bfield, current, error)
    tarray = np.append(tarray,seconds_since_start)
    darray = np.append(darray,bext)
    iarray = np.append(iarray,bfield)
    oarray = np.append(oarray,current)
    earray = np.append(earray,error)
    tcarray = np.append(tcarray,timeChange)

plt.figure()
plt.plot(tarray,iarray,tarray,darray,tarray,oarray)
plt.figure()
n, bins, patches = plt.hist(tcarray, 400)

plt.show()
