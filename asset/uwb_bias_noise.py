'''
Outlier rejection & bias fitting of UWB distance measurements
'''
import numpy as np
import math
import csv
import statistics

realDist01 = []
measDist01 = [] # only rob0 and rob1
measDist01keep = []
realDist = [] # ground truth distance
measDist = [] # measurements of distance
with open("./dataset/dat01old.csv") as csvfile:
    reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC) # change contents to floats
    for row in reader: # each row is a list
        dist01 = math.sqrt((row[46]-row[53])**2+(row[48]-row[55])**2)
        dist02 = math.sqrt((row[46]-row[60])**2+(row[48]-row[62])**2)
        dist12 = math.sqrt((row[53]-row[60])**2+(row[55]-row[62])**2)
        realDist01.append(dist01)
        measDist01.append(row[6])
        measDist01keep.append(row[6])
        realDist.append([dist01, dist01, dist02, dist02, dist12, dist12])
        measDist.append([row[6], row[14],row[7],row[23],row[16],row[24]])
    print("finish reading the csv file")
realDist01 = np.array(realDist01)
measDist01 = np.array(measDist01)/1000
measDist01keep = np.array(measDist01keep)/1000
realDist = np.array(realDist)
measDist = np.array(measDist)/1000

import matplotlib.pyplot as plt
# distance of optiTrack VS UWB
plt.figure()
timePlot = np.arange(0, len(measDist01)/100, 0.01)
plt.plot(timePlot, measDist01, label='uwb')
plt.plot(timePlot, realDist01, label='optiTrack')
plt.title("distance between rob0 and rob1")
plt.xlabel("time (s)")
plt.ylabel("distance (m)")
plt.legend()

# average history filter to reject the outliers
def uwbRejeOutlier(uwbDatColum):
    distNoOutlier = []
    averValue = 0
    for i in range(len(uwbDatColum)):
        if i < 4:
            distNoOutlier.append(uwbDatColum[i])
        else:
            averValue = statistics.mean(uwbDatColum[i-3:i])
            if np.abs(averValue - uwbDatColum[i]) > 0.4:
                uwbDatColum[i] = uwbDatColum[i-1] # comment for noise-free dataset
                distNoOutlier.append(uwbDatColum[i])
            else:
                distNoOutlier.append(uwbDatColum[i])
    return np.array(distNoOutlier)
distNoOutlier = uwbRejeOutlier(measDist01)
plt.figure()
timePlot = np.arange(0, len(measDist01)/100, 0.01)
plt.plot(timePlot, distNoOutlier, label='uwb')
plt.plot(timePlot, realDist01, label='optiTrack')
plt.title("distance after outlier rejection")
plt.xlabel("time (s)")
plt.ylabel("distance (m)")
plt.legend()

# distribution of measurement error
plt.figure()
_ = plt.hist((measDist-realDist).flatten(), bins='auto')  # arguments are passed to np.histogram
plt.title("distribution of measurement error")
plt.xlabel("measurement error [m]")
plt.ylabel("number")

# Now fit a simple bias function to the data
from scipy import optimize
def test_func(x, k, b):
    return k * x + b
x_data = distNoOutlier
y_data = distNoOutlier - realDist01
params, params_covariance = optimize.curve_fit(test_func, x_data, y_data,
                                               p0=[1, 1])
print(params) # [0.07204161 0.62081056]
# And plot the resulting curve on the data
plt.figure(figsize=(6, 4))
plt.scatter(x_data, y_data, label='Data')
plt.plot(x_data, test_func(x_data, params[0], params[1]),
         label='Fitted function', color='red')
plt.legend(loc='best')
plt.title("bias function w.r.t distance")
plt.xlabel("measured distance (m)")
plt.ylabel("bias (m)")

# fitted data of distance01
plt.figure()
timePlot = np.arange(0, len(measDist01)/100, 0.01)
distBias = 0.072*distNoOutlier + 0.6208
plt.plot(timePlot, distNoOutlier-distBias, label='uwb')
plt.plot(timePlot, realDist01, label='optiTrack')
plt.title("fitted distance")
plt.xlabel("time (s)")
plt.ylabel("distance (m)")
plt.legend()

# fitted data of all distance
plt.figure()
realDistFlatten = (realDist.T).flatten()
measDistNoOutlie = np.array([])
for i in range(len(measDist[1,:])):
    measDistNoOutlie = np.append(measDistNoOutlie, uwbRejeOutlier(measDist[:,i]))
timePlot = np.arange(0, len(realDistFlatten)/100, 0.01)
distBias = params[0]*measDistNoOutlie + params[1]
plt.plot(timePlot, measDistNoOutlie-distBias, color='blue', label='uwb')
plt.plot(timePlot, realDistFlatten, color='red',  label='optiTrack')
plt.title("fitted distance")
plt.xlabel("time (s)")
plt.ylabel("distance (m)")
plt.legend()
plt.show()

# Fig? UWB data processing distance between rob0 and rob1, dat01
plt.figure(figsize=(8, 2))
timePlot = np.arange(0, len(measDist01)/100, 0.01)
distBias = 0.072*distNoOutlier + 0.6208
plt.plot(timePlot, measDist01keep, color='limegreen', label='UWB ranging')
plt.plot(timePlot, realDist01, color='blueviolet', label='Ground-truth')
# plt.plot(timePlot, distNoOutlier, label='outlier rejection')
plt.plot(timePlot, distNoOutlier-distBias, color='steelblue', label='Processed UWB data')
plt.xlabel("Time (s)", fontsize=12)
plt.ylabel("Distance (m)", fontsize=12)
plt.tight_layout() # make room for xlabel
plt.margins(x=0)
plt.legend(fontsize=12)
plt.show()