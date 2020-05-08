"""
UWB Ranging frequency of different number of robots
"""
import numpy as np
import math
import csv
import matplotlib.pyplot as plt

def checkFreqByColumn(col):
    # return a list of how many 10-seconds each ranging takes
    length = len(col)
    freqList = []
    i = 0
    current = col[0]
    count = 0
    while i<length :
        if current==col[i]:
            count+=1
        else:
            freqList.append(count)
            count = 0
            current = col[i]
        i+=1
    return freqList

# 3 robots
realDist3 = []
with open("./dataset/dat01old.csv") as csvfile:
    reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC) # change contents to floats
    for row in reader: # each row is a list
        realDist3.append(row[6:8]+row[14:15]+row[16:17]+row[23:25])       
    print("reading the csv file is done")
allFreqList3 = []
for i in range(len(realDist3[0])):
    column_i = [row[i] for row in realDist3]
    allFreqList3 = allFreqList3 + checkFreqByColumn(column_i)

# 4 robots
realDist4 = []
with open("./dataset/dat03old.csv") as csvfile:
    reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC) # change contents to floats
    for row in reader: # each row is a list
        realDist4.append(row[6:8]+row[14:15]+row[16:17]+row[23:25])       
    print("reading the csv file is done")
allFreqList4 = []
for i in range(len(realDist4[0])):
    column_i = [row[i] for row in realDist4]
    allFreqList4 = allFreqList4 + checkFreqByColumn(column_i)

# 5 robots
realDist5 = []
with open("./dataset/freq_5robots.csv") as csvfile:
    reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC) # change contents to floats
    for row in reader: # each row is a list
        realDist5.append(row[0:3])       
    print("reading the csv file is done")
allFreqList5 = []
for i in range(len(realDist5[0])):
    column_i = [row[i] for row in realDist5]
    allFreqList5 = allFreqList5 + checkFreqByColumn(column_i)

# 6 robots
realDist6 = []
with open("./dataset/freq_6robots.csv") as csvfile:
    reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC) # change contents to floats
    for row in reader: # each row is a list
        realDist6.append(row[0:3])       
    print("reading the csv file is done")
allFreqList6 = []
for i in range(len(realDist6[0])):
    column_i = [row[i] for row in realDist6]
    allFreqList6 = allFreqList6 + checkFreqByColumn(column_i)

plt.style.use('seaborn-white')
kwargs = dict(histtype='stepfilled', alpha=0.3, density=True, bins=60, ec="k")
plt.hist([i*10 for i in allFreqList3], **kwargs, label="3 robots")
plt.hist([i*10 for i in allFreqList4], **kwargs, label="4 robots")
plt.hist([i*10 for i in allFreqList5], **kwargs, label="5 robots")
plt.hist([i*10 for i in allFreqList6], **kwargs, label="6 robots")
plt.xlabel("Time (ms)", fontsize=12)
plt.ylabel("Density", fontsize=12)
plt.margins(x=0)
plt.legend(fontsize=12)
plt.show()