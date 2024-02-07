# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
import csv
import numpy as np

try:
    from IPython import get_ipython
    get_ipython().magic('clear')
    get_ipython().magic('reset -f')
except:
    pass

rows = []

file_path = "imu_content_line.csv"

with open(file_path, 'r', newline='') as csvfile:
    csvreader = csv.reader(csvfile)

    past_header = False
    for row in csvreader:
        
        if past_header:
            as_floats = [float(x) for x in row]
            rows.append(as_floats)
        else:
            print("Headers: ", row)
        
        past_header = True


rows = np.matrix(rows).T

secs = rows[0, :]
secs = secs.tolist()[0]
secs = np.array(secs)

ns = rows[1, :]
ns = ns.tolist()[0]
ns = np.array(ns)

time = secs + ns / 10e8

plt.plot(time, rows[2].T, label="Acc X")
plt.plot(time, rows[3].T, label="Acc Y")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s^2)")
plt.legend()
plt.grid()
plt.show()

plt.plot(time, rows[4].T, label="Rot Z")
plt.xlabel("Time (s)")
plt.ylabel("Rotational Velocity (rad/s)")
plt.legend()
plt.grid()
plt.show()












