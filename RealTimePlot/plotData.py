import numpy
from scipy.interpolate import interp1d
from matplotlib import pyplot as plt

data = numpy.genfromtxt('2015-01-29/right_leg/prelim_csv.csv', skiprows=1, delimiter=',')
time = data[:,0]
accel = data[:,1]

time = time- time[0]
end = time.max()
ts = numpy.arange(end + 1)
new_acc = numpy.interp(ts, time, accel)

f = interp1d(time, accel)
#f2 = interp1d(time, accel, kind='cubic')

plt.plot(ts[30000:70000], new_acc[30000:70000], '-')

plt.title("Acceleration Data")
plt.xlabel("Unix Timestamp (millis)")
plt.ylabel("Mag of Acceleration")
plt.legend(['data', 'linear', 'cubic'], loc='best')
plt.grid(True)

plt.savefig('testPlot_right_leg.png', dpi=100)

plt.show()

