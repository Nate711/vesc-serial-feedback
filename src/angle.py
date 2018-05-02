import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
#lengths of top and bottom legs
top_leg_len = 60.0
bot_leg_len = 120.0

#motor angle data
text_file = open("motordata.txt")
lines = text_file.read().split('\n')

number_points = len(lines)

myarray = np.array(lines)
print(myarray)

#arrays for motor angles A and B
angleA = []
angleB = []

#fill arrays for motor angles A and B
for x in range(0, number_points-1):
	angles = myarray[x].split('\t')
	# print(len(angles))
	angleA.append(angles[0])
	angleB.append(angles[1])

#arrays for x and y coordinates of leg's tip
xcor = []
ycor = []

#calculate and fill x and y coordinates
for x in range(0, number_points-1):
    theta = (float(angleB[x]) + float(angleA[x])) * 0.5 * math.pi / 180.0
    gamma = (float(angleB[x]) - float(angleA[x])) * 0.5 * math.pi / 180.0
    exten_len = - top_leg_len*math.cos(gamma) - math.sqrt(math.pow(bot_leg_len,2)-math.pow(top_leg_len,2)*math.pow(math.sin(gamma), 2))
    ycor.append(math.sin(theta)*exten_len)
    xcor.append(math.cos(theta)*exten_len)

#plt.plot(angleA)
#plt.plot(angleB)
#graph scatter plot of coordinates
plt.plot(0, xcor, ycor)
#plt.xticks(np.arange(min(xcor), max(xcor), 10))
plt.show()
