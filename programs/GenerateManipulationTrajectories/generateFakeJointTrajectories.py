from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt
import random
import csv

# Joint limits
qmin = [-30.0, -8.5, -93.1, -70.5, -74.1, -94.6, -75.4, -110.1]
qmax = [30.0, 14.0, 100.0, 15.4, 49.0, 91.4, 90.6, 38.7]

# Start joints position
q_start = qmin + qmax
q_start[:] = [x / 2 for x in q_start]
print("q start: ", q_start)


# time up to 150 s
time = np.linspace(0,1.0, num = 5000, endpoint= True)
y_demo = np.sin(time*3.5*np.pi/2)*30

y_demo = np.vstack((y_demo,np.sin(time*0.5*np.pi/2)*8.5))
y_demo = np.vstack((y_demo,-np.sin(time*1.1*np.pi/2)*80-10))
y_demo = np.vstack((y_demo,-np.sin(time*1.5*np.pi/2)*20-30))

for i in range(5):
    y_demo = np.vstack((y_demo,-np.sin(time*3*np.pi/2)*20))
# y_demo = np.vstack((y_demo,-np.sin(time*5*np.pi/2)*30))
# # Generate fake data between -1 and 1
# q = []
# x = [random.randint(-3, 3) for p in range(0, 8)]
# for i in range(len(x)):
#     # while ((x[i]<=5 and x[i]>=0) or (x[i]>=-5 and x[i]<0)):
#     while (x[i]==0):
#         x[i] = random.randint(-3,3)
# print(x)
# rnd = 0
# for i in range(0,8):
#     if x[i]<0:
#         aux = np.sin(-t**abs(x[i])/2) + q_start[i]
#     else:
#         aux = np.sin(t**(x[i])/2) + q_start[i]
#     q.append(aux.tolist())
# print(q)

# # Interpolate the data
# f = []
# for i in range(0,8):
#     aux = interp1d(t,q[i], kind='cubic')
#     f.append(aux)

# # Plot the interpolated data contained between -1 and 1
# t_final = np.linspace(0,1000, num=1000, endpoint= True)
# fig, axs = plt.subplots(2, 4)
# for i in range(0,8):
#     if i<4:
#         axs[0, i].plot(t,q[0], 'o', t_final, f[0](t_final), '-')
#     else:
#         axs[1, i-4].plot(t,q[0], 'o', t_final, f[0](t_final), '-')
# plt.show()


# # Rescale the data and plot it

# q_rescale = []
# for i in range (0,8):
#     aux = f[0](t_final)
#     aux_rescale = np.interp(aux,(aux.min(),aux.max()), (qmin[1]+2, qmax[1]-2))
#     q_rescale.append(aux_rescale.tolist())
# print(q_rescale)

# fig, axs = plt.subplots(2, 4)
# for i in range(0,8):
#     if i<4:
#         axs[0, i].plot(t_final, q_rescale[i], '-')
#     else:
#         axs[1, i-4].plot(t_final, q_rescale[i], '-')
# plt.show()

csv_file_write = open("trajectories/fake/fake-right-arm-motion-joint-"+str(1)+".csv", "w")
writer = csv.writer(csv_file_write, dialect='excel')
for t in range(0, 5000):
    rowData = [t, y_demo[0][t], y_demo[1][t], y_demo[2][t], y_demo[3][t], y_demo[4][t], y_demo[5][t], y_demo[6][t], y_demo[7][t], y_demo[8][t]]
    writer.writerow(rowData)
csv_file_write.close()