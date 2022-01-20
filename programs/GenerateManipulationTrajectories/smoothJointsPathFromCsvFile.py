
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
from scipy import interpolate
import os

frames = []
q0 = []
q1 = []
q2 = []
q3 = []
q4 = []
q5 = []
q6 = []
q7 = []
with open('trajectories/test/test-right-arm-motion-smooth31-joint.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=' ')
    line_count = 0
    for row in csv_reader:
        frames.append(float(row[0]))
        q0.append(float(row[1]))  
        q1.append(float(row[2]))
        q2.append(float(row[3]))
        q3.append(float(row[4]))
        q4.append(float(row[5]))
        q5.append(float(row[6]))
        q6.append(float(row[7]))
        q7.append(float(row[8]))
        line_count += 1

    print(f'Processed {line_count} lines.')
# x = np.array(x)
# frames = np.array(frames)
# y = np.array(y)
# z = np.array(z)
# print(x)

fig, ax = plt.subplots(8)
fig.suptitle("Right arm motion.")


ax[0].plot(np.array(frames),np.array(q0), label='q0')
ax[0].set_ylim([-50, 50])
ax[1].plot(np.array(frames),np.array(q1), label='q1')
ax[1].set_ylim([-50, 50])

ax[2].plot(np.array(frames),np.array(q2), label='q2')
ax[1].set_ylim([-100, 100])

ax[3].plot(np.array(frames),np.array(q3), label='q3')
ax[3].set_ylim([-80, 25])

ax[4].plot(np.array(frames),np.array(q4), label='q4')
ax[4].set_ylim([-90, 100])

ax[5].plot(np.array(frames),np.array(q5), label='q5')
ax[5].set_ylim([-100, 100])

ax[6].plot(np.array(frames),np.array(q6), label='q6')
ax[6].set_ylim([-85, 100])

ax[7].plot(np.array(frames),np.array(q7), label='q7')
ax[7].set_ylim([-120, 50])


# ax[0].plot(frames,z, label='z')
# ax[0].legend()

# ax[1].plot(frames, qx, label='qx')
# ax[1].plot(frames, qy, label='qy')
# ax[1].plot(frames, qz, label='qz')
# ax[1].plot(frames, qw, label='qw')
# ax[1].legend()
# ax[0].grid(True)
# ax[1].grid(True)
# fig.savefig("trajectories/test-right-arm-motion.png")


# Smoothed
spl = interpolate.splrep(frames, q0)
#create interpolated lists of points
x2 = np.linspace(0,len(frames),1000)
q0new = interpolate.splev(x2,spl)

spl = interpolate.splrep(frames, q1)
#create interpolated lists of points
q1new = interpolate.splev(x2,spl)

spl = interpolate.splrep(frames, q2)
#create interpolated lists of points
q2new = interpolate.splev(x2,spl)

spl = interpolate.splrep(frames, q3)
#create interpolated lists of points
q3new = interpolate.splev(x2,spl)

spl = interpolate.splrep(frames, q4)
#create interpolated lists of points
q4new = interpolate.splev(x2,spl)

spl = interpolate.splrep(frames, q5)
#create interpolated lists of points
q5new = interpolate.splev(x2,spl)

spl = interpolate.splrep(frames, q6)
#create interpolated lists of points
q6new = interpolate.splev(x2,spl)

spl = interpolate.splrep(frames, q7)
#create interpolated lists of points
q7new = interpolate.splev(x2,spl)


fig2,ax2 = plt.subplots(8)

ax2[0].plot(x2,q0new, label='q0')
ax2[0].set_ylim([-50, 50])
ax2[1].plot(x2,np.array(q1new), label='q1')
ax2[1].set_ylim([-50, 50])

ax2[2].plot(np.array(x2),np.array(q2new), label='q2')
ax2[1].set_ylim([-100, 100])

ax2[3].plot(np.array(x2),np.array(q3new), label='q3')
ax2[3].set_ylim([-80, 25])

ax2[4].plot(np.array(x2),np.array(q4new), label='q4')
ax2[4].set_ylim([-90, 100])

ax2[5].plot(np.array(x2),np.array(q5new), label='q5')
ax2[5].set_ylim([-100, 100])

ax2[6].plot(np.array(x2),np.array(q6new), label='q6')
ax2[6].set_ylim([-85, 100])

ax2[7].plot(np.array(x2),np.array(q7new), label='q7')
ax2[7].set_ylim([-120, 50])

plt.show()

# tck, u = interpolate.splprep([frames, z], s=0.02)
# #create interpolated lists of points
# frameZnew, znew = interpolate.splev(u,tck)

# tck, u = interpolate.splprep([frames, qx], s=0.02)
# #create interpolated lists of points
# frameqxnew, qxnew = interpolate.splev(u,tck)

# tck, u = interpolate.splprep([frames, qy], s=0.02)
# #create interpolated lists of points
# frameqynew, qynew = interpolate.splev(u,tck)

# tck, u = interpolate.splprep([frames, qz], s=0.02)
# #create interpolated lists of points
# frameqznew, qznew = interpolate.splev(u,tck)

# tck, u = interpolate.splprep([frames, qw], s=0.02)
# #create interpolated lists of points
# frameqwnew, qwnew = interpolate.splev(u,tck)

# fig2,ax2 = plt.subplots(2)
# fig2.suptitle('Smoothed right arm motion.')

# ax2[0].plot(frameXnew, xnew, label="x")
# ax2[0].plot(frameYnew, ynew, label='y')
# ax2[0].plot(frameZnew, znew, label='z')
# ax2[0].legend()

# ax2[1].plot(frameqxnew, qxnew, label='qx')
# ax2[1].plot(frameqynew, qynew, label='qy')
# ax2[1].plot(frameqznew, qznew, label='qz')
# ax2[1].plot(frameqwnew, qwnew, label='qw')
# ax2[1].legend()


# ax2[0].grid(True)
# ax2[1].grid(True)
# plt.show()
# fig2.savefig("trajectories/graspcup2/test-right-arm-motion-smooth23.png")

csv_file_write = open("trajectories/test/test-right-arm-motion-smooth31-joint-more-points.csv", "w")
writer = csv.writer(csv_file_write, dialect='excel')
for i in range(0, len(x2)):
    rowData = [i, q0new[i], q1new[i], q2new[i], q3new[i], q4new[i], q5new[i], q6new[i], q7new[i]]
    writer.writerow(rowData)
csv_file_write.close()
# print(frameXnew.shape, frameYnew.shape, frameZnew.shape, frameqxnew.shape, frameqynew.shape, frameqznew.shape, frameqwnew.shape)
