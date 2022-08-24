import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv

time_step = 0.3

xori = []
yori = []
zori = []
framesori = []
qxori = []
qyori = []
qzori = []
qwori = []

number = 10



# with open('trajectories/test/test-right-arm-motion-smooth4-reaching.csv') as csv_file:
#     csv_reader = csv.reader(csv_file, delimiter=',')
#     line_count = 0
#     for row in csv_reader:
#         line_count += 1
#     print(line_count)
nDemo = 1
oriPathFile = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + str(nDemo) + "-smoothed-link-adj.csv";
csvPoseFileWrite =  "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + str(nDemo) + "-poses-optimization.csv";
csvDataFileWrite = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + str(nDemo) + "-smoothed-data-optimization.csv";
csvQFileWrite = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + str(nDemo) + "-q-positions-optimization.csv"
csvQFileTracIK = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + str(nDemo) + "-smoothed-ik-joints.csv"


with open(oriPathFile) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=' ')
    current_line = 0
    for row in csv_reader:
        print(row[0])
        #Frame
        framesori.append(float(row[0]))
        
        #hand position+orientation
        xori.append(float(row[1]))  
        yori.append(float(row[2]))
        zori.append(float(row[3]))
        qwori.append(float(row[7]))
        qxori.append(float(row[4]))
        qyori.append(float(row[5]))
        qzori.append(float(row[6]))
       
        print(current_line)
        current_line +=1
    
u = np.array([1.0, 0.0, 0])
q = np.array([qwori[0], qwori[1], qwori[2], qwori[3]])
        
    
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(xori, yori, zori,'b', label='humand hand motion')
ax.plot(xori[0], yori[0], zori[0],'bo')




x = []
y = []
z = []
frames = []
qx = []
qy = []
qz = []
qw = []

with open(csvPoseFileWrite) as csv_file:
    print(csvPoseFileWrite)
    csv_reader = csv.reader(csv_file, delimiter=' ')
    line_count = 0
    for row in csv_reader:
        frames.append(float(row[0]))
        x.append(float(row[1]))  
        y.append(float(row[2]))
        z.append(float(row[3]))
        qw.append(float(row[7]))
        qx.append(float(row[4]))
        qy.append(float(row[5]))
        qz.append(float(row[6]))
        print(x[line_count], y[line_count], z[line_count])
        line_count += 1
    print(f'Processed {line_count} lines.')
    

ax.plot(x, y, z,'g', label='Robots end effector motion')
ax.plot(x[0], y[0], z[0],'go')
ax.legend()

fig3, ax3 = plt.subplots(3)
fig.suptitle("Human arm motion.")


ax3[0].plot(np.array(framesori),np.array(xori), label='x')
ax3[1].plot(np.array(framesori),np.array(yori), label='y')
ax3[2].plot(np.array(framesori),np.array(zori), label='z')
ax3[0].plot(np.array(frames),np.array(x),'-.', label='x-opt',)
ax3[1].plot(np.array(frames),np.array(y),'-.', label='y-opt')
ax3[2].plot(np.array(frames),np.array(z),'-.', label='z-opt')

frames = []
posdist = []
oridist = []
angleDistShoulderElbow = []
angleDistElbowWrist = []
with open(csvDataFileWrite) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=' ')
    line_count = 0
    for row in csv_reader:
        frames.append(float(row[0]))
        posdist.append(float(row[1]))
        oridist.append(float(row[2]))
        angleDistShoulderElbow.append(float(row[3]))
        angleDistElbowWrist.append(float(row[4]))
        line_count += 1
    print(f'Processed {line_count} lines.')
    
fig1 = plt.figure()
fig1.suptitle("Hand position dist.")
ax2 = fig1.add_subplot(111)
ax2.plot(frames, posdist)

fig2 = plt.figure()
fig2.suptitle("Hand orientation dist.")
ax2 = fig2.add_subplot(111)

ax2.plot(frames, oridist)

fig3 = plt.figure()
fig3.suptitle("Shoulder elbow angle dist.")
ax3 = fig3.add_subplot(111)
ax3.plot(frames, angleDistShoulderElbow)

fig4 = plt.figure()
fig4.suptitle("Elbow wrist angle dist.")
ax4 = fig4.add_subplot(111)
ax4.plot(frames, angleDistElbowWrist)


frames = []
q0 = []
q1 = []
q2 = []
q3 = []
q4 = []
q5 = []
q6 = []
q7 = []
print(csvQFileWrite)
with open(csvQFileWrite) as csv_file:
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

frames_trac_ik = []
q0_trac_ik = []
q1_trac_ik = []
q2_trac_ik = []
q3_trac_ik = []
q4_trac_ik = []
q5_trac_ik = []
q6_trac_ik = []
q7_trac_ik = []
print(csvQFileTracIK)
with open(csvQFileTracIK) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=' ')
    line_count = 0
    for row in csv_reader:
        frames_trac_ik.append(float(row[0]))
        q0_trac_ik.append(float(row[1]))  
        q1_trac_ik.append(float(row[2]))
        q2_trac_ik.append(float(row[3]))
        q3_trac_ik.append(float(row[4]))
        q4_trac_ik.append(float(row[5]))
        q5_trac_ik.append(float(row[6]))
        q6_trac_ik.append(float(row[7]))
        q7_trac_ik.append(float(row[8]))
        line_count += 1

    print(f'Processed {line_count} lines.')
# x = np.array(x)
# frames = np.array(frames)
# y = np.array(y)
# z = np.array(z)
# print(x)

fig, ax = plt.subplots(4,2)
fig.suptitle("Right arm motion.")


ax[0,0].plot(np.array(frames),np.array(q0), label='q0')
ax[0,0].plot(np.array(frames_trac_ik),np.array(q0_trac_ik),'r-.', label='q0-trac-ik')
ax[0,0].set_ylim([-50, 50])
ax[1,0].plot(np.array(frames),np.array(q1), label='q1')
ax[1,0].plot(np.array(frames_trac_ik),np.array(q1_trac_ik),'r-.', label='q1-trac-ik')
ax[1,0].set_ylim([-50, 50])

ax[2,0].plot(np.array(frames),np.array(q2), label='q2')
ax[2,0].plot(np.array(frames_trac_ik),np.array(q2_trac_ik),'r-.', label='q2-trac-ik')
ax[2,0].set_ylim([-100, 100])

ax[3,0].plot(np.array(frames),np.array(q3), label='q3')
ax[3,0].plot(np.array(frames_trac_ik),np.array(q3_trac_ik),'r-.', label='q3-trac-ik')
ax[3,0].set_ylim([-80, 25])

ax[0,1].plot(np.array(frames),np.array(q4), label='q4')
ax[0,1].plot(np.array(frames_trac_ik),np.array(q4_trac_ik),'r-.', label='q4-trac-ik')
ax[0,1].set_ylim([-90, 100])

ax[1,1].plot(np.array(frames),np.array(q5), label='q5')
ax[1,1].plot(np.array(frames_trac_ik),np.array(q5_trac_ik),'r-.', label='q5-trac-ik')
ax[1,1].set_ylim([-100, 100])

ax[2,1].plot(np.array(frames),np.array(q6), label='q6')
ax[2,1].plot(np.array(frames_trac_ik),np.array(q6_trac_ik),'r-.', label='q6-trac-ik')
ax[2,1].set_ylim([-85, 101])

ax[3,1].plot(np.array(frames),np.array(q7), label='q7')
ax[3,1].plot(np.array(frames_trac_ik),np.array(q7_trac_ik),'r-.', label='q7-trac-ik')
ax[3,1].set_ylim([-120, 50])

fig, ax = plt.subplots(8)
fig.suptitle("Joints veloctity")

q0vel = [0.0]
q1vel = [0.0]
q2vel = [0.0]
q3vel = [0.0]
q4vel = [0.0]
q5vel = [0.0]
q6vel = [0.0]
q7vel = [0.0]
for i in range(1,len(q0),1):
    q0vel.append((q0[i]-q0[i-1])/time_step)
    q1vel.append((q1[i] - q1[i-1])/time_step)
    q2vel.append((q2[i] - q2[i-1])/time_step)
    q3vel.append((q3[i] - q3[i-1])/time_step)
    q4vel.append((q4[i] - q4[i-1])/time_step)
    q5vel.append((q5[i] - q5[i-1])/time_step)
    q6vel.append((q6[i] - q6[i-1])/time_step)
    q7vel.append((q7[i] - q7[i-1])/time_step)
ax[0].plot(np.array(frames),np.array(q0vel), label='q0')
ax[1].plot(np.array(frames),np.array(q1vel), label='q1')
ax[2].plot(np.array(frames),np.array(q2vel), label='q2')
ax[3].plot(np.array(frames),np.array(q3vel), label='q3')
ax[4].plot(np.array(frames),np.array(q4vel), label='q4')
ax[5].plot(np.array(frames),np.array(q5vel), label='q5')
ax[6].plot(np.array(frames),np.array(q6vel), label='q6')
ax[7].plot(np.array(frames),np.array(q7vel), label='q7')


fig, ax = plt.subplots(8)
fig.suptitle("Joints acceleration")

q0acc = [0.0]
q1acc = [0.0]
q2acc = [0.0]
q3acc = [0.0]
q4acc = [0.0]
q5acc = [0.0]
q6acc = [0.0]
q7acc = [0.0]
for i in range(1,len(q0),1):
    q0acc.append((q0vel[i]-q0vel[i-1])/time_step)
    q1acc.append((q1vel[i] - q1vel[i-1])/time_step)
    q2acc.append((q2vel[i] - q2vel[i-1])/time_step)
    q3acc.append((q3vel[i] - q3vel[i-1])/time_step)
    q4acc.append((q4vel[i] - q4vel[i-1])/time_step)
    q5acc.append((q5vel[i] - q5vel[i-1])/time_step)
    q6acc.append((q6vel[i] - q6vel[i-1])/time_step)
    q7acc.append((q7vel[i] - q7vel[i-1])/time_step)
ax[0].plot(np.array(frames),np.array(q0acc), label='q0')
ax[1].plot(np.array(frames),np.array(q1acc), label='q1')
ax[2].plot(np.array(frames),np.array(q2acc), label='q2')
ax[3].plot(np.array(frames),np.array(q3acc), label='q3')
ax[4].plot(np.array(frames),np.array(q4acc), label='q4')
ax[5].plot(np.array(frames),np.array(q5acc), label='q5')
ax[6].plot(np.array(frames),np.array(q6acc), label='q6')
ax[7].plot(np.array(frames),np.array(q7acc), label='q7')

fig, ax = plt.subplots(8)
fig.suptitle("Joints jerk")

q0jerk = [0.0]
q1jerk = [0.0]
q2jerk = [0.0]
q3jerk = [0.0]
q4jerk = [0.0]
q5jerk = [0.0]
q6jerk = [0.0]
q7jerk = [0.0]
for i in range(1,len(q0),1):
    q0jerk.append((q0acc[i]-q0acc[i-1])/time_step)
    q1jerk.append((q1acc[i] - q1acc[i-1])/time_step)
    q2jerk.append((q2acc[i] - q2acc[i-1])/time_step)
    q3jerk.append((q3acc[i] - q3acc[i-1])/time_step)
    q4jerk.append((q4acc[i] - q4acc[i-1])/time_step)
    q5jerk.append((q5acc[i] - q5acc[i-1])/time_step)
    q6jerk.append((q6acc[i] - q6acc[i-1])/time_step)
    q7jerk.append((q7acc[i] - q7acc[i-1])/time_step)
ax[0].plot(np.array(frames),np.array(q0jerk), label='q0')
ax[1].plot(np.array(frames),np.array(q1jerk), label='q1')
ax[2].plot(np.array(frames),np.array(q2jerk), label='q2')
ax[3].plot(np.array(frames),np.array(q3jerk), label='q3')
ax[4].plot(np.array(frames),np.array(q4jerk), label='q4')
ax[5].plot(np.array(frames),np.array(q5jerk), label='q5')
ax[6].plot(np.array(frames),np.array(q6jerk), label='q6')
ax[7].plot(np.array(frames),np.array(q7jerk), label='q7')
# # ax[0].set_ylim([-50, 50])
plt.subplots_adjust(left=0.1, right=0.9, top=0.95, bottom=0.05)   
plt.subplots_adjust(hspace=.0)
plt.show()