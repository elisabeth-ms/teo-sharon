import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv


xori = []
yori = []
zori = []
framesori = []
qxori = []
qyori = []
qzori = []
qwori = []

number = 21



# with open('trajectories/test/test-right-arm-motion-smooth4-reaching.csv') as csv_file:
#     csv_reader = csv.reader(csv_file, delimiter=',')
#     line_count = 0
#     for row in csv_reader:
#         line_count += 1
#     print(line_count)
    
    
with open('trajectories/test/test-right-arm-motion-smooth2-reaching.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    current_line = 0
    for row in csv_reader:
        print(row)
        framesori.append(float(row[0]))
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

with open('trajectories/test/test-right-arm-motion-smooth'+str(number)+'-poses.csv') as csv_file:
    print('trajectories/test/test-right-arm-motion-smooth'+str(number)+'-poses.csv')
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

frames = []
oridist = []
with open('trajectories/test/test-right-arm-motion-smooth'+str(number)+'-data.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=' ')
    line_count = 0
    for row in csv_reader:
        frames.append(float(row[0]))
        oridist.append(float(row[1]))  
        line_count += 1
    print(f'Processed {line_count} lines.')
fig2 = plt.figure()
ax2 = fig2.add_subplot(111)

ax2.plot(frames, oridist)


plt.show()