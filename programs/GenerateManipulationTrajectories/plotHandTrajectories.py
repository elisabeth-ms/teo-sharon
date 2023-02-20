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

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


range_numbers = range(1,11,3)
filenames = []
for i in range_numbers:
    filenames.append('prueba'+str(i)+'-smoothed')

for filename in filenames:
    xori = []
    yori = []
    zori = []
    framesori = []
    qxori = []
    qyori = []
    qzori = []
    qwori = []

    with open('/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/shoulderElbowWirstHandTest/'+filename+'.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
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
            
        

    ax.plot(xori, yori, zori, label=filename)
    ax.plot(xori[0], yori[0], zori[0])
plt.legend()
plt.show()