from turtle import color
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
from scipy import interpolate
import os
import matplotlib.colors as mcolors

x = []
y = []
z = []
frames = []
qx = []
qy = []
qz = []
qw = []

x_wrist = []
y_wrist = []
z_wrist = []
qx_wrist = []
qy_wrist = []
qz_wrist = []
qw_wrist = []

x_elbow = []
y_elbow = []
z_elbow = []
qx_elbow = []
qy_elbow = []
qz_elbow = []
qw_elbow = []

x_shoulder = []
y_shoulder = []
z_shoulder = []
qx_shoulder = []
qy_shoulder = []
qz_shoulder = []
qw_shoulder = []

x_neck = []
y_neck = []
z_neck = []
qx_neck = []
qy_neck = []
qz_neck = []
qw_neck = []

x_hip   = []
y_hip   = []
z_hip   = []
qx_hip  = []
qy_hip  = []
qz_hip  = []
qw_hip  = []

nDemo = 3 

world_frame_is_rotated = True
csvPoseFileRepeated =  "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + str(nDemo) + "-repeated-frames.csv";



with open(csvPoseFileRepeated) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        print(row)
        if row != []:
            if len(frames) > 0:
                if(float(row[0]) != frames[-1]):
                    addFrame = True
                else:
                    addFrame = False
            else:
                addFrame = True
            if addFrame:
                frames.append(float(row[0]))
                x.append(float(row[1]))
                y.append(float(row[2]))
                z.append(float(row[3]))
                qw.append(float(row[7]))
                qx.append(float(row[4]))
                qy.append(float(row[5]))
                qz.append(float(row[6]))

                # wrist position+orientation

                x_wrist.append(float(row[8]))
                y_wrist.append(float(row[9]))
                z_wrist.append(float(row[10]))
                qx_wrist.append(float(row[11]))
                qy_wrist.append(float(row[12]))
                qz_wrist.append(float(row[13]))
                qw_wrist.append(float(row[14]))

                # elbow position+orientation

                x_elbow.append(float(row[15]))
                y_elbow.append(float(row[16]))
                z_elbow.append(float(row[17]))
                qx_elbow.append(float(row[18]))
                qy_elbow.append(float(row[19]))
                qz_elbow.append(float(row[20]))
                qw_elbow.append(float(row[21]))

                x_shoulder.append(float(row[22]))
                y_shoulder.append(float(row[23]))
                z_shoulder.append(float(row[24]))
                qx_shoulder.append(float(row[25]))
                qy_shoulder.append(float(row[26]))
                qz_shoulder.append(float(row[27]))
                qw_shoulder.append(float(row[28]))
                
                #neck position+orientation
                
                x_neck.append(float(row[29]))
                y_neck.append(float(row[30]))
                z_neck.append(float(row[31]))
                qx_neck.append(float(row[32]))
                qy_neck.append(float(row[33]))
                qz_neck.append(float(row[34]))
                qw_neck.append(float(row[35]))
                
                #hip position+orientation
                x_hip.append(float(row[36]))
                y_hip.append(float(row[37]))
                z_hip.append(float(row[38]))
                qx_hip.append(float(row[39]))
                qy_hip.append(float(row[40]))
                qz_hip.append(float(row[41]))
                qw_hip.append(float(row[42]))
                
                
                line_count += 1
                # if line_count == 400:
                #     break

    print(f'Processed {line_count} lines.')

# if (world_frame_is_rotated):
#     x = -y
#     y = x
#     x_wrist = -x_wrist

filename_not_repeated = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba"+str(nDemo)+".csv"

csv_file_write = open(filename_not_repeated, "w")
writer = csv.writer(csv_file_write, dialect='excel')
for i in range(0, len(frames)):
    rowData = [i, x[i], y[i], z[i], qx[i], qy[i], qz[i], qw[i],
               x_wrist[i], y_wrist[i], z_wrist[i], qx_wrist[i], qy_wrist[i], qz_wrist[i], qw_wrist[i],
               x_elbow[i], y_elbow[i], z_elbow[i], qx_elbow[i], qy_elbow[i], qz_elbow[i], qw_elbow[i],
               x_shoulder[i], y_shoulder[i], z_shoulder[i], qx_shoulder[i], qy_shoulder[i], qz_shoulder[i], qw_shoulder[i],
               x_neck[i], y_neck[i], z_neck[i], qx_neck[i], qy_neck[i], qz_neck[i], qw_neck[i],
               x_hip[i], y_hip[i], z_hip[i], qx_hip[i], qy_hip[i], qz_hip[i], qw_hip[i]]
    writer.writerow(rowData)
csv_file_write.close()
    
