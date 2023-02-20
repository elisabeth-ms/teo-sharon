import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
import PyKDL
import math

xori = []
yori = []
zori = []
framesori = []
qxori = []
qyori = []
qzori = []
qwori = []

number = 20


def quatHProd(p, q):
    """Compute the Hamilton product of quaternions `p` and `q`."""
    r = np.array([p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3],
                  p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2],
                  p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1],
                  p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0]])
    return r

def rotVecByQuat(u, q):
    """Rotate a 3-vector `u` according to the quaternion `q`. The output `v` is
    also a 3-vector such that::

        [0; v] = q * [0; u] * q^{-1}

    with Hamilton product."""
    v = quatHProd(quatHProd(q, np.append(0, u)), quatRecip(q))
    return v[1:]

def quatRecip(q):
    """Compute the reciprocal of quaternion `q`."""
    return quatConj(q) / np.dot(q,q)

def quatConj(q):
    """Return the conjugate of quaternion `q`."""
    return np.append(q[0], -q[1:])



with open('trajectories/test/test-right-arm-motion-smooth2-reaching.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        line_count += 1
    
    print(line_count)

with open('trajectories/test/test-right-arm-motion-smooth2-reaching.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    current_line = 0
    for row in csv_reader:
        if current_line >= line_count-4:
            framesori.append(float(row[0]))
            xori.append(float(row[1]))  
            yori.append(float(row[2]))
            zori.append(float(row[3]))
            qwori.append(float(row[7]))
            qxori.append(float(row[4]))
            qyori.append(float(row[5]))
            qzori.append(float(row[6]))
        current_line +=1
    
ux = np.array([1.0, 0.0, 0])
uy = np.array([0.0, 1.0, 0])
uz = np.array([0.0, 0.0, 1.0])


q = np.array([qwori[3], qxori[3], qyori[3], qzori[3]])


rot = PyKDL.Rotation.Quaternion(qxori[3],qyori[3] , qzori[3], qwori[3])
# rot = PyKDL.Rotation.Quaternion(0, 1, 0, 0)
x_axes = rot.UnitX()
y_axes = rot.UnitY()
z_axes = rot.UnitZ()
print("x: ",x_axes)

qresult =rot.GetQuaternion()
print(qresult)


# rot = rot*PyKDL.Rotation.RotX(-math.pi/2.0)
rot = rot*PyKDL.Rotation.RotZ(-math.pi/2.0)
# rot = rot*PyKDL.Rotation.RotY(math.pi/4.0)

x_axes2 = rot.UnitX()
y_axes2 = rot.UnitY()
z_axes2 = rot.UnitZ()

print(qresult)

    
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(xori, yori, zori,'b', label='humand hand motion')
# ax.plot([1, 1+vx[0]], [1, 1+vx[1]], [1, 1+vx[2]], 'r-')
# ax.plot([1, 1+vy[0]], [1, 1+vy[1]], [1, 1+vy[2]], 'g-')
# ax.plot([1, 1+vz[0]], [1, 1+vz[1]], [1, 1+vz[2]], 'b-')
ax.plot([0, x_axes[0]], [0, x_axes[1]], [0, x_axes[2]], 'r-')
ax.plot([0, y_axes[0]], [0, y_axes[1]], [0, y_axes[2]], 'g-')
ax.plot([0, z_axes[0]], [0, z_axes[1]], [0, z_axes[2]], 'b-')



ux = np.array([1.0, 0.0, 0])
uy = np.array([0.0, 1.0, 0])
uz = np.array([0.0, 0.0, 1.0])
q = np.array([qresult[0], qresult[1], qresult[2], qresult[3]])
vx = rotVecByQuat(ux,q)
vy = rotVecByQuat(uy,q)
vz = rotVecByQuat(uz,q)

# ax.plot([1, 1+vx[0]], [1, 1+vx[1]], [1, 1+vx[2]], 'r:')
# ax.plot([1, 1+vy[0]], [1, 1+vy[1]], [1, 1+vy[2]], 'g:')
# ax.plot([1, 1+vz[0]], [1, 1+vz[1]], [1, 1+vz[2]], 'b:')

ax.plot([0, x_axes2[0]], [0, x_axes2[1]], [0, x_axes2[2]], 'r:')
ax.plot([0, y_axes2[0]], [0, y_axes2[1]], [0, y_axes2[2]], 'g:')
ax.plot([0, z_axes2[0]], [0, z_axes2[1]], [0, z_axes2[2]], 'b:')

rot = PyKDL.Rotation.Quaternion(qxori[3], qyori[3], qzori[3], qwori[3])
qresult =rot.GetQuaternion()
print(qresult)



# ux = np.array([1.0, 0.0, 0])
# uy = np.array([0.0, 1.0, 0])
# uz = np.array([0.0, 0.0, 1.0])
# q = np.array([qresult[0], qresult[1], qresult[2], qresult[3]])
# vx = rotVecByQuat(ux,q)
# vy = rotVecByQuat(uy,q)
# vz = rotVecByQuat(uz,q)

# ax.plot([1, 1+vx[0]], [1, 1+vx[1]], [1, 1+vx[2]], 'r--')
# ax.plot([1, 1+vy[0]], [1, 1+vy[1]], [1, 1+vy[2]], 'g--')
# ax.plot([1, 1+vz[0]], [1, 1+vz[1]], [1, 1+vz[2]], 'b--')


plt.show()

# x = []
# y = []
# z = []
# frames = []
# qx = []
# qy = []
# qz = []
# qw = []

# with open('trajectories/test/test-right-arm-motion-smooth'+str(number)+'-poses.csv') as csv_file:
#     print('trajectories/test/test-right-arm-motion-smooth'+str(number)+'-poses.csv')
#     csv_reader = csv.reader(csv_file, delimiter=' ')
#     line_count = 0
#     for row in csv_reader:
#         frames.append(float(row[0]))
#         x.append(float(row[1]))  
#         y.append(float(row[2]))
#         z.append(float(row[3]))
#         qw.append(float(row[7]))
#         qx.append(float(row[4]))
#         qy.append(float(row[5]))
#         qz.append(float(row[6]))
#         print(x[line_count], y[line_count], z[line_count])
#         line_count += 1
#     print(f'Processed {line_count} lines.')


# print(qxori[0], qyori[0], qzori[0], qwori[0])
# print(qx[0], qy[0], qz[0], qw[0])

