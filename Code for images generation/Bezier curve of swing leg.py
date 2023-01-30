#import libraries
import numpy as np
import matplotlib.pyplot as plt

#Graph cracteristics
fig1 = plt.figure(1)
ax = plt.axes(projection='3d', title= 'Trajectory of leg in swing phase', xlabel= 'X axis (mm)', ylabel= '    Y', zlabel= 'Z axis (mm)')
ax.view_init(20,-45)

#Drawing in 2D
fig2, axs = plt.subplots(1)
#axs.set_title('Curva de Bezier e pontos de controlo da fase de balanço')
axs.set(xlabel= 'x [mm]', ylabel= 'z [mm]')
plt.xlim(-5, 55)
plt.ylim(-80, -20)
plt.autoscale(False)

#Points
P1 = [0, 0, -75]
P2 = [0, 0, -45]
P3 = [50.9, 0, -58.1]
P4 = [36, 0, -75]

for t in np.arange (0, 1, 0.01, dtype=float):
    x = (1 - t) ** 3 * P1[0] + 3 * (1 - t) ** 2 * t * P2[0] + 3 * (1 - t) * t ** 2 * P3[0] + t ** 3 * P4[0]
    y = (1 - t) ** 3 * P1[1] + 3 * (1 - t) ** 2 * t * P2[1] + 3 * (1 - t) * t ** 2 * P3[1] + t ** 3 * P4[1]
    z = (1 - t) ** 3 * P1[2] + 3 * (1 - t) ** 2 * t * P2[2] + 3 * (1 - t) * t ** 2 * P3[2] + t ** 3 * P4[2]
    ax.scatter3D(x, y, z, s=10, color='red', alpha=1)
    axs.scatter(x, z, s= 17, color='red', alpha=1)

#Transformation matrix function-Denavit Hartenberg modified
def matrix_transform(phi1_rad, theta2_rad, theta3_rad):
    t01 = np.eye(4)
    t01[0, 0] = np.cos(phi1_rad)
    t01[0, 1] = -np.sin(phi1_rad)
    t01[1, 0] = np.sin(phi1_rad)
    t01[1, 1] = np.cos(phi1_rad)
    t12 = np.zeros((4, 4))
    t12[0, 0] = np.cos(theta2_rad)
    t12[0, 1] = -np.sin(theta2_rad)
    t12[2, 0] = np.sin(theta2_rad)
    t12[2, 1] = np.cos(theta2_rad)
    t12[1, 2] = -1
    t12[3, 3] = 1
    t23 = np.eye(4)
    t23[0, 0] = np.cos(theta3_rad)
    t23[0, 1] = -np.sin(theta3_rad)
    t23[1, 0] = np.sin(theta3_rad)
    t23[1, 1] = np.cos(theta3_rad)
    t23[0, 3] = l1
    t34 = np.eye(4)
    t34[0, 3] = l2
    t02 = np.dot(t01, t12)
    t03 = np.dot(t02, t23)
    t04 = np.dot(t03, t34)
    x = t04[0, 3]
    y = t04[1, 3]
    z = t04[2, 3]
    return [x, y, z]

#Partial Transformation matrix
def matrix_transform0_3(phi1_rad, theta2_rad, theta3_rad):
    t01 = np.eye(4)
    t01[0, 0] = np.cos(phi1_rad)
    t01[0, 1] = -np.sin(phi1_rad)
    t01[1, 0] = np.sin(phi1_rad)
    t01[1, 1] = np.cos(phi1_rad)
    t12 = np.zeros((4, 4))
    t12[0, 0] = np.cos(theta2_rad)
    t12[0, 1] = -np.sin(theta2_rad)
    t12[2, 0] = np.sin(theta2_rad)
    t12[2, 1] = np.cos(theta2_rad)
    t12[1, 2] = -1
    t12[3, 3] = 1
    t23 = np.eye(4)
    t23[0, 0] = np.cos(theta3_rad)
    t23[0, 1] = -np.sin(theta3_rad)
    t23[1, 0] = np.sin(theta3_rad)
    t23[1, 1] = np.cos(theta3_rad)
    t23[0, 3] = l1
    t02 = np.dot(t01, t12)
    t03 = np.dot(t02, t23)
    x = t03[0, 3]
    y = t03[1, 3]
    z = t03[2, 3]
    return [x, y, z]

# Drawing the robot leg
l1 = 50.9
l2 = 58.1
phi1_l = np.radians(0)
theta2_l = np.radians(20.79)
theta3_l = np.radians(80.71)
p0 = [0, 0, 0]
p1 = matrix_transform0_3(phi1_l, theta2_l, theta3_l)
p2 = matrix_transform(phi1_l, theta2_l, theta3_l)
ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], -p1[2]], 'o-', c = 'green')
ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [-p1[2], -p2[2]], 'o-', c = 'blue') #o menos na coordenada dos z's é meramente corretivo pelo facto de o z me dar erroneamente positivo

#Drawing in the leg an the Bezier curve in 2D
axs.plot([P1[0], P2[0], P3[0],P4[0]], [P1[2], P2[2], P3[2], P4[2]]) #curve
axs.text(2, -75, 'P1(0, 0, -75)')
axs.text(2, -45, 'P2(0, 0, -45)')
axs.text(40, -55.0, 'P3(50.9, 0, -58.1)')
axs.text(24, -75, 'P4(36, 0, -75)')
#axs.plot([p0[0], p1[0]], [p0[2], -p1[2]], 'o-', c = 'green')
#axs.plot([p1[0], p2[0]], [-p1[2], -p2[2]], 'o-', c = 'blue')
plt.grid()

plt.show()