#import libraries
import numpy as np
import matplotlib.pyplot as plt

#Points that define the Bezier Curve
P1 = [0, 0, -75]
P2 = [0, 0, -45]
P3 = [50.9, 0, -58.1]
P4 = [36, 0, -75]

#Dimensions
l1 = 50.9
l2 = 58.

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

#Transformation matrix
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

#Plotting the points of the Bezier Curve
for t in np.arange (0, 1, 0.01, dtype=float):
    plt.figure(t*100)
    ax = plt.axes(title='Trajectory of leg in swing phase', xlabel='X axis (mm)', ylabel='Z axis (mm)')
    ax.set_xlim(0, 100)
    ax.set_ylim(0, -100)
    ax.invert_yaxis()
    x = (1 - t) ** 3 * P1[0] + 3 * (1 - t) ** 2 * t * P2[0] + 3 * (1 - t) * t ** 2 * P3[0] + t ** 3 * P4[0]
    y = 0
    z = (1 - t) ** 3 * P1[2] + 3 * (1 - t) ** 2 * t * P2[2] + 3 * (1 - t) * t ** 2 * P3[2] + t ** 3 * P4[2]
    ax.scatter(x, z, s=10, color='red', alpha=1)

    ##inverse kinematics
    r = np.sqrt(x ** 2 + y ** 2 + z ** 2)  # atenção, este cálculo considera sempre a origem no (0,0,0), apesar de ser usado no referencial 1
    # phi 1
    if x != 0 and y != 0:
        phi1_rad = np.arctan(y / x)
    else:
        phi1_rad = 0
    phi1 = round(np.degrees(phi1_rad), 2)
    # theta 2
    alpha2_rad = np.arccos((x / np.cos(phi1_rad)) / r)
    beta2_rad = np.arccos((l1 ** 2 + r ** 2 - l2 ** 2) / (2 * l1 * r))
    theta2_rad = alpha2_rad - beta2_rad
    theta2 = round(np.degrees(theta2_rad), 2)
    # theta 3
    beta3_rad = np.arccos((l1 ** 2 + l2 ** 2 - r ** 2) / (2 * l1 * l2))
    theta3_rad = np.pi - beta3_rad
    theta3 = round(np.degrees(theta3_rad), 2)

    #Drawing the leg
    p0 = [0, 0, 0]
    p1 = matrix_transform0_3(phi1_rad, theta2_rad, theta3_rad)
    p2 = matrix_transform(phi1_rad, theta2_rad, theta3_rad)
    ax.plot([p0[0], p1[0]], [p0[2], -p1[2]], 'o-', c='green')
    ax.plot([p1[0], p2[0]], [-p1[2], -p2[2]], 'o-', c='blue')#o menos na coordenada dos z's é meramente corretivo pelo facto de o z me dar erroneamente positivo

    #Drawing the ground
    ax.plot([0, 100], [-75, -75], '-', c='red')

    #Save the fgure
    plt.savefig(r'C:\Universidade\10º Semestre - Tese de mestrado\Python code\2D Images of swing leg motion\ ' + str(int(t*100)) + ".jpg")
    plt.close()


#no figure 29, don't know why
