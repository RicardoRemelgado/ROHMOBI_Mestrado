# import libraries
import numpy as np
import matplotlib.pyplot as plt

# Points that define the Bezier Curves
P01 = [-36, 0, -75]
P02 = [0, 0, -45]
P03 = [50.9, 0, -58.1]
P04 = [36, 0, -75]
P11 = [36, 0, -75]
P12 = [0, 0, -80]
P13 = [-36, 0, -75]

# Dimensions
l1 = 50.9
l2 = 58.1

# Partial Transformation matrix
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


# Transformation matrix
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

# Plotting the points of the Bezier Curve

for t in np.arange(0, 1, 0.01, dtype=float):
    fig = plt.figure(t * 100)
    ax = plt.axes(projection='3d', title='Movimento das pernas do módulo bípede', xlabel='X (mm)', ylabel='Y (mm)', zlabel='Z (mm)')
    ax.view_init(200, 135)
    ax.set_xlim(-50, 100)
    ax.set_ylim(-50, 50)
    ax.set_zlim(0, -100)
    x0 = (1 - t) ** 3 * P01[0] + 3 * (1 - t) ** 2 * t * P02[0] + 3 * (1 - t) * t ** 2 * P03[0] + t ** 3 * P04[0]
    y0 = (1 - t) ** 3 * P01[1] + 3 * (1 - t) ** 2 * t * P02[1] + 3 * (1 - t) * t ** 2 * P03[1] + t ** 3 * P04[1]
    z0 = (1 - t) ** 3 * P01[2] + 3 * (1 - t) ** 2 * t * P02[2] + 3 * (1 - t) * t ** 2 * P03[2] + t ** 3 * P04[2]
    ax.scatter3D(x0, 38+y0, z0, s=10, color='red', alpha=1)
    x1 = (1 - t) ** 2 * P11[0] + 2 * (1 - t) * t * P12[0] + t ** 2 * P13[0]
    y1 = (1 - t) ** 2 * P11[1] + 2 * (1 - t) * t * P12[1] + t ** 2 * P13[1]
    z1 = (1 - t) ** 2 * P11[2] + 2 * (1 - t) * t * P12[2] + t ** 2 * P13[2]
    ax.scatter3D(x1, -38+y1, z1, s=10, color='red', alpha=1)

    ##inverse kinematics first leg

    r0 = np.sqrt(x0 ** 2 + y0 ** 2 + z0 ** 2)  # atenção, este cálculo considera sempre a origem no (0,0,0), apesar de ser usado no referencial 1
    # phi 1
    if x0 != 0 and y0 != 0:
        phi1_rad0 = np.arctan(y0 / x0)
    else:
        phi1_rad0 = 0
    phi10 = round(np.degrees(phi1_rad0), 2)
    # theta 2
    alpha2_rad0 = np.arccos((x0 / np.cos(phi1_rad0)) / r0)
    beta2_rad0 = np.arccos((l1 ** 2 + r0 ** 2 - l2 ** 2) / (2 * l1 * r0))
    theta2_rad0 = alpha2_rad0 - beta2_rad0
    theta20 = round(np.degrees(theta2_rad0), 2)
    # theta 3
    beta3_rad0 = np.arccos((l1 ** 2 + l2 ** 2 - r0 ** 2) / (2 * l1 * l2))
    theta3_rad0 = np.pi - beta3_rad0
    theta30 = round(np.degrees(theta3_rad0), 2)

    # Drawing the first leg
    p00 = [0, 38, 0]
    p01 = matrix_transform0_3(phi1_rad0, theta2_rad0, theta3_rad0)
    p02 = matrix_transform(phi1_rad0, theta2_rad0, theta3_rad0)
    ax.plot([p00[0], p01[0]], [p00[1], 38+p01[1]], [p00[2], -p01[2]], 'o-', c='green')
    ax.plot([p01[0], p02[0]], [38+p01[1], 38+p02[1]], [-p01[2], -p02[2]], 'o-',c='blue')  # o menos na coordenada dos z's é meramente corretivo pelo facto de o z me dar erroneamente positivo
    ##inverse kinematics second leg
    r1 = np.sqrt( x1 ** 2 + y1 ** 2 + z1 ** 2)  # atenção, este cálculo considera sempre a origem no (0,0,0), apesar de ser usado no referencial 1
    # phi 1
    if x1 != 0 and y1 != 0:
        phi1_rad1 = np.arctan(y1 / x1)
    else:
        phi1_rad1 = 0
    phi11 = round(np.degrees(phi1_rad1), 2)
    # theta 2
    alpha2_rad1 = np.arccos((x1 / np.cos(phi1_rad1)) / r1)
    beta2_rad1 = np.arccos((l1 ** 2 + r1 ** 2 - l2 ** 2) / (2 * l1 * r1))
    theta2_rad1 = alpha2_rad1 - beta2_rad1
    theta21 = round(np.degrees(theta2_rad1), 2)
    # theta 3
    beta3_rad1 = np.arccos((l1 ** 2 + l2 ** 2 - r1 ** 2) / (2 * l1 * l2))
    theta3_rad1 = np.pi - beta3_rad1
    theta31 = round(np.degrees(theta3_rad1), 2)

    # Drawing the second leg
    p10 = [0, -38, 0]
    p11 = matrix_transform0_3(phi1_rad1, theta2_rad1, theta3_rad1)
    p12 = matrix_transform(phi1_rad1, theta2_rad1, theta3_rad1)
    ax.plot([p10[0], p11[0]], [p10[1], -38 + p11[1]], [p10[2], -p11[2]], 'o-', c='green')
    ax.plot([p11[0], p12[0]], [-38 + p11[1], -38 + p12[1]], [-p11[2], -p12[2]], 'o-', c='blue')
    # Drawing the ground
    xx, yy = np.meshgrid(range(-50, 100, 1), range(-60, 60, 1))
    zz = (-75 - 0 * xx - 0 * yy) / 1
    ax.plot_surface(xx, yy, zz, alpha=0.5)
    ax.plot([0, 0], [-38, 38], [0, 0])
    ax.plot([0, 0], [0, 0], [0, 20], c='red')
    # Save the figure
    plt.savefig(r'C:\Universidade\10º Semestre - Tese de mestrado\Python code\3D Legs motion\ ' + str(int(t * 100)) + ".jpg")
    plt.close()

#fig2.show()

# Plotting the points again but in the reverse way
for t in np.arange(0, 1, 0.01, dtype=float):
    fig = plt.figure((t+1) * 100)
    ax = plt.axes(projection='3d', title='Movimento das pernas do módulo bípede', xlabel='X (mm)', ylabel='Y (mm)', zlabel='Z (mm)')
    ax.view_init(200, 135)
    ax.set_xlim(-50, 100)
    ax.set_ylim(-50, 50)
    ax.set_zlim(0, -100)
    x1 = (1 - t) ** 3 * P01[0] + 3 * (1 - t) ** 2 * t * P02[0] + 3 * (1 - t) * t ** 2 * P03[0] + t ** 3 * P04[0]
    y1 = (1 - t) ** 3 * P01[1] + 3 * (1 - t) ** 2 * t * P02[1] + 3 * (1 - t) * t ** 2 * P03[1] + t ** 3 * P04[1]
    z1 = (1 - t) ** 3 * P01[2] + 3 * (1 - t) ** 2 * t * P02[2] + 3 * (1 - t) * t ** 2 * P03[2] + t ** 3 * P04[2]
    ax.scatter3D(x1, -38+y1, z1, s=10, color='red', alpha=1)
    x0 = (1 - t) ** 2 * P11[0] + 2 * (1 - t) * t * P12[0] + t ** 2 * P13[0]
    y0 = (1 - t) ** 2 * P11[1] + 2 * (1 - t) * t * P12[1] + t ** 2 * P13[1]
    z0 = (1 - t) ** 2 * P11[2] + 2 * (1 - t) * t * P12[2] + t ** 2 * P13[2]
    ax.scatter3D(x0, 38+y0, z0, s=10, color='red', alpha=1)

    ##inverse kinematics second leg

    r1 = np.sqrt(x1 ** 2 + y1 ** 2 + z1 ** 2)  # atenção, este cálculo considera sempre a origem no (0,0,0), apesar de ser usado no referencial 1
    # phi 1
    if x1 != 0 and y1 != 0:
        phi1_rad1 = np.arctan(y1 / x1)
    else:
        phi1_rad1 = 0
    phi11 = round(np.degrees(phi1_rad1), 2)
    # theta 2
    alpha2_rad1 = np.arccos((x1 / np.cos(phi1_rad1)) / r1)
    beta2_rad1 = np.arccos((l1 ** 2 + r1 ** 2 - l2 ** 2) / (2 * l1 * r1))
    theta2_rad1 = alpha2_rad1 - beta2_rad1
    theta21 = round(np.degrees(theta2_rad1), 2)
    # theta 3
    beta3_rad1 = np.arccos((l1 ** 2 + l2 ** 2 - r1 ** 2) / (2 * l1 * l2))
    theta3_rad1 = np.pi - beta3_rad1
    theta31 = round(np.degrees(theta3_rad1), 2)

    # Drawing the first leg
    p10 = [0, -38, 0]
    p11 = matrix_transform0_3(phi1_rad1, theta2_rad1, theta3_rad1)
    p12 = matrix_transform(phi1_rad1, theta2_rad1, theta3_rad1)
    ax.plot([p10[0], p11[0]], [p10[1], -38+p11[1]], [p10[2], -p11[2]], 'o-', c='green')
    ax.plot([p11[0], p12[0]], [-38+p11[1], -38+p12[1]], [-p11[2], -p12[2]], 'o-',c='blue')  # o menos na coordenada dos z's é meramente corretivo pelo facto de o z me dar erroneamente positivo
    ##inverse kinematics first leg

    r0 = np.sqrt(
        x0 ** 2 + y0 ** 2 + z0 ** 2)  # atenção, este cálculo considera sempre a origem no (0,0,0), apesar de ser usado no referencial 1
    # phi 1
    if x0 != 0 and y0 != 0:
        phi1_rad0 = np.arctan(y0 / x0)
    else:
        phi1_rad0 = 0
    phi10 = round(np.degrees(phi1_rad0), 2)
    # theta 2
    alpha2_rad0 = np.arccos((x0 / np.cos(phi1_rad0)) / r0)
    beta2_rad0 = np.arccos((l1 ** 2 + r0 ** 2 - l2 ** 2) / (2 * l1 * r0))
    theta2_rad0 = alpha2_rad0 - beta2_rad0
    theta20 = round(np.degrees(theta2_rad0), 2)
    # theta 3
    beta3_rad0 = np.arccos((l1 ** 2 + l2 ** 2 - r0 ** 2) / (2 * l1 * l2))
    theta3_rad0 = np.pi - beta3_rad0
    theta30 = round(np.degrees(theta3_rad0), 2)

    # Drawing the second leg
    p00 = [0, 38, 0]
    p01 = matrix_transform0_3(phi1_rad0, theta2_rad0, theta3_rad0)
    p02 = matrix_transform(phi1_rad0, theta2_rad0, theta3_rad0)
    ax.plot([p00[0], p01[0]], [p00[1], 38 + p01[1]], [p00[2], -p01[2]], 'o-', c='green')
    ax.plot([p01[0], p02[0]], [38 + p01[1], 38 + p02[1]], [-p01[2], -p02[2]], 'o-', c='blue')  # o menos na coordenada dos z's é meramente corretivo pelo facto de o z me dar erroneamente positivo

    # Drawing the ground
    xx, yy = np.meshgrid(range(-50, 100, 1), range(-60, 60, 1))
    zz = (-75 - 0 * xx - 0 * yy) / 1
    ax.plot_surface(xx, yy, zz, alpha=0.5)
    ax.plot([0, 0], [-38, 38], [0, 0])
    ax.plot([0, 0], [0, 0], [0, 20], c='red')
    # Save the figure
    plt.savefig(r'C:\Universidade\10º Semestre - Tese de mestrado\Python code\3D Legs motion\ ' + str(int((t+1) * 100)) + ".jpg")
    plt.close()
