#import libraries
import numpy as np
import matplotlib.pyplot as plt
from numpy import arange
from scipy.optimize import curve_fit

#Graph cracteristics
fig1 = plt.figure(1)
ax = plt.axes(projection='3d', xlabel= 'x [mm]', ylabel= 'y [mm]', zlabel= 'z [mm]')
ax.view_init(20,-45)

fig2 = plt.figure(2)
ax2 = plt.axes()

#Leg dimensions
l1 = 50.9
l2 = 58.1

#Angles arrays
Phi1 =[]
Theta2 = []
Theta3 = []
X = []
Y = []
Z = []

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

#angle
angle = float(input('Introduce angle of rotation: '))
#angle of increment
inc_angle = angle/3

#Angles for Bezier Curve
A1 = [0*inc_angle, 72.07, 80.71]
A2 = [1*inc_angle, 15.71, 131.79]
A3 = [2*inc_angle, 0, 90]
A4 = [3*inc_angle, 20.79, 80.71]

P1 = matrix_transform(np.radians(A1[0]), np.radians(A1[1]), np.radians(A1[2]))
P2 = matrix_transform(np.radians(A2[0]), np.radians(A2[1]), np.radians(A2[2]))
P3 = matrix_transform(np.radians(A3[0]), np.radians(A3[1]), np.radians(A3[2]))
P4 = matrix_transform(np.radians(A4[0]), np.radians(A4[1]), np.radians(A4[2]))

print(A1, A2, A3, A4)
print(P1, P2, P3, P4)

for t in np.arange (0, 1, 0.01, dtype=float):
    x = (1 - t) ** 3 * P1[0] + 3 * (1 - t) ** 2 * t * P2[0] + 3 * (1 - t) * t ** 2 * P3[0] + t ** 3 * P4[0]
    y = (1 - t) ** 3 * P1[1] + 3 * (1 - t) ** 2 * t * P2[1] + 3 * (1 - t) * t ** 2 * P3[1] + t ** 3 * P4[1]
    z = (1 - t) ** 3 * P1[2] + 3 * (1 - t) ** 2 * t * P2[2] + 3 * (1 - t) * t ** 2 * P3[2] + t ** 3 * P4[2]

    X.append(x)
    Y.append(y)
    Z.append(z)
    #inverse kinematics
    r = np.sqrt(x ** 2 + y ** 2 + z ** 2)  # atenção, este cálculo considera sempre a origem no (0,0,0), apesar de ser usado no referencial 1
    # phi 1
    if x != 0 and y != 0:
        phi1_rad = np.arctan((y-P1[1]) / (x-P1[0]))
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
    #index in array
    Phi1.append(phi1)
    Theta2.append(theta2)
    Theta3.append(theta3)

    ax.scatter3D(x, y, -z, s=10, color='red', alpha=1) #é necessário fazer a correção do -z, pq tem tudo de passar pela transformação da cinemática direta que dá o z como positivo


# Drawing the robot leg
p0 = [0, 0, 0]
p1 = matrix_transform0_3(np.radians(A1[0]), np.radians(A1[1]), np.radians(A1[2]))
p2 = matrix_transform(np.radians(A1[0]), np.radians(A1[1]), np.radians(A1[2]))
ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], -p1[2]], 'o-', c = 'green')
ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [-p1[2], -p2[2]], 'o-', c = 'blue') #o menos na coordenada dos z's é meramente corretivo pelo facto de o z me dar erroneamente positivo

'''
#Drawing the points of the Bezier Curve
ax.plot([P1[0], P2[0]], [P1[1], P2[1]], [-P1[2], -P2[2]])
ax.plot([P2[0], P3[0]], [P2[1], P3[1]], [-P2[2], -P3[2]])
ax.plot([P3[0], P4[0]], [P3[1], P4[1]], [-P3[2], -P4[2]])
'''

#Curve fitted
ax2.plot(X, Y, alpha= 0.4)
ax2.set_title('Plane Bezier Curve')
def objective(x, a, b, c, d, e):
    return a * x + b * x ** 2 + c * x ** 3 + d * x ** 4 + e
    ## curve fit
popt, _ = curve_fit(objective, X, Y)
    ## summarize the parameter values
a, b, c, d, e = popt
print('y = %.3f * x + %.3f * x^2 + %.3f * x^3 + %.3f * x^4 + %.3f ' % (a, b, c, d, e))
    ## define a sequence of inputs between the smallest and largest known inputs
X_line = arange(min(X), max(X), 0.01)
    ## calculate the output for the range
Y_line = objective(X_line, a, b, c, d, e)
    ## create a line plot for the mapping function
ax2.plot(X_line, Y_line, '--', color='red')
ax2.text(-36, 15, 'y = %.5f * x + %.5f * x^2 + %.5f * x^3 + %.5f * x^4 + %.5f' % (a, b, c, d, e))

print('Phi1: ', Phi1)
print('Theta2: ', Theta2)
print('Theta3: ', Theta3)
print('x: ', X)
print('y: ', Y)
print('z: ', Z)


plt.grid()
plt.show()