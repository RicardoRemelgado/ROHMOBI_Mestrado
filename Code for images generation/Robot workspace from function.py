#import libraries
import numpy as np
import matplotlib.pyplot as plt
import statistics

#initial conditions
height = []
iteration = []
avg = []
l1 = 50.9
l2 = 58.1

#establishing the axes for the visualization graph
fig1 = plt.figure(1)
ax = plt.axes(projection='3d', xlabel= 'x [mm]', ylabel= 'y [mm]', zlabel= 'z [mm]')
ax.view_init(17, 115) #160,45


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
    z = - t04[2, 3]
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
    z = - t03[2, 3]
    return [x, y, z]

#Iteration through different angles values to obtain robot workspace
for phi1 in np.arange (-90, 90, 5, dtype=int):
    for theta2 in np.arange(-45, 135, 5, dtype=int):
        for theta3 in np.arange(0, 180, 5, dtype=int):
            phi1_rad = np.radians(phi1)
            theta2_rad = np.radians(theta2)
            theta3_rad = np.radians(theta3)
            x, y, z = matrix_transform(phi1_rad, theta2_rad, theta3_rad)
            height.append(z)
            ax.scatter3D(x, y, z, s = 10, color = 'red', alpha = 0.01) #0.05

#Drawing the robot leg
phi1_l = np.radians(45)
theta2_l = np.radians(45)
theta3_l = np.radians(45)
p0 = [0, 0, 0]
p1 = matrix_transform0_3(phi1_l, theta2_l, theta3_l)
p2 = matrix_transform(phi1_l, theta2_l, theta3_l)
ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], 'o-', c = 'green')
ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'o-', c = 'blue')

'''
#Plot height graph with mean
fig2 = plt.figure(2)
iteration = np.arange(0, len(height), 1)
mean = statistics.mean(height)
for i in np.arange(0, len(height), 1):
    avg.append(mean)
plt.plot(iteration, height)
plt.plot(iteration, avg)
plt.legend(["Height", "Height average"])
plt.xlabel('Iterations')
plt.ylabel('Height')
plt.title('End effector height')

#Plot histogram of height of the end effector
fig3 = plt.figure(3)
#histogram, frequency = np.histogram(height, bins = np.arange(min(height), max(height), 1))
#plt.plot(histogram)
plt.hist(height, bins = np.arange(min(height), max(height), 1))
plt.xlabel('Height')
plt.ylabel('Frequency')
plt.title('End effector height')
'''

#print('Average height: ', mean)
print('Number of iterations: ', len(height))
print(p2)
#print('Most common height value: ', ...)

plt.show()