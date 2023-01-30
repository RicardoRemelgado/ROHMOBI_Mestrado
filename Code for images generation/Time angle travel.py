import numpy as np
import matplotlib.pyplot as plt

# Points that define the Bezier Curves
P01 = [-36, 0, -75]
P02 = [0, 0, -45]
P03 = [50.9, 0, -58.1]
P04 = [36, 0, -75]
P11 = [36, 0, -75]
P12 = [-18, 0, -80]
P13 = [-36, 0, -75]

# Dimensions
l1 = 50.9
l2 = 58.

#List of joint angles in different phases
time = []
totaltime = []
swingtheta2 = []
swingtheta3 = []
suptheta2 = []
suptheta3 = []
theta2 = []
theta3 = []


#Swing phase
for t in np.arange(0, 1, 0.01, dtype=float):
    x0 = (1 - t) ** 3 * P01[0] + 3 * (1 - t) ** 2 * t * P02[0] + 3 * (1 - t) * t ** 2 * P03[0] + t ** 3 * P04[0]
    y0 = (1 - t) ** 3 * P01[1] + 3 * (1 - t) ** 2 * t * P02[1] + 3 * (1 - t) * t ** 2 * P03[1] + t ** 3 * P04[1]
    z0 = (1 - t) ** 3 * P01[2] + 3 * (1 - t) ** 2 * t * P02[2] + 3 * (1 - t) * t ** 2 * P03[2] + t ** 3 * P04[2]

    ##inverse kinematics swing phase
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
    time.append(t)
    totaltime.append(t)
    swingtheta2.append(theta20)
    swingtheta3.append(theta30)
    theta2.append(theta20)
    theta3.append(theta30)

#Support phase
for t in np.arange(0, 1, 0.01, dtype=float):
    x1 = (1 - t) ** 2 * P11[0] + 2 * (1 - t) * t * P12[0] + t ** 2 * P13[0]
    y1 = (1 - t) ** 2 * P11[1] + 2 * (1 - t) * t * P12[1] + t ** 2 * P13[1]
    z1 = (1 - t) ** 2 * P11[2] + 2 * (1 - t) * t * P12[2] + t ** 2 * P13[2]

    ##inverse kinematics support phase
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

    totaltime.append(t+1)
    suptheta2.append(theta21)
    suptheta3.append(theta31)
    theta2.append(theta21)
    theta3.append(theta31)

print('Time: ', time)
print('Total time', totaltime)
print('Swing theta 2: ', swingtheta2)
print('Swing theta 3: ', swingtheta3)
print('Support theta 2: ', suptheta2)
print('Support theta 3: ', suptheta3)

fig1, axs = plt.subplots(2, 2)
axs[0, 0].plot(time, swingtheta2)
axs[0, 0].set_title('Theta2 na fase de balanço')
axs[0, 1].plot(time, swingtheta3, 'tab:orange')
axs[0, 1].set_title('Theta3 na fase de balanço')
axs[1, 0].plot(time, suptheta2, 'tab:green')
axs[1, 0].set_title('Theta 2 na fase de suporte')
axs[1, 1].plot(time, suptheta3, 'tab:red')
axs[1, 1].set_title('Theta 3 na fase de suporte')

axs[0, 0].set(ylabel='Ângulo [º]')
axs[1, 0].set(xlabel='Tempo', ylabel='Ângulo [º]')
axs[1, 1].set(xlabel='Tempo')

#Combine the swing and support phase
fig2, (ax1, ax2) = plt.subplots(2)
ax1.plot(totaltime, theta2)
ax1.set_title('Theta 2')
ax1.set( ylabel='Ângulo [º]')
ax1.label_outer()
ax2.plot(totaltime, theta3)
ax2.set_title('Theta 3')
ax2.set(xlabel='Tempo', ylabel='Ângulo [º]')

# Fit a second degree polynomial to the curves of the timed angle variation
from numpy import arange
from scipy.optimize import curve_fit

fig3, axs3 = plt.subplots(2,2)

#theta 2 in swing phase curve and curve fitted
axs3[0, 0].plot(time, swingtheta2, alpha= 0.4)
axs3[0, 0].set_title('Theta 2 na fase de balanço')
def objective(x, a, b, c, d):
    return a * x + b * x ** 2 + c * x ** 3 + d
    ## curve fit
popt, _ = curve_fit(objective, time, swingtheta2)
    ## summarize the parameter values
a, b, c, d = popt
print('y = %.3f * x + %.3f * x^2 + %.3f * x^3 + %.3f' % (a, b, c, d))
    ## define a sequence of inputs between the smallest and largest known inputs
time_line = arange(min(time), max(time), 0.01)
    ## calculate the output for the range
swingtheta2_line = objective(time_line, a, b, c, d)
    ## create a line plot for the mapping function
axs3[0, 0].plot(time_line, swingtheta2_line, '--', color='red')
axs3[0, 0].text(0.20, 30, 'y = %.3f * x + %.3f * x^2 + %.3f * x^3 + %.3f' % (a, b, c, d))

#theta 3 in swingphase curve and curve fitted
axs3[0, 1].plot(time, swingtheta3, 'tab:orange', alpha=0.4)
axs3[0, 1].set_title('Theta 3 na fase de balanço')
def objective(x, a, b, c, d):
    return a * x + b * x ** 2 + c * x ** 3 + d
    ## curve fit
popt, _ = curve_fit(objective, time, swingtheta3)
    ## summarize the parameter values
a, b, c, d = popt
print('y = %.3f * x + %.3f * x^2 + %.3f * x ** 3 + %.3f' % (a, b, c, d))
    ## define a sequence of inputs between the smallest and largest known inputs
time_line = arange(min(time), max(time), 0.01)
    ## calculate the output for the range
swingtheta3_line = objective(time_line, a, b, c, d)
    ## create a line plot for the mapping function
axs3[0, 1].plot(time_line, swingtheta3_line, '--', color='red')
axs3[0, 1].text(0, 85, 'y = %.3f * x + %.3f * x^2 + %.3f * x ** 3 + %.3f' % (a, b, c, d))

#theta 2 in support phase curve and curve fitted
axs3[1, 0].plot(time, suptheta2, 'tab:green', alpha=0.4)
axs3[1, 0].set_title('Theta 2 na fase de suporte')
def objective(x, a, b, c):
    return a * x + b * x ** 2 + c
    ## curve fit
popt, _ = curve_fit(objective, time, suptheta2)
    ## summarize the parameter values
a, b, c = popt
print('y = %.3f * x + %.3f * x^2 + %.3f' % (a, b, c))
    ## define a sequence of inputs between the smallest and largest known inputs
time_line = arange(min(time), max(time), 0.01)
    ## calculate the output for the range
suptheta2_line = objective(time_line, a, b, c)
    ## create a line plot for the mapping function
axs3[1, 0].plot(time_line, suptheta2_line, '--', color='red')
axs3[1, 0].text(0.2, 36, 'y = %.3f * x + %.3f * x^2 + %.3f' % (a, b, c))

#theta 3 in support phase curve and curve fitted
axs3[1, 1].plot(time, suptheta3, 'tab:red', alpha=0.4)
axs3[1, 1].set_title('Theta 3 na fase de suporte')
def objective(x, a, b, c, d):
    return a * x + b * x ** 2 + c * x ** 3 + d
    ## curve fit
popt, _ = curve_fit(objective, time, suptheta3)
    ## summarize the parameter values
a, b, c, d = popt
print('y = %.3f * x + %.3f * x^2 + %.3f * x^3 + %.3f' % (a, b, c, d))
    ## define a sequence of inputs between the smallest and largest known inputs
time_line = arange(min(time), max(time), 0.01)
    ## calculate the output for the range
suptheta3_line = objective(time_line, a, b, c, d)
    ## create a line plot for the mapping function
axs3[1, 1].plot(time_line, suptheta3_line, '--', color='red')
axs3[1, 1].text(0, 92, 'y = %.3f * x + %.3f * x^2 + %.3f * x^3 + %.3f' % (a, b, c, d))

axs3[0, 0].set(ylabel='Ângulo [º]')
axs3[1, 0].set(xlabel='Tempo', ylabel='Ângulo [º]')
axs3[1, 1].set(xlabel='Tempo')

#just swing phase
fig4, (ax41, ax42) = plt.subplots(2)
ax41.plot(time, swingtheta2)
#ax41.set_title('Theta2 na fase de balanço')
ax41.set(ylabel='Ângulo [º]')
ax41.legend(["Theta2"])
ax42.plot(time, swingtheta3, 'tab:orange')
#ax42.set_title('Theta3 na fase de balanço')
ax42.set(xlabel='Tempo [s]', ylabel='Ângulo [º]')
ax42.legend(["Theta3"])

#just support phase
fig5, (ax51, ax52) = plt.subplots(2)
ax51.plot(time, suptheta2)
#ax51.set_title('Theta2 na fase de suporte')
ax51.set(ylabel='Ângulo [º]')
ax51.legend(["Theta 2"])
ax52.plot(time, suptheta3, 'tab:orange')
#ax52.set_title('Theta3 na fase de suporte')
ax52.set(xlabel='Tempo [s]', ylabel='Ângulo [º]')
ax52.legend(["Theta 3"])

#swing and support phase in the same graph
fig6, (ax61, ax62) = plt.subplots(2)
ax61.plot(time, swingtheta2)
ax61.plot(time, suptheta2, 'tab:green')
#ax61.set_title('Theta2')
ax61.legend(["Fase de balanço Theta 2", "Fase de suporte Theta 2"])
ax61.set(ylabel='Ângulo [º]')
ax62.plot(time, swingtheta3, 'tab:orange')
ax62.plot(time, suptheta3, 'tab:red')
#ax62.set_title('Theta3')
ax62.legend(["Fase de balanço Theta 3", "Fase de suporte Theta 3"])
ax62.set(xlabel='Tempo [s]', ylabel='Ângulo [º]')

print('Max swing theta2: ', max( swingtheta2))
print('Min swing theta2: ', min( swingtheta2))
print('Time of min swing theta 2: ', swingtheta2.index(min( swingtheta2)))
print('Max swing theta3: ', max( swingtheta3))
print('Min swing theta3: ', min( swingtheta3))
print('Time of max swing theta 3: ', swingtheta3.index(max( swingtheta3)))
print('Max support theta2: ', max( suptheta2))
print('Min support theta2: ', min( suptheta2))
print('Medium support theta 2: ', suptheta2[50])
print('Max support theta3: ', max( suptheta3))
print('Min support theta3: ', min( suptheta3))
print('Medium support theta 3: ', suptheta3[50])

plt.show()

