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

#Create figure with the 2 Bezier Curves
fig, ax = plt.subplots(1)
ax.set_title('Curva de Bezier na fase de balanço e suporte')
ax.set(xlabel= 'x [mm]', ylabel= 'z [mm]')

#Travel
fig2, (ax21,ax22) = plt.subplots(2)
#ax21.set_title('Deslocamento em x na fase de balanço e suporte')
ax21.set(ylabel= 'x [mm]')
#ax22.set_title('Deslocamento em z na fase de balanço e suporte')
ax22.set(xlabel= 'Tempo [s]', ylabel= 'z [mm]')

#Lists to plot
X0 = []
Y0 = []
Z0 = []
X1 = []
Y1 = []
Z1 = []
T = []

for t in np.arange(0, 1, 0.01, dtype=float):
    x0 = (1 - t) ** 3 * P01[0] + 3 * (1 - t) ** 2 * t * P02[0] + 3 * (1 - t) * t ** 2 * P03[0] + t ** 3 * P04[0]
    y0 = (1 - t) ** 3 * P01[1] + 3 * (1 - t) ** 2 * t * P02[1] + 3 * (1 - t) * t ** 2 * P03[1] + t ** 3 * P04[1]
    z0 = (1 - t) ** 3 * P01[2] + 3 * (1 - t) ** 2 * t * P02[2] + 3 * (1 - t) * t ** 2 * P03[2] + t ** 3 * P04[2]
    x1 = (1 - t) ** 2 * P11[0] + 2 * (1 - t) * t * P12[0] + t ** 2 * P13[0]
    y1 = (1 - t) ** 2 * P11[1] + 2 * (1 - t) * t * P12[1] + t ** 2 * P13[1]
    z1 = (1 - t) ** 2 * P11[2] + 2 * (1 - t) * t * P12[2] + t ** 2 * P13[2]
    X0.append(x0)
    Y0.append(y0)
    Z0.append(z0)
    X1.append(x1)
    Y1.append(y1)
    Z1.append(z1)
    T.append(t)

ax.plot(X0, Z0)
ax.plot(X1, Z1)
ax.legend(["Fase de balanço", "Fase de suporte"])

ax21.plot(T, X0)
ax21.plot(T, X1)
ax21.legend(["Fase de balanço ∆x", "Fase de suporte ∆x"])

ax22.plot(T, Z0)
ax22.plot(T, Z1)
ax22.legend(["Fase de balanço ∆z", "Fase de suporte ∆z"])

ax.grid()
ax21.grid()
ax22.grid()
plt.show()
