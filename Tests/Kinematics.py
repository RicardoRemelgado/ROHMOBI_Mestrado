#import libraries
import numpy as np

#characteristics of leg
l1 = 50.9
l2 = 58.1

#Kinematics calculation
while True:
    resposta = input('Deseja calcular a cinemática? s ou n: ')
    #Direct kinematics
    if resposta == 's' :
        type = input("Qual a cinemática que pretende fazer? d ou i: ")
        if type == 'd':
            while True:
                phi1 = float(input("Ângulo phi1 [-90, 90]: "))
                if phi1 < -90 or phi1 > 90:
                    print('Ângulo da junta fora dos limites!')
                    print('Insira o ângulo outra vez')
                else:
                    break
            while True:
                theta2 = float(input("Ângulo theta2 [-45, 135]: "))
                if theta2 < -45 or theta2 > 135:
                    print('Ângulo da junta fora dos limites!')
                    print('Insira o ângulo outra vez')
                else:
                    break
            while True:
                theta3 = float(input("Ângulo theta3 [0, 180]: "))
                if theta3 < 0 or theta3 > 180:
                    print('Ângulo da junta fora dos limites!')
                    print('Insira o ângulo outra vez')
                else:
                    break
            phi1_rad = np.radians(phi1)
            theta2_rad = np.radians(theta2)
            theta3_rad = np.radians(theta3)
            x = round(np.cos(phi1_rad) * (l2 * (np.cos(theta2_rad + theta3_rad)) + l1 * np.cos(theta2_rad)), 2)
            y = round(np.sin(phi1_rad) * (l2 * (np.cos(theta2_rad + theta3_rad)) + l1 * np.cos(theta2_rad)), 2)
            z = round(l2 * np.sin(theta2_rad + theta3_rad) + l1 * np.sin(theta2_rad), 2)
            print('x= ', x, 'mm')
            print('y= ', y, 'mm')
            print('z= ', z, 'mm')
        #Inverse Kinematics
        else:
            while True:
                x = float(input("Coordenada do end effector em x: "))
                y = float(input("Coordenada do end effector em y: "))
                z = float(input("Coordenada do end effector em z: "))
                r = np.sqrt(x**2 + y**2 + z**2) #atenção, este cálculo considera sempre a origem no (0,0,0), apesar de ser usado no referencial 1
                # phi 1
                if x != 0 and y != 0:
                    phi1_rad = np.arctan(y / x)
                else:
                    phi1_rad = 0
                phi1 = round(np.degrees(phi1_rad), 2)
                # theta 2
                alpha2_rad = np.arccos((x/np.cos(phi1_rad)) / r)
                beta2_rad = np.arccos((l1**2 + r**2 - l2**2) / (2 * l1 * r))
                theta2_rad = alpha2_rad - beta2_rad
                theta2 = round(np.degrees(theta2_rad), 2)
                # theta 3
                beta3_rad = np.arccos((l1**2 + l2**2 - r**2) / (2 * l1 * l2))
                theta3_rad = np.pi - beta3_rad
                theta3 = round(np.degrees(theta3_rad), 2)
                if (-90 < phi1 > 90) or (-45 < theta2 > 135) or (0 < theta3 > 180):
                    print('A posição do end effector está fora do espaço de trabalho')
                    print('Introduza uma nova posição do end effector')
                else:
                    print('Ângulo phi1 =', phi1, 'º')
                    print('Ângulo theta2 =', theta2, 'º')
                    print('Ângulo theta3 =', theta3, 'º')
                    break
    else:
        break