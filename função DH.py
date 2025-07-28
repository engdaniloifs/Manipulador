#função DH
import numpy as np
import matplotlib.pyplot as plt

def DH(a, alpha, d, theta):
    An = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha),a*np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0 ,0, 1]])
    
    return An

#x_e = 1
#y_e = -1

elo1 = 1
elo2 = 1


#D = (x_e**2 + y_e**2 - elo1**2 -elo2**2)/(2*elo1*elo2)

#theta2 = np.arctan2(np.sqrt(1-D**2),D )

#theta1 = np.arctan2(y_e, x_e) - np.arctan2( elo2*np.sin(theta2), elo1 + elo2*np.cos(theta2))


theta1 = np.deg2rad(45)
theta2 = np.deg2rad(45)

a1 = elo1
alpha1 = 0
d1 = 0

a2 = elo2
alpha2 = 0
d2 = 0
A1 = DH(a1,alpha1,d1,theta1)
A2 = DH(a2,alpha2,d2,theta2) 

T_0_1 = A1
T_0_2 = A1@A2

x1,y1 = T_0_1[0:2,3]
x2,y2 = T_0_2[0:2,3]

plt.figure()
plt.plot([0, x1, x2], [0,y1, y2], 'o-')  # 'o-' plota pontos conectados por linha
plt.title("Ligação entre junta 1 e junta 2")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.xlim(-elo1-elo2-3,+elo1+elo2+3)
plt.ylim(-elo1-elo2-3,+elo1+elo2+3)
plt.show()
