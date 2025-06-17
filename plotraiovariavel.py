#Plot raio variável, cinemática inversa.

import numpy as np
import matplotlib.pyplot as plt

class Joint:
    def __init__(self, link):
        self.link = link
        self.x = 0
        self.y = 0
        self.ang = 0
        self.angfuturo = 0
        
    def calc_pos(self, joint0, theta):
        #theta_rad = np.deg2rad(theta)
        self.x = joint0.x + joint0.link * np.cos(theta+joint0.ang)
        self.y = joint0.y + joint0.link * np.sin(theta+joint0.ang)
        self.ang = theta
        self.angfuturo = theta+joint0.ang




def garra(ponto_final):
    linkgarra = 0.2
    xi = ponto_final.x + linkgarra*np.cos(ponto_final.angfuturo -np.pi/2)
    yi = ponto_final.y + linkgarra*np.sin(ponto_final.angfuturo -np.pi/2)

    xif = xi + linkgarra*np.cos(ponto_final.angfuturo)
    yif = yi + linkgarra*np.sin(ponto_final.angfuturo)

    xs = ponto_final.x + linkgarra*np.cos(ponto_final.angfuturo +np.pi/2)
    ys = ponto_final.y + linkgarra*np.sin(ponto_final.angfuturo +np.pi/2)

    xsf = xs + linkgarra*np.cos(ponto_final.angfuturo)
    ysf = ys + linkgarra*np.sin(ponto_final.angfuturo)

    return xi,yi,xif,yif,xs,ys,xsf,ysf




# Criar juntas
elo1 = 1
elo2 = 1

n_pontos = 180
passo_angulo = (2*np.pi)/n_pontos
raio = 1.5
path = []


for i in range (n_pontos):
    x = raio * np.cos(passo_angulo * i)
    y = raio * np.sin(passo_angulo * i)
    path.append((x, y))  

    


joint0 = Joint(elo1)
joint1 = Joint(elo2)
ponto_final = Joint(0)

# Posição inicial da base
joint0.x = 0
joint0.y = 0

for i in range(n_pontos):
    xe,ye = path[i]


    theta2 = -np.arccos((xe**2+ye**2-elo1**2-elo2**2)/(2*elo1*elo2))

    k1 = elo1 + elo2*np.cos(theta2)


    k2 = elo2*np.sin(theta2)



    theta1 = (np.arctan2(ye,xe) - np.arctan2(k2,k1))


    # Calcular posições
    joint1.calc_pos(joint0, theta1)
    ponto_final.calc_pos(joint1,theta2)

    xi, yi, xif, yif, xs, ys, xsf, ysf = garra(ponto_final)







    # Plot
    
    plt.plot([joint0.x, joint1.x, ponto_final.x], [joint0.y, joint1.y, ponto_final.y],marker='o')
    plt.plot([ponto_final.x, xi, xif], [ponto_final.y, yi, yif], color = 'blue')
    plt.plot([ponto_final.x, xs, xsf], [ponto_final.y, ys, ysf],color = 'blue')
    
    plt.axis('equal')
    plt.xlim(-elo1 -elo2 - 3, elo1 + elo2 + 3)
    plt.ylim(-elo1 -elo2 - 3, elo1 + elo2 +3)
    plt.grid(True)
    plt.title("Manipulador Planar")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.pause(0.5)
    plt.cla()
