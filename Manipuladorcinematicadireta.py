import numpy as np
import matplotlib.pyplot as plt

class Joint:
    def __init__(self, link):
        self.link = link
        self.x = 0
        self.y = 0
        self.ang = 0
        self.angfuturo = 0
        
    def calc_pos(self, joint0, theta_deg):
        theta_rad = np.deg2rad(theta_deg)
        self.x = joint0.x + joint0.link * np.cos(theta_rad+joint0.ang)
        self.y = joint0.y + joint0.link * np.sin(theta_rad+joint0.ang)
        self.ang = theta_rad
        self.angfuturo = theta_rad+joint0.ang




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

    return xi,yi,xif,yif,xs,ys , xsf,ysf

# Criar juntas
joint0 = Joint(1)
joint1 = Joint(1)
ponto_final = Joint(0)

# Posição inicial da base
joint0.x = 0
joint0.y = 0



# Calcular posições
joint1.calc_pos(joint0, 0)
ponto_final.calc_pos(joint1,80)

xi, yi, xif, yif, xs, ys, xsf, ysf = garra(ponto_final)

# Plot
plt.figure()
plt.plot([joint0.x, joint1.x, ponto_final.x], [joint0.y, joint1.y, ponto_final.y], marker='o')
plt.plot([ponto_final.x, xi, xif], [ponto_final.y, yi, yif], marker='o')
plt.plot([ponto_final.x, xs, xsf], [ponto_final.y, ys, ysf], marker='o')
plt.xlim(-2, 2)
plt.ylim(-2, 2)
plt.grid(True)
plt.title("Manipulador Planar")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()
