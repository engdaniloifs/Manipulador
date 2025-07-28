####################################################################################
#                                                                                  #
#                 LEMBRE-SE QUE A SIMULAÇÃO DEVE ESTAR EM EXECUÇÃO!                #
#                                                                                  #
####################################################################################
import numpy as np
import matplotlib.pyplot as plt
import random
import math
import matplotlib.patches as patches


try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

NUM_NODES = 500
STEP_SIZE = 1

atan2 = np.arctan2


def readSensorData(clientId=-1, 
                    range_data_signal_id="hokuyo_range_data", 
                    angle_data_signal_id="hokuyo_angle_data"):

    # the first call should be non-blocking to avoid getting out-of-sync angle data
    returnCodeRanges, string_range_data = sim.simxGetStringSignal(clientId, range_data_signal_id, sim.simx_opmode_streaming)

    # the second call should block to avoid out-of-sync scenarios
    # between your python script and the simulator's main loop
    # (your script may be slower than the simulator's main loop, thus
    # slowing down data processing)
    returnCodeAngles, string_angle_data = sim.simxGetStringSignal(clientId, angle_data_signal_id, sim.simx_opmode_blocking)

    # check the if both data were obtained correctly
    if returnCodeRanges == 0 and returnCodeAngles == 0:
        # unpack data from range and sensor messages
        raw_range_data = sim.simxUnpackFloats(string_range_data)
        raw_angle_data = sim.simxUnpackFloats(string_angle_data)

        return raw_range_data, raw_angle_data

    # return none in case were nothing was gotten from the simulator
    return None

def get_obstacles():
    obstacles = []
    index = 1
    INFLATION_SIZE = 0.5  # Ajuste essa constante para definir a margem de segurança

    while True:
        obj_name = f'obstacle{index}'
        result, obj_handle = sim.simxGetObjectHandle(clientID, obj_name, sim.simx_opmode_blocking)
        if result != sim.simx_return_ok:
            break  # Sai do loop se não encontrar mais obstáculos

        # Obter posição (x, y) do obstáculo
        _, pos = sim.simxGetObjectPosition(clientID, obj_handle, -1, sim.simx_opmode_blocking)

        # Obter orientação (yaw) do obstáculo
        _, euler_angles = sim.simxGetObjectOrientation(clientID, obj_handle, -1, sim.simx_opmode_blocking)
        yaw = euler_angles[2]  # Rotação em torno do eixo Z

        # Obter dimensões do bounding box
        _, min_x = sim.simxGetObjectFloatParameter(clientID, obj_handle, 15, sim.simx_opmode_blocking)
        _, max_x = sim.simxGetObjectFloatParameter(clientID, obj_handle, 18, sim.simx_opmode_blocking)
        _, min_y = sim.simxGetObjectFloatParameter(clientID, obj_handle, 16, sim.simx_opmode_blocking)
        _, max_y = sim.simxGetObjectFloatParameter(clientID, obj_handle, 19, sim.simx_opmode_blocking)

        width = max_x - min_x
        height = max_y - min_y

        # Inflar os obstáculos
        width += 2 * INFLATION_SIZE
        height += 2 * INFLATION_SIZE

        # Definir os quatro cantos do bounding box antes da rotação
        #half_w, half_h = width / 2, height / 2
        corners = np.array([
            [min_x, min_y],
            [max_x, min_y],
            [max_x, max_y],
            [min_x, max_y]
        ])

        # Criar matriz de rotação
        rotation_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])

        # Rotacionar os cantos do bounding box
        rotated_corners = np.dot(corners, rotation_matrix.T)

        # Obter as coordenadas corrigidas
        new_min_x = np.min(rotated_corners[:, 0]) + pos[0]
        new_max_x = np.max(rotated_corners[:, 0]) + pos[0]
        new_min_y = np.min(rotated_corners[:, 1]) + pos[1]
        new_max_y = np.max(rotated_corners[:, 1]) + pos[1]

        new_w = new_max_x - new_min_x
        new_h = new_max_y - new_min_y
        #new_w += 2 * INFLATION_SIZE
        #new_h += 2 * INFLATION_SIZE

        new_ox = ((new_max_x + new_min_x) / 2) 
        new_oy = ((new_max_y + new_min_y) / 2) 

        print(new_ox,new_oy,width,height)
        

        obstacles.append(((new_ox, new_oy), (width, height), yaw))
        index += 1

    return obstacles

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

def generate_random_point(xlim, ylim):
    return random.uniform(*xlim), random.uniform(*ylim)

def nearest_node(tree, point):
    return min(tree, key=lambda node: distance(node, Node(*point)))

def new_node(nearest, point, step_size):
    angle = math.atan2(point[1] - nearest.y, point[0] - nearest.x)
    x = nearest.x + step_size * math.cos(angle)
    y = nearest.y + step_size * math.sin(angle)
    return Node(x, y, parent=nearest)

def is_in_obstacle(point):
    px, py = point
    # Para cada obstáculo, assumimos que OBSTACLES contém:
    # ((ox, oy), (w, h), theta)
    # onde (ox, oy) é o centro do obstáculo, (w, h) suas dimensões e
    # theta o ângulo de rotação em radianos.
    for (ox, oy), (w, h), theta in OBSTACLES:
        # 1. Translada o ponto para o referencial do obstáculo.
        dx = px - ox
        dy = py - oy

        # 2. Rotaciona o ponto para "desalinhar" o obstáculo.
        # Rotacionamos pelo ângulo negativo para voltar ao sistema não rotacionado.
        cos_theta = math.cos(-theta)
        sin_theta = math.sin(-theta)
        local_x = dx * cos_theta - dy * sin_theta
        local_y = dx * sin_theta + dy * cos_theta

        # 3. Verifica se o ponto está dentro dos limites do retângulo.
        if abs(local_x) <= w / 2 and abs(local_y) <= h / 2:
            return True

    return False


def build_rrt(start, goal, xlim, ylim):
    tree = [Node(*start)]
    plt.figure(figsize=(5, 5))
    plt.plot(*start, 'go', markersize=12, label="Início")
    plt.plot(*goal, 'ro', markersize=12, label="Objetivo")
    plt.xlim(xlim)
    plt.ylim(ylim)
    plt.grid(True)
    plt.title("RRT em Progresso")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()

    

    ax = plt.gca()
    for (ox, oy), (w, h), theta in OBSTACLES:
        # Converter theta de radianos para graus
        angle_degrees = math.degrees(theta)
        
        # Criar o retângulo com o canto inferior esquerdo calculado a partir do centro.
        rect = patches.Rectangle((ox - w/2, oy - h/2), w, h, color='gray', alpha=0.5)
        
        # Criar uma transformação que rotaciona o retângulo em torno do centro (ox, oy)
        transform = patches.transforms.Affine2D().rotate_deg_around(ox, oy, angle_degrees) + ax.transData
        rect.set_transform(transform)
        
        ax.add_patch(rect)

    for iteration in range(1, NUM_NODES + 1):
        random_point = generate_random_point(xlim, ylim)

        if is_in_obstacle(random_point):
            continue

        nearest = nearest_node(tree, random_point)
        new = new_node(nearest, random_point, STEP_SIZE)

        if is_in_obstacle((new.x, new.y)):
            continue


        tree.append(new)

        # Visualiza o progresso
        plt.plot([nearest.x, new.x], [nearest.y, new.y], color='black')  # Nós visitados em preto
        plt.plot(new.x, new.y, 'go', markersize=2)  # Nós novos em verde
        plt.pause(0.02)

        path = get_path(new)
        print(f"Iteração: {iteration}")  # Imprime no terminal


        if distance(new, Node(*goal)) < STEP_SIZE:
            goal_node = Node(*goal, parent=new)
            tree.append(goal_node)
            return tree, goal_node, iteration

    return tree, None, NUM_NODES


def get_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]
    
def low_pass_filter(path, alpha):
    smoothed_path = [path[0]]  # O primeiro ponto permanece o mesmo

    for i in range(1, len(path) - 1):  # Não inclui o último ponto na suavização
        new_x = alpha * path[i][0] + (1 - alpha) * smoothed_path[i - 1][0]
        new_y = alpha * path[i][1] + (1 - alpha) * smoothed_path[i - 1][1]
        smoothed_path.append((new_x, new_y))

    smoothed_path.append(path[-1])  # Mantém o último ponto inalterado

    return smoothed_path

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def distance(node1, node2):
    return math.hypot(node1.x - node2.x, node1.y - node2.y)

def calcular_thetaobs(laser_data, frente, theta_atual):

    ang, dist = laser_data[frente - 20]
    xi = dist * np.cos(ang + theta_atual)
    yi = dist * np.sin(ang + theta_atual)
    
    ang, dist = laser_data[frente]
    xf = dist * np.cos(ang + theta_atual)
    yf = dist * np.sin(ang + theta_atual)
    
    thetaobs = np.arctan2(yf - yi, xf - xi)
    
    return thetaobs


    

# Função principal para desenhar a trajetória

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')
    
    # Iniciando a simulação
    # Deve usar a porta do 'continuous remote API server services' (remoteApiConnections.txt)
    # e = sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

    # Handle para o ROBÔ    
    robotname = 'Pioneer_p3dx'
    returnCode, robotHandle = sim.simxGetObjectHandle(clientID, robotname, sim.simx_opmode_oneshot_wait)     
    
    # Handle para as juntas das RODAS
    returnCode, l_wheel = sim.simxGetObjectHandle(clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
    returnCode, r_wheel = sim.simxGetObjectHandle(clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)    
    
    laser_range_data = "hokuyo_range_data"
    laser_angle_data = "hokuyo_angle_data"

    returnCode = 1
    while returnCode != 0:
        returnCode, range_data = sim.simxGetStringSignal(clientID, laser_range_data, sim.simx_opmode_streaming + 10)
    
    returnCode, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)        
    print('Pos: ', pos)
    

    xlim,ylim = (-4.5,4.5),(-4.5,4.5)
    start = (pos[0], pos[1])
    goal = (3.5, -3)
    OBSTACLES = get_obstacles()
    #raise SystemExit()
   
    tree, goal_node, iterations = build_rrt(start, goal, xlim, ylim)
        
    if goal_node:
        path = get_path(goal_node)
    else:
        path = [], float('inf')

    if goal_node:
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            plt.plot([x1, x2], [y1, y2], 'r-', linewidth=2)  # Linha vermelha para o caminho
            plt.plot(x1, y1, 'ro', markersize=5)  # Ponto atual destacado                
            plt.pause(0.5)
        plt.plot(path[-1][0], path[-1][1], 'ro', markersize=5,label="Caminho sem filtro")  # Último ponto
        plt.legend()
        plt.title(f"RRT Concluído - Iterações: {iterations}")
        plt.show(block=False)
    else:
        plt.title(f"RRT Falhou - Iterações: {iterations}")
        plt.pause(1)

    time.sleep(2)


    

    plt.show()

    
    

    # ---------------------------
    # Criação da figura com dois subplots
    # ---------------------------
    fig, (ax_traj, ax_angle) = plt.subplots(2, 1, figsize=(5, 6))

    # --- Subplot 1: Trajetória Planejada ---
    ax_traj.plot(*start, 'go', markersize=4, label="Início")
    ax_traj.plot(*goal, 'ro', markersize=4, label="Objetivo")
    ax_traj.set_xlim(xlim)
    ax_traj.set_ylim(ylim)

        # Traçar o caminho planejado:
    x1, y1 = start
    x2, y2 = path[1]  # O primeiro nó
    ax_traj.plot([x1, x2], [y1, y2], 'k-', linewidth=2, label="Trajetória planejada")
    ax_traj.plot(x2, y2, 'ko', markersize=2)

# Plotar os segmentos intermediários:
    for i in range(1, len(path)-1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        ax_traj.plot([x1, x2], [y1, y2], 'k-', linewidth=2)
        ax_traj.plot(x1, y1, 'ko', markersize=2)
        # Se preferir uma animação mais lenta, pode usar plt.pause(0.002) aqui
        plt.pause(0.002)

    # Conectar o último nó ao objetivo:
    x1, y1 = path[-2]
    x2, y2 = goal
    ax_traj.plot([x1, x2], [y1, y2], 'k-', linewidth=2)
    ax_traj.plot(x2, y2, 'ro', markersize=2)
    # Opcional: criar a linha para a trajetória real (caso queira atualizar este subplot também)
    line_real, = ax_traj.plot([], [], 'b-', label="Trajetória Real", linewidth=1)

    real_trajectory = []
    

    ax_traj.legend()
    ax_traj.legend(prop={'size': 8})
    ax_traj.set_title("Rastreabilidade")

    # --- Subplot 2: Gráfico do Ângulo do Robô por Tempo ---
    ax_angle.set_xlim(0, 200)  # tempo de 0 a 10 segundos, ajuste conforme necessário
    ax_angle.set_ylim(-np.pi, np.pi)
    ax_angle.set_yticks([
        -np.pi, 
        -3*np.pi/4, 
        -np.pi/2, 
        -np.pi/4, 
        0, 
        np.pi/4, 
        np.pi/2, 
        3*np.pi/4, 
        np.pi
    ])
    ax_angle.set_yticklabels([
        r'$-\pi$', 
        r'$-\frac{3\pi}{4}$', 
        r'$-\frac{\pi}{2}$', 
        r'$-\frac{\pi}{4}$', 
        '0', 
        r'$\frac{\pi}{4}$', 
        r'$\frac{\pi}{2}$', 
        r'$\frac{3\pi}{4}$', 
        r'$\pi$'
    ])
    ax_angle.set_xlabel("Tempo (s)")
    ax_angle.set_ylabel("Ângulo (rad)")
    ax_angle.set_title("Orientação do Robô por Tempo")
    ax_angle.grid(True)

    # Linha que será atualizada dinamicamente com os dados do ângulo
    line_angle, = ax_angle.plot([], [], 'b-', label="Ângulo trajetória",linewidth=1)  # Azul

    time_data, angle_data = [], []


    
    ax_angle.legend()
    ax_angle.legend(prop={'size': 8})

    plt.tight_layout()
    plt.show(block=False)
        

    raw_range_data, raw_angle_data = readSensorData(clientID, laser_range_data, laser_angle_data)
    laser_data = np.array([raw_angle_data, raw_range_data]).T
    frente = int(len(laser_data)/2)

    start = int( len(laser_data) / 1.71)  # Garante que seja um número inteiro
    stop = len(laser_data) - 1
    mediadistesquerda = 0
    indice1 = 1

    # Dados do Pioneer
    L = 0.381   # Metros
    r = 0.0975  # Metros

    maxv = .8
    maxw = np.deg2rad(45)

    maxwbug = np.deg2rad(30)
    maxvbug = 0.5
    
    t = 0
    # Lembrar de habilitar o 'Real-time mode'


    
    

    while True:
        user_input = input("Digite 'run' para continuar: ")
        
        # Verifica se o comando é 'run'
        if user_input.lower() == 'run':
            print("Execução continuada!")
            break  # Sai do loop e continua a execução
        else:
            print("Comando inválido. Tente novamente.")

    start_time = time.time()

    modbug = 0
    u = 0
    p = 0
    flag = 0

    rho = np.inf
    while(rho > 0.1):
        for indice, (x_goal, y_goal) in enumerate(path):
        # Obtém a posição e orientação do robô no início de cada novo objetivo

                
            rho = np.inf
                # Calcula a distância até o objetivo
                
                
                

            if (x_goal, y_goal) == (goal):
                d = 0.1
            else:
                d = 0.5

            

            while rho > d:  # Ajuste do threshold
                    # Obtém a posição e orientação do robô novamente a cada iteração
                returnCode, robotPos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
                returnCode, robotOri = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)

                #sim.simx_opmode_streaming
                if (flag != 0 ):
                    flag -= 1 
                    break

                    # Atualiza a configuração do robô
                robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])
                real_trajectory.append((robotConfig[0], robotConfig[1]))
                    # Calcula a distância (erro) em relação ao objetivo

                current_time = time.time() - start_time
                time_data.append(current_time)
                angle_data.append(robotConfig[2])

                line_angle.set_data(time_data, angle_data)
                    

                real_x = [pos[0] for pos in real_trajectory]
                real_y = [pos[1] for pos in real_trajectory]
                line_real.set_data(real_x, real_y)  # Atualiza a linha da trajetória real
                
                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.05)  # Ajuste a pausa conforme necessário
                            
                dx, dy = x_goal - robotConfig[0], y_goal - robotConfig[1]
                rho = np.sqrt(dx**2 + dy**2)

                    # Controladores de ganho
                kp = 0.8
                ka = 4


                    # Calcula a velocidade linear (v) e angular (w)
                v = kp * (dx * np.cos(robotConfig[2]) + dy * np.sin(robotConfig[2]))
                w = ka * normalize_angle(np.arctan2(dy, dx) - robotConfig[2]) 

                    # Limita as velocidades para os valores máximos permitidos
                v = max(min(v, maxv), -maxv)
                w = max(min(w, maxw), -maxw)


                raw_range_data, raw_angle_data = readSensorData(clientID, laser_range_data, laser_angle_data)
                laser_data = np.array([raw_angle_data, raw_range_data]).T

                if laser_data[frente, 1] < 1:
                    

                    tetaatual = robotOri[2]
                    
                    v = 0
                    w = 0
                    modbug = 1
                    ang, dist = laser_data[frente]
                    xi = dist * np.cos(ang + tetaatual)
                    yi = dist * np.sin(ang + tetaatual)
                    
                    ang, dist = laser_data[frente-20]
                    xf = dist * np.cos(ang + tetaatual)
                    yf = dist * np.sin(ang + tetaatual)
                    
                    tetaparede = atan2(yf-yi, xf-xi)

                    print(tetaparede)

                    
                    rho = 0

                    # Modelo cinemático: calcula as velocidades das rodas
                wl = v / r - (w * L) / (2 * r)
                wr = v / r + (w * L) / (2 * r)

                    # Envia as velocidades para as rodas
                sim.simxSetJointTargetVelocity(clientID, l_wheel, wl, sim.simx_opmode_streaming + 5)
                sim.simxSetJointTargetVelocity(clientID, r_wheel, wr, sim.simx_opmode_streaming + 5)

                

            while modbug == 1:

                returnCode, robotPos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
                returnCode, robotOri = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
                    # Atualiza a configuração do robô
                robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])

                real_trajectory.append((robotConfig[0], robotConfig[1]))
                    # Calcula a distância (erro) em relação ao objetivo

                current_time = time.time() - start_time
                time_data.append(current_time)
                angle_data.append(robotConfig[2])

                line_angle.set_data(time_data, angle_data)
                    

                real_x = [pos[0] for pos in real_trajectory]
                real_y = [pos[1] for pos in real_trajectory]
                line_real.set_data(real_x, real_y)  # Atualiza a linha da trajetória real
                
                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.05)  # Ajuste a pausa conforme necessário

                raw_range_data, raw_angle_data = readSensorData(clientID, laser_range_data, laser_angle_data)
                laser_data = np.array([raw_angle_data, raw_range_data]).T

                for u in range(start,stop):
                    mediadistesquerda += laser_data[u, 1]
                    indice1 += 1  # Corrigido para incrementar corretamente
                mediadistesquerda /= indice1
                    
                distancia_desejadaesquerda = 2
                erro = mediadistesquerda - distancia_desejadaesquerda  

                distancia_atualfrente = laser_data[frente,1]
                distancia_desejadafrente = 0.7
                erro1 = distancia_desejadafrente - distancia_atualfrente

                kabug = 1
                kpbug = 0.36
                w = kabug*erro
                v = -kpbug*erro1
            

                w = max(min(w, maxwbug), -maxwbug) 
                v = max(min(v, maxvbug), -maxvbug)

                mediadistesquerda = 0
                indice1 = 1

                if (abs(normalize_angle(tetaparede + np.deg2rad(90))-robotConfig[2]) <= np.deg2rad(10)):
                    v = 0
                    w = 0
                    #flag = 1
                    modbug = 0
                    

                wl = v / r - (w * L) / (2 * r)
                wr = v / r + (w * L) / (2 * r)

                    # Envia as velocidades para as rodas
                sim.simxSetJointTargetVelocity(clientID, l_wheel, wl, sim.simx_opmode_streaming + 5)
                sim.simxSetJointTargetVelocity(clientID, r_wheel, wr, sim.simx_opmode_streaming + 5)
            
            dx, dy = goal[0] - robotConfig[0], goal[1] - robotConfig[1]
            rho = np.sqrt(dx**2 + dy**2)

            
            # Aqui, o robô alcançou o primeiro ponto. Agora, ele começará a ir para o próximo ponto.     
        
            

        # Parando o robô    
    sim.simxSetJointTargetVelocity(clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait) 
    print("Posição final")      
    print(robotConfig)
    sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)   
    plt.show()
        
    # Parando a simulação     
    time.sleep(2)

             
        


else:
    print ('Failed connecting to remote API server')

 
sim.simxFinish(clientID)
    
print ('Program ended')