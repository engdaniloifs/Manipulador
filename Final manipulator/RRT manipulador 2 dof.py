#RRT manipulador 2 dof
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button
import random
import matplotlib.patches as patches
import serial
import time

class Node:
    def __init__(self, theta1, theta2, parent=None):
        self.theta1 = theta1
        self.theta2 = theta2
        self.parent = parent

def generate_random_point(theta1_lim, theta2_lim):
    return random.uniform(*theta1_lim), random.uniform(*theta2_lim)

def nearest_node(tree, point):
    distancemin = float('inf')
    near_node = None
    for i in range(len(tree)):
        distanceatual = distance(tree[i], Node(*point))
        if (distancemin > distanceatual):
            distancemin = distanceatual
            near_node = tree[i]
    
    return near_node

def new_node(nearest, point, step_size):
    angle = np.arctan2(point[1] - nearest.theta2, point[0] - nearest.theta1)
    theta1 = nearest.theta1 + step_size * np.cos(angle)
    theta2 = nearest.theta2 + step_size * np.sin(angle)
    return Node(theta1, theta2, parent=nearest)


def is_in_obstacle(point,OBSTACLES):
    px, py = point
    for (ox, oy), (w, h) in OBSTACLES:
        if abs(px - ox) <= w / 2 and abs(py - oy) <= h / 2:
            return True
    return False


def distance(node1, node2):
    return np.hypot(node1.theta1 - node2.theta1, node1.theta2 - node2.theta2)


def edge_collision_check(p1, p2, num_points, OBSTACLES):
    """
    Check for collisions along a line segment from p1 to p2,
    including p1 and p2 themselves.
    """
    x1, y1 = p1
    x2, y2 = p2
    for i in range(0, num_points + 2):  # includes 0 and num_points+1
        t = i / (num_points + 1)
        x = (1 - t) * x1 + t * x2
        y = (1 - t) * y1 + t * y2
        if is_in_obstacle((x, y), OBSTACLES):
            return True
    return False


def build_rrt(start, goal, theta1_lim, theta2_lim,OBSTACLES,NUM_NODES, STEP_SIZE,a1,alpha1,d1,a2,alpha2,d2):
    tree = [Node(*start)]
    plt.figure(figsize=(5, 5))
    plt.plot(*start, 'go', markersize=12, label="Início")
    plt.plot(*goal, 'ro', markersize=12, label="Objetivo")
    plt.xlim(theta1_lim)
    plt.ylim(theta2_lim)
    plt.grid(True)
    plt.title("RRT em Progresso")
    plt.xlabel(r"$\theta_1$")
    plt.ylabel(r"$\theta_2$")
    plt.legend()



    for iteration in range(1, NUM_NODES + 1):
        random_point = generate_random_point(theta1_lim, theta2_lim)

        

        nearest = nearest_node(tree, random_point)
        new = new_node(nearest, random_point, STEP_SIZE)
        # Forward kinematics
        A1 = DH(a1, alpha1, d1, new.theta1)
        A2 = DH(a2, alpha2, d2, new.theta2)
        T_0_2 = A1 @ A2
        x_new, y_new = T_0_2[0:2, 3]

        # Get end-effector of nearest node
        A1_near = DH(a1, alpha1, d1, nearest.theta1)
        A2_near = DH(a2, alpha2, d2, nearest.theta2)
        T_0_2_near = A1_near @ A2_near
        x_near, y_near = T_0_2_near[0:2, 3]

        if edge_collision_check((x_near, y_near), (x_new, y_new), num_points=5, OBSTACLES=OBSTACLES):
            continue  # reject edge



        tree.append(new)

        # Visualiza o progresso
        plt.plot([nearest.theta1, new.theta1], [nearest.theta2, new.theta2], color='black')  # Nós visitados em preto
        plt.plot(new.theta1, new.theta2, 'go', markersize=2)  # Nós novos em verde
        plt.pause(0.02)

        #path = get_path(new)
        print(f"Iteração: {iteration}")  # Imprime no terminal


        if distance(new, Node(*goal)) < STEP_SIZE:
            goal_node = Node(*goal, parent=new)
            tree.append(goal_node)
            return goal_node, iteration

    plt.close()
    build_rrt(start, goal, theta1_lim, theta2_lim,OBSTACLES,NUM_NODES, STEP_SIZE,a1,alpha1,d1,a2,alpha2,d2)


def get_path(node):
    path = []
    while node:
        path.append((node.theta1, node.theta2))
        node = node.parent
    return path[::-1]

def get_start_and_goal(elo1,elo2,theta1_lim,theta2_lim):
    # Storage for input values
    
    # --- Prepare the workspace plot ---
    

    theta1_vals = np.linspace(theta1_lim[0], theta1_lim[1], 50)
    theta2_vals = np.linspace(theta2_lim[0], theta2_lim[1], 50)

    x_vals = []
    y_vals = []

    for theta1 in theta1_vals:
        for theta2 in theta2_vals:
            x1 = elo1 * np.cos(theta1)
            y1 = elo1 * np.sin(theta1)
            x2 = x1 + elo2 * np.cos(theta1 + theta2)
            y2 = y1 + elo2 * np.sin(theta1 + theta2)
            x_vals.append(x2)
            y_vals.append(y2)

    # --- Create figure and axes ---
    fig = plt.figure(figsize=(8, 6))
    ax_plot = plt.axes([0.1, 0.2, 0.65, 0.7])  # workspace plot area
    ax_plot.scatter(x_vals, y_vals, s=1, color='blue', label='Reachable Area')
    ax_plot.set_title("Reachable Workspace of 2DOF Arm with Limited Joint Angles")
    ax_plot.set_xlabel("x")
    ax_plot.set_ylabel("y")
    ax_plot.set_xlim(-13,13)
    ax_plot.set_ylim(-13,13)
    ax_plot.set_xticks(np.arange(-14, 14, 2))  # for x-axis
    ax_plot.set_yticks(np.arange(-14, 14, 2))  # for y-axis
    #ax_plot.axis("equal")
    ax_plot.grid(True)
    # --- Text boxes for input ---
    tb_start_x = TextBox(plt.axes([0.85, 0.75, 0.05, 0.05]), "Start x")
    tb_start_y = TextBox(plt.axes([0.85, 0.68, 0.05, 0.05]), "Start y")
    tb_goal_x = TextBox(plt.axes([0.85, 0.55, 0.05, 0.05]), "Goal x")
    tb_goal_y = TextBox(plt.axes([0.85, 0.48, 0.05, 0.05]), "Goal y")

    start = None
    goal = None

    # --- Confirmation button ---
    def on_confirm(event):
        nonlocal start, goal
        try:
            
            sx = float(tb_start_x.text)
            sy = float(tb_start_y.text)
            gx = float(tb_goal_x.text)
            gy = float(tb_goal_y.text)
            start = (sx, sy)
            goal = (gx, gy)

            
            plt.close()
        except ValueError:
            print("Invalid input. Please enter valid numbers.")

    btn_ax = plt.axes([0.8, 0.35, 0.1, 0.05])
    button = Button(btn_ax, "OK")
    button.color = "green"
    button.ax.set_facecolor("green")
    button.label.set_color("white")
    button.hovercolor = "darkgreen"
    button.on_clicked(on_confirm)

    plt.show()
    return start, goal

def get_obstacles():
    # Store obstacle here
    obstacle_input = []

    # --- First window: ask how many obstacles ---
    def on_confirm_obstacle_number(event):
        nonlocal obstacles_number
        try:
            obstacles_number = int(textbox_n.text)
            if 0 <= obstacles_number <= 3:
                plt.close()
            else:
                print("Please enter a number between 0 and 3.")
        except ValueError:
            print("Invalid input, please enter an integer.")
    
    obstacles_number = None
    fig1 = plt.figure(figsize=(6, 1))
    ax_n = plt.axes([0.5, 0.5, 0.05, 0.2])
    textbox_n = TextBox(ax_n, "Enter number of obstacles (0–3)")
    button_ax_OK = plt.axes([0.6, 0.5, 0.1, 0.2])
    button_OK = Button(button_ax_OK, "OK")
    button_OK.color = "green"
    button_OK.ax.set_facecolor("green")
    button_OK.label.set_color("white")
    button_OK.hovercolor = "darkgreen"
    button_OK.on_clicked(on_confirm_obstacle_number)

    plt.show()

    if obstacles_number == 0 or obstacles_number is None:
        return []  # user closed or entered 0

    # --- Second window: enter obstacle parameters ---
    def on_confirm_obstacle_params(event):
        nonlocal obstacle_input
        obstacle_input = []
        inflation = 0.3  # ← Fixed inflation constant
        try:
            for boxes in textboxes:
                xc = float(boxes[0].text)
                yc = float(boxes[1].text)
                w = float(boxes[2].text)
                h = float(boxes[3].text)
                inflated_w = w + 2 * inflation
                inflated_h = h + 2 * inflation
                obstacle_input.append(((xc, yc), (inflated_w, inflated_h)))
            plt.close()
        except ValueError:
            print("Invalid input. Please enter numbers in all fields.")

    fig2 = plt.figure(figsize=(6, 4))
    fig2.suptitle("Enter obstacle parameters", fontsize=14)
    fig2.text(0.5, 0.9, "xc = center x | yc = center y | w = width | h = height", 
              ha='center', fontsize=12, color='gray')

    textboxes = []
    for i in range(obstacles_number):
        y_base = 0.75 - i * 0.12
        boxes = []
        for j, label in enumerate(['xc', 'yc', 'w', 'h']):
            ax_box = plt.axes([0.3 + j * 0.1, y_base, 0.06, 0.05])
            tb = TextBox(ax_box, label)
            boxes.append(tb)
        textboxes.append(boxes)

    button_ax_OK2 = plt.axes([0.8, 0.5, 0.1, 0.1])
    button_OK2 = Button(button_ax_OK2, "OK")
    button_OK2.color = "green"
    button_OK2.ax.set_facecolor("green")
    button_OK2.label.set_color("white")
    button_OK2.hovercolor = "darkgreen"
    button_OK2.on_clicked(on_confirm_obstacle_params)

    plt.show()

    return obstacle_input



def DH(a, alpha, d, theta):
    An = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha),a*np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0 ,0, 1]])
    
    return An

def inverse_kinematics(point, elo1, elo2):
    x_e, y_e = point
    D = (x_e**2 + y_e**2 - elo1**2 - elo2**2) / (2 * elo1 * elo2)

    # Clip to avoid numerical errors
    D = np.clip(D, -1.0, 1.0)

    # Two possible theta2 values
    theta2_options = [
        np.arctan2(np.sqrt(1 - D**2), D),   # elbow-up
        np.arctan2(-np.sqrt(1 - D**2), D)   # elbow-down
    ]

    # Try both and select valid one
    for theta2 in theta2_options:
        theta1 = np.arctan2(y_e, x_e) - np.arctan2(elo2 * np.sin(theta2), elo1 + elo2 * np.cos(theta2))

        # Normalize theta1 to [0, 2π]
        if theta1 < 0:
            theta1 += 2 * np.pi
        
        # Check joint limits
        if 0 <= theta1 <= np.pi and -np.pi/2 <= theta2 <= np.pi/2:
            return theta1, theta2

    # If none valid
    print("No valid IK solution within joint limits for point:", point)
    input("stop")
    return None, None


def main():

    theta1_lim,theta2_lim = (0,np.pi),(-np.pi/2,np.pi/2)

    elo1 = 7.2
    elo2 = 4.9

    a1 = elo1
    alpha1 = 0
    d1 = 0

    a2 = elo2
    alpha2 = 0
    d2 = 0

    start, goal = get_start_and_goal(elo1,elo2,theta1_lim,theta2_lim)

    #print(start)
    #print(goal)
    theta_start = inverse_kinematics(start, elo1,elo2)

    theta_goal = inverse_kinematics(goal, elo1,elo2)
    
    NUM_NODES = 1000
    STEP_SIZE = np.deg2rad(10)
    OBSTACLES = get_obstacles()

    goal_node, iterations = build_rrt(theta_start, theta_goal, theta1_lim, theta2_lim,OBSTACLES,NUM_NODES, STEP_SIZE,a1,alpha1,d1,a2,alpha2,d2)


    if goal_node:
        path_angle = get_path(goal_node)
    else:
        path_angle = [], float('inf')
        
    if goal_node:
        for i in range(len(path_angle) - 1):
            theta1_1, theta2_1 = path_angle[i]
            theta1_2, theta2_2 = path_angle[i + 1]
            plt.plot([theta1_1, theta1_2], [theta2_1, theta2_2], 'r-', linewidth=2)  # Linha vermelha para o caminho
            plt.plot(theta1_1, theta2_1, 'ro', markersize=5)  # Ponto atual destacado                
            plt.pause(0.5)
        plt.plot(path_angle[-1][0], path_angle[-1][1], 'ro', markersize=5,label="Caminho final")  # Último ponto
        plt.legend()
        plt.title(f"RRT Concluído - Iterações: {iterations}")
        plt.show()

    plt.figure(figsize=(5, 5))
    theta1, theta2 = path_angle[0]

    A1 = DH(a1,alpha1,d1,theta1)
    A2 = DH(a2,alpha2,d2,theta2) 

    #T_0_1 = A1
    T_0_2 = A1@A2
    #x1,y1 = T_0_1[0:2,3]
    x,y = T_0_2[0:2,3]
    ax = plt.gca()
    for (ox, oy), (w, h) in OBSTACLES:
        
        
        # Criar o retângulo com o canto inferior esquerdo calculado a partir do centro.
        rect = patches.Rectangle((ox - w/2, oy - h/2), w, h, color='gray', alpha=0.5)
        
        
        ax.add_patch(rect)

    plt.plot(x, y, 'go', markersize=12, label="Start")
    plt.xlim(-13,13)
    plt.ylim(-6,13)
    plt.grid(True)
    plt.title("Path no plano cartesiano")
    plt.xlabel("x")
    plt.ylabel("y")
    theta1, theta2 = path_angle[1]
    A1 = DH(a1,alpha1,d1,theta1)
    A2 = DH(a2,alpha2,d2,theta2) 

    #T_0_1 = A1
    T_0_2 = A1@A2
    #x1,y1 = T_0_1[0:2,3]
    x1,y1 = T_0_2[0:2,3]


    plt.plot([x, x1], [y, y1], 'r-', linewidth=2)  # Linha vermelha para o caminho
    plt.legend()

    for i in range(1,len(path_angle)-1):
        theta1, theta2 = path_angle[i]
        A1 = DH(a1,alpha1,d1,theta1)
        A2 = DH(a2,alpha2,d2,theta2) 

        #T_0_1 = A1
        T_0_2 = A1@A2
        #x1,y1 = T_0_1[0:2,3]
        x1,y1 = T_0_2[0:2,3]

        theta1, theta2 = path_angle[i+1]
        A1 = DH(a1,alpha1,d1,theta1)
        A2 = DH(a2,alpha2,d2,theta2) 

        #T_0_1 = A1
        T_0_2 = A1@A2
        #x1,y1 = T_0_1[0:2,3]
        x2,y2 = T_0_2[0:2,3]
        
        plt.plot([x1, x2], [y1, y2], 'r-', linewidth=2)  # Linha vermelha para o caminho
        plt.plot(x1, y1, 'ro', markersize=5)  # Ponto atual destacado                
        plt.pause(0.5)
    
    theta1, theta2 = path_angle[-1]
    A1 = DH(a1,alpha1,d1,theta1)
    A2 = DH(a2,alpha2,d2,theta2) 

    #T_0_1 = A1
    T_0_2 = A1@A2
    #x1,y1 = T_0_1[0:2,3]
    xg,yg = T_0_2[0:2,3]


    plt.plot(xg, yg, 'bo', markersize=12,label="Goal")  # Último ponto
    plt.legend()
    
    plt.show()




    while True:
        print(np.rad2deg(path_angle))
        user_input = input("Enter 'start' to send the code to the Arduino: ")
        if user_input.lower() == "start":  # .lower() makes it case-insensitive
            break

    ser_number_input = input("Which COM port is your Arduino connected to?")

    ser_input = "COM" + ser_number_input

    ser = serial.Serial(ser_input, 9600)  # ou '/dev/ttyACM0'
    time.sleep(2)  # Espera o Arduino iniciar


    ser.write(f"{len(path_angle)}\n".encode())
    time.sleep(0.05)
    for theta1, theta2 in path_angle:
        ser.write(f"{int(np.rad2deg(theta1))},{int(np.rad2deg(theta2+np.pi/2))}\n".encode())
        time.sleep(0.05)

    ser.write("END\n".encode())  # Informa fim do caminho
    ser.close()
    print("Path successfully sent to Arduino.")



   

    #plt.figure()
    #plt.plot([0, x1, x2], [0,y1, y2], 'o-')  # 'o-' plota pontos conectados por linha
    #plt.title("Ligação entre junta 1 e junta 2")
    #plt.xlabel("X")
    #plt.ylabel("Y")
    #plt.grid(True)
    #plt.xlim(-elo1-elo2-3,+elo1+elo2+3)
    #plt.ylim(-elo1-elo2-3,+elo1+elo2+3)
    #plt.show()

if __name__ == "__main__":
    main()
