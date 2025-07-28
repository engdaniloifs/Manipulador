import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button

import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button

def get_obstacles_from_user():
    # Store result here
    result = []

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
    textbox_n = TextBox(ax_n, "Enter number of obstacles (0–5)")
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
        nonlocal result
        result = []
        try:
            for boxes in textboxes:
                xc = float(boxes[0].text)
                yc = float(boxes[1].text)
                w = float(boxes[2].text)
                h = float(boxes[3].text)
                result.append(((xc, yc), (w, h)))
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

    return result


OBSTACLES = get_obstacles_from_user()

print(OBSTACLES)