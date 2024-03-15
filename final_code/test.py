import mujoco
import mujoco_viewer
import numpy as np

#Prompt the user for their choice
print("Which example would you like to try out? Please choose from the following generations: 1, 10, 25, 50, 250, or 500.")
generation_input = input("Your choice: ")

#Define the paths and moves based on the input
if generation_input == "1":
    xml_path = "examples/gen1.xml"
    moves = np.array([
            0.38503006052691247,
            0.6300593125849123,
            0.32751724739631805,
            0.5751899174240933,
            0.5814510467518661,
            0.8526478694474833,
            0.7586516381288169,
            0.5561277559147753,
            0.9641184988642897,
            0.6672796135658887,
            0.6949052412342243,
            0.27629479183237177,
            0.3783524972973028
        ])
elif generation_input == "10":
    xml_path = "examples/gen10.xml"
    moves = np.array([
            0.7577992923090502,
            0.9323950535902464,
            0.547425372499866
        ])
elif generation_input == "25":
    xml_path = "examples/gen25.xml"
    moves = np.array([
            0.8787749564958481,
            0.8992896789627538,
            0.4662183778972575
        ])
elif generation_input == "50":
    xml_path = "examples/gen50.xml"
    moves = np.array([
            0.36357209342290175,
            1.0
        ])
elif generation_input == "250":
    xml_path = "examples/gen250.xml"
    moves = np.array([
            0.29309032767879073,
            1.0
        ])

elif generation_input == "500":
    xml_path = "examples/gen500.xml"
    moves = np.array([
            0.3261989217432363,
            1.0
        ])
else:
    print("Invalid choice. Exiting.")
    exit()

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)
actuators = model.nu

data.ctrl[:actuators] = moves

#Enter simulation loop
for i in range(10000):
    if viewer.is_alive:
        if i % 40 == 0:
            #Switch movement direction back and forth every 40 steps
            moves *= -1
        data.ctrl[:actuators] = moves
        mujoco.mj_step(model, data)
        viewer.render()
    else:
        break

viewer.close()
