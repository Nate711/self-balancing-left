# IMPORT PACKAGES
import numpy as np
import pandas as pd
import pdb
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat import animation
from numpy import sin, cos
from tqdm import tqdm

if __name__ == "__main__":

    # IMPORT MG OUTPUT
    cols = "t sec, x m, y m, z m, qA deg, qB deg, qC deg, ME J"

    df = pd.read_csv('MGLeftDisk.1',
                     delim_whitespace=True, header=None, skiprows=4)
    cols = [c.strip() for c in cols.split(",")]
    df.columns = cols

    # CREATE NEW VISUALIZER
    vis = meshcat.Visualizer()

    # DECLARE CONSTANTS
    WHEEL_THICKNESS = 0.01
    R = 0.1
    FRAME_RATE = 200

    # CREATE OBJECTS IN ENVIRONMENT
    # Robot Body
    vis["A"].set_object(g.Cylinder(WHEEL_THICKNESS, R))
    vis["B"].set_object(g.Box([R*1.7, R/2, R/4]), g.MeshBasicMaterial(color=0xff0000))

    # ANIMATE SIMULATION
    anim = animation.Animation(clips=None, default_framerate=FRAME_RATE)

    for i in tqdm(range(0, df.shape[0])):
        # pdb.set_trace()
        x = df.iloc[i]["x m"]
        y = df.iloc[i]["y m"]
        z = df.iloc[i]["z m"]

        # pdb.set_trace()
        qA = df.iloc[i]["qA deg"] * np.pi/180
        qB = df.iloc[i]["qB deg"] * np.pi/180
        qC = df.iloc[i]["qC deg"] * np.pi/180

        with anim.at_frame(vis, i) as frame:
            A_transform = np.eye(4)
            qA = -qA
            qB = -qB
            qC = qC

            N_R_A = np.array([[np.cos(qA), np.sin(qA), 0],
                              [-np.sin(qA), np.cos(qA), 0],
                              [0, 0, 1.0]])

            A_R_B = np.array([[1.0, 0, 0],
                              [0, np.cos(qB), np.sin(qB)],
                              [0, -np.sin(qB), np.cos(qB)]])

            B_R_C = np.array([[np.cos(qC), 0, np.sin(qC)],
                              [0, 1.0, 0],
                              [-np.sin(qC), 0, np.cos(qC)]])

            N_R_C = N_R_A @ A_R_B @ B_R_C                

            A_transform[0:3, 0:3] = N_R_C
            A_transform[0:3, 3] = np.array([x, y, z])
            frame["A"].set_transform(A_transform)
            frame["B"].set_transform(A_transform)

        vis.set_animation(anim)
    print("")
    print("Click 'Open Controls' in the top-right of the window to view the animation controls")
    input("Press enter to kill the simulation")
