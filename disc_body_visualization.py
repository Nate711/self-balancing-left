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
    cols = "t sec, xC m, yC m, zC m, xD m, yD m, zD m, qA deg, qB deg, qC deg, qD deg, ME J"

    df = pd.read_csv('MGLeftRigid.1',
                     delim_whitespace=True, header=None, skiprows=4)
    cols = [c.strip() for c in cols.split(",")]
    df.columns = cols

    # CREATE NEW VISUALIZER
    vis = meshcat.Visualizer()

    # DECLARE CONSTANTS
    WHEEL_THICKNESS = 0.01
    R = 0.1
    FRAME_RATE = 200
    body_L = 0.76
    body_W = 0.14
    body_H = 0.61

    # CREATE OBJECTS IN ENVIRONMENT
    # Robot Body
    vis["C"].set_object(g.Cylinder(WHEEL_THICKNESS, R))
    vis["C_tick"].set_object(g.Box([R*1.7, R/2, R/4]),
                        g.MeshBasicMaterial(color=0xff0000))

    # , g.MeshBasicMaterial(color=0x005500))
    vis["D"].set_object(g.Box([body_L, body_W, body_H]))

    # ANIMATE SIMULATION
    anim = animation.Animation(clips=None, default_framerate=FRAME_RATE)

    for i in tqdm(range(0, df.shape[0])):
        # pdb.set_trace()
        xC = df.iloc[i]["xC m"]
        yC = df.iloc[i]["yC m"]
        zC = df.iloc[i]["zC m"]

        xD = df.iloc[i]["xD m"]
        yD = df.iloc[i]["yD m"]
        zD = df.iloc[i]["zD m"]

        # pdb.set_trace()
        qA = df.iloc[i]["qA deg"] * np.pi/180
        qB = df.iloc[i]["qB deg"] * np.pi/180
        qC = df.iloc[i]["qC deg"] * np.pi/180
        qD = df.iloc[i]["qD deg"] * np.pi/180

        with anim.at_frame(vis, i) as frame:
            C_transform = np.eye(4)
            qA = -qA
            qB = -qB
            qD = -qD
            qC = qC

            N_R_A = np.array([[np.cos(qA), np.sin(qA), 0],
                              [-np.sin(qA), np.cos(qA), 0],
                              [0, 0, 1.0]])

            A_R_B = np.array([[1.0, 0, 0],
                              [0, np.cos(qB), np.sin(qB)],
                              [0, -np.sin(qB), np.cos(qB)]])

            B_R_D = np.array([[np.cos(qD), 0, np.sin(qD)],
                              [0, 1.0, 0],
                              [-np.sin(qD), 0, np.cos(qD)]])

            B_R_C = np.array([[np.cos(qC), 0, np.sin(qC)],
                              [0, 1.0, 0],
                              [-np.sin(qC), 0, np.cos(qC)]])

            N_R_C = N_R_A @ A_R_B @ B_R_C
            N_R_D = N_R_A @ A_R_B @ B_R_D

            C_transform[0:3, 0:3] = N_R_D
            C_transform[0:3, 3] = np.array([xC, yC, zC])
            frame["C"].set_transform(C_transform)
            frame["C_tick"].set_transform(C_transform)

            D_transform = np.eye(4)
            D_transform[0:3, 0:3] = N_R_D
            D_transform[0:3, 3] = np.array([xD, yD, zD])
            frame["D"].set_transform(D_transform)

        vis.set_animation(anim)
    print("")
    print("Click 'Open Controls' in the top-right of the window to view the animation controls")
    input("Press enter to kill the simulation")
