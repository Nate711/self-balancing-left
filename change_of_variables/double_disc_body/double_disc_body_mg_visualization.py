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
    cols = "t sec, xD m, yD m, zD m, xE m, yE m, zE m, xHcm m, yHcm m, zHcm m, xI m, yI m, zI m, qH deg, qL deg, qS deg, qE deg, qHH deg, qHL deg, qHS deg, qI deg, ME J"

    df = pd.read_csv('MGLeftRigidDouble_MG.1',
                     delim_whitespace=True, header=None, skiprows=4)
    cols = [c.strip() for c in cols.split(",")]
    df.columns = cols

    # CREATE NEW VISUALIZER
    vis = meshcat.Visualizer()

    # DECLARE CONSTANTS
    WHEEL_THICKNESS = 0.01
    R = 0.1
    FRAME_RATE = 50
    body_L = 0.76
    body_W = 0.14
    body_H = 0.61

    LI = 1
    RI = 0.01

    L_HANDLE = 0.4
    R_HANDLE = 0.02

    # CREATE OBJECTS IN ENVIRONMENT
    # Back wheel
    vis["D"].set_object(g.Cylinder(WHEEL_THICKNESS, R))
    vis["D_tick"].set_object(g.Box([R*1.7, R/2, R/4]),
                             g.MeshBasicMaterial(color=0xff0000))

    # Chassis
    vis["E"].set_object(g.Box([body_L, body_W, body_H]))

    # Front wheel
    vis["H"].set_object(g.Cylinder(WHEEL_THICKNESS, R))
    vis["H_tick"].set_object(g.Box([R*1.7, R/2, R/4]),
                             g.MeshBasicMaterial(color=0xff0000))

    # Steering column wheel
    vis["I"].set_object(g.Cylinder(LI, RI))
    vis["I_handle"].set_object(g.Cylinder(L_HANDLE, R_HANDLE))

    # ANIMATE SIMULATION
    anim = animation.Animation(clips=None, default_framerate=FRAME_RATE)

    for i in tqdm(range(0, df.shape[0])):
        # pdb.set_trace()
        xD = df.iloc[i]["xD m"]
        yD = df.iloc[i]["yD m"]
        zD = df.iloc[i]["zD m"]

        xE = df.iloc[i]["xE m"]
        yE = df.iloc[i]["yE m"]
        zE = df.iloc[i]["zE m"]

        xH = df.iloc[i]["xHcm m"]
        yH = df.iloc[i]["yHcm m"]
        zH = df.iloc[i]["zHcm m"]

        xI = df.iloc[i]["xI m"]
        yI = df.iloc[i]["yI m"]
        zI = df.iloc[i]["zI m"]

        # pdb.set_trace()
        qH = - df.iloc[i]["qH deg"] * np.pi/180
        qL = df.iloc[i]["qL deg"] * np.pi/180
        qS = df.iloc[i]["qS deg"] * np.pi/180
        qE = df.iloc[i]["qE deg"] * np.pi/180

        qHH = - df.iloc[i]["qHH deg"] * np.pi/180
        qHL = df.iloc[i]["qHL deg"] * np.pi/180
        qHS = df.iloc[i]["qHS deg"] * np.pi/180
        qI = df.iloc[i]["qI deg"] * np.pi/180

        with anim.at_frame(vis, i) as frame:
            A_R_B = np.array([[np.cos(qH), np.sin(qH), 0],
                              [-np.sin(qH), np.cos(qH), 0],
                              [0, 0, 1.0]])

            B_R_C = np.array([[1.0, 0, 0],
                              [0, np.cos(qL), np.sin(qL)],
                              [0, -np.sin(qL), np.cos(qL)]])

            C_R_D = np.array([[np.cos(qS), 0, np.sin(qS)],
                              [0, 1.0, 0],
                              [-np.sin(qS), 0, np.cos(qS)]])

            C_R_E = np.array([[np.cos(qE), 0, np.sin(qE)],
                              [0, 1.0, 0],
                              [-np.sin(qE), 0, np.cos(qE)]])

            # Front system
            A_R_F = np.array([[np.cos(qHH), np.sin(qHH), 0],
                              [-np.sin(qHH), np.cos(qHH), 0],
                              [0, 0, 1.0]])

            F_R_G = np.array([[1.0, 0, 0],
                              [0, np.cos(qHL), np.sin(qHL)],
                              [0, -np.sin(qHL), np.cos(qHL)]])

            G_R_H = np.array([[np.cos(qHS), 0, np.sin(qHS)],
                              [0, 1.0, 0],
                              [-np.sin(qHS), 0, np.cos(qHS)]])

            G_R_I = np.array([[np.cos(qI), 0, np.sin(qI)],
                              [0, 1.0, 0],
                              [-np.sin(qI), 0, np.cos(qI)]])

            # Correct for cylinder orientation
            steering_column_corrector = np.array([[1.0, 0, 0.0],
                                                  [0, 0, 1],
                                                  [0, -1, 0]])

            A_R_D = A_R_B @ B_R_C @ C_R_D
            A_R_E = A_R_B @ B_R_C @ C_R_E

            A_R_H = A_R_F @ F_R_G @ G_R_H
            A_R_I = A_R_F @ F_R_G @ G_R_I

            D_transform = np.eye(4)
            D_transform[0:3, 0:3] = A_R_D
            D_transform[0:3, 3] = np.array([xD, yD, zD])
            frame["D"].set_transform(D_transform)
            frame["D_tick"].set_transform(D_transform)

            E_transform = np.eye(4)
            E_transform[0:3, 0:3] = A_R_E
            E_transform[0:3, 3] = np.array([xE, yE, zE])
            frame["E"].set_transform(E_transform)

            H_transform = np.eye(4)
            H_transform[0:3, 0:3] = A_R_H
            H_transform[0:3, 3] = np.array([xH, yH, zH])
            frame["H"].set_transform(H_transform)
            frame["H_tick"].set_transform(H_transform)

            I_transform = np.eye(4)
            I_transform[0:3, 0:3] = A_R_I @ steering_column_corrector
            I_transform[0:3, 3] = np.array([xI, yI, zI])
            frame["I"].set_transform(I_transform)

            I_handle_transform = np.eye(4)
            I_handle_transform[0:3, 0:3] = A_R_I
            I_handle_transform[0:3, 3] = np.array([xI, yI, zI])
            frame["I_handle"].set_transform(I_handle_transform)

        vis.set_animation(anim)
    print("")
    print("Click 'Open Controls' in the top-right of the window to view the animation controls")
    input("Press enter to kill the simulation")
