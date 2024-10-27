import cv2
# import traj_planning as tk
import keyboard
import cv_functions as cvf
import numpy as np

flag = 1
fPressed = 0

cvf.camera_initialisation()


def imgProdussing():
    cvf.video_capture()

    ids, arucoIdDict, transMatrixDict, tvecDict = cvf.markers_detection()


    if ids is not None:
        for j in range(len(ids)):
            index = arucoIdDict[ids[j][0]]

            tvecDict[index][0][0][0] -= 0.005
            tvecDict[index][0][0][1] -= 0.03
            tvecDict[index][0][0][2] -= (0.0562 + 0.034)

    cvf.imgDrawing()

    return ids, arucoIdDict, transMatrixDict, tvecDict


while(1):
    ids, arucoIdDict, transMatrixDict, tvecDict = imgProdussing()

        