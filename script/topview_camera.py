import cv_functions as cvf
import numpy as np

cvf.camera_initialisation(cameraID = 3, imageHight= 480, imageWidth= 640)


def imgProdussing():
    cvf.video_capture()

    ids, arucoIdDict, transMatrixDict, tvecDict = cvf.markers_detection()

    cvf.imgDrawing()

    return ids, arucoIdDict, transMatrixDict, tvecDict

def angleBetweenMatrexes(matx1, matx2):
    cos = np.dot(matx1[0], matx2[0])/(np.linalg.norm(matx1[0]) * np.linalg.norm(matx2[0]))
    sin = np.linalg.norm(np.cross(matx1[0], matx2[0]))/(np.linalg.norm(matx1[0]) * np.linalg.norm(matx2[0]))

    angle = np.angle(cos + sin * 1.j, deg=True)

    return(angle)


def tMatrixBuilding(ids, arucoIdDict, tvecDict, transMatrixDict):
    corners = []
    tMratrexes = []
    tmatrix = None
    center = None
    for i in ids:
        if i in range(11,51) and i!= 47:
            corners.append(np.array(tvecDict[cvf.fromArucIDtoIndex(i, arucoIdDict)]))
            tMratrexes.append(transMatrixDict[cvf.fromArucIDtoIndex(i, arucoIdDict)])

    if(len(corners) == 4):
    
        center = sum(np.array(corners))/4
    	
        corners = sorted(corners)
        corners[:2] = [l[::-1] for l in sorted([l[::-1] for l in corners[:2]])]
        corners[2:4] = [l[::-1] for l in sorted([l[::-1] for l in corners[2:4]])]

        for i in range(4):
            corners[i] = np.array(corners[i])

        xvec = (corners[2] + corners[3] - corners[1] - corners[0])/2
        yvec = (corners[1] + corners[3] - corners[2] - corners[0])/2
        zvec = np.cross(xvec, yvec)

        tmatrix = np.array(list(map(lambda x: x/np.linalg.norm(x), [xvec, yvec, zvec])))
        
        tmatrix = sum(np.array(tMratrexes))/4

    return tmatrix, center


def robotsTracking(ids, arucoIdDict, transMatrixDict, tvecDict, tmatrix, center):
    robotCoords = []
    angles = []

    for i in ids:
        if i in range(1,10):
            robotCoords.append(np.linalg.inv(tmatrix).dot(np.array(tvecDict[cvf.fromArucIDtoIndex(i, arucoIdDict)]) - center))
            angles.append(angleBetweenMatrexes(transMatrixDict[cvf.fromArucIDtoIndex(i, arucoIdDict)][0], tmatrix[0]))

    return robotCoords, angles



while  1:
    ids, arucoIdDict, transMatrixDict, tvecDict = imgProdussing()

    if ids is not None:
        tmatrix, center = tMatrixBuilding(ids, arucoIdDict, tvecDict, transMatrixDict)
        
        robotCoords, angles = robotsTracking(ids, arucoIdDict, transMatrixDict, tvecDict, tmatrix, center)

