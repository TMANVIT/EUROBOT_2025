import cv_functions as cvf
import numpy as np

cvf.camera_initialisation(cameraID = 2, imageHight= 2160, imageWidth= 3840)


def imgProdussing():
    cvf.video_capture()

    ids, transMatrixDict, tvecDict = cvf.markers_detection()

    cvf.imgDrawing()

    return ids, transMatrixDict, tvecDict

def angleBetweenVectors(vector1, vector2):
    cos = np.dot(vector1, vector2)/(np.linalg.norm(vector1) * np.linalg.norm(vector2))
    sin = np.linalg.norm(np.cross(vector1, vector2))/(np.linalg.norm(vector1) * np.linalg.norm(vector2))

    angle = np.angle(cos + sin * 1.j, deg=True)

    return(angle)


def tMatrixBuilding(ids, tvecDict, transMatrixDict):
    corners = []
    tMratrexes = []
    tmatrix = None
    center = None
    for i in ids:
        if i in range(11,51) and i!= 47:
            corners.append(tvecDict[i])
            tMratrexes.append(transMatrixDict[i])

    if(len(corners) == 4):
        center = sum(np.array(corners))/4
    	
        corners = sorted(list(map(lambda x: x.tolist(), corners)))
        corners[:2] = [x for _,x in sorted(zip(list(map(lambda x: x[1], corners[:2])),corners[:2]))]
        corners[2:4] = [x for _,x in sorted(zip(list(map(lambda x: x[1], corners[2:4])),corners[2:4]))]

        for i in range(4):
            corners[i] = np.array(corners[i])

        xvec = (corners[2] + corners[3] - corners[1] - corners[0])/2
        yvec = (corners[1] + corners[3] - corners[2] - corners[0])/2
        zvec = np.cross(xvec, yvec)

        tmatrix = np.array(list(map(lambda x: x/np.linalg.norm(x), [xvec, yvec, zvec])))
        
        # tmatrix = sum(np.array(tMratrexes))/4

    return tmatrix, center


def robotsTracking(ids, transMatrixDict, tvecDict, tmatrix, center):
    robotCoords = []
    angles = []

    for i in ids:
        if i in range(1,10):
            robotCoords.append(np.dot(np.linalg.inv(tmatrix),np.array(tvecDict[i] - center)))
            angles.append(angleBetweenVectors(transMatrixDict[i][0], tmatrix[0]))

    return robotCoords, angles



while  1:
    ids, transMatrixDict, tvecDict = imgProdussing()

    if ids is not None:
        tmatrix, center = tMatrixBuilding(ids, tvecDict, transMatrixDict)

        if tmatrix is not None:
            robotCoords, angles = robotsTracking(ids, transMatrixDict, tvecDict, tmatrix, center)
            print(robotCoords, angles)