import cv2
import numpy as np
import json

marker_length = 0.10

def camera_initialisation(cameraID = 0, imageWidth = 1920, imageHight = 1080):
    global camera, newcameramtx, camera_matrix, dist_coefs, detector, arucoDict, arucoParams

    f = open("C:/coding/git/EUROBOT_2025/script/fisheye_calibration_data.json")
    data = json.load(f)

    camera_matrix = np.array(data["camera_matrix"])
    dist_coefs = np.array(data["dist_coeff"])
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (imageWidth, imageHight), 1, (imageWidth, imageHight))

    camera = cv2.VideoCapture(cameraID, cv2.CAP_DSHOW)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, imageWidth+10)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, imageHight+10)

    arucoParams = cv2.aruco.DetectorParameters()
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

def video_capture():
    global img, imgToProduse, gray

    good, img = camera.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgToProduse = gray

def markers_detection():
    tvecDictionary = {}
    transMatrixDictionary = {}
    corners, ids, rejected = cv2.aruco.detectMarkers(imgToProduse, arucoDict, parameters=arucoParams)
    
    
    if ids is not None:
        ids = list(map(lambda x: x[0], ids))
        for i in range(len(ids)):
            
            retval, rvec, tvec = cv2.fisheye.solvePnP(np.array([
                [-marker_length/2, marker_length/2, 0],
                [marker_length/2, marker_length/2, 0],
                [marker_length/2, -marker_length/2, 0],
                [-marker_length/2, -marker_length/2, 0]]), 
                corners[i], camera_matrix, dist_coefs, flags=cv2.SOLVEPNP_ITERATIVE)
            
            cv2.drawFrameAxes(img, camera_matrix, dist_coefs, rvec, tvec, length=0.1)

            tvecDictionary[ids[i]] = tvec
            transMatrixDictionary[ids[i]] = cv2.Rodrigues(rvec)[0]
            
            cv2.putText(img, str(ids[i]),(int(corners[i][0][0][0]), int(corners[i][0][0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.aruco.drawDetectedMarkers(img, corners)
    
    return ids, transMatrixDictionary, tvecDictionary

def imgDrawing(imageWidth = 720, imageHight = 480):

    cv2.namedWindow("Object_detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Object_detection", width=imageWidth, height=imageHight)
    cv2.imshow("Object_detection", img)
    cv2.waitKey(1)

