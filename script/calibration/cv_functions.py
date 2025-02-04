import cv2
import numpy as np
import json


def camera_initialisation(cameraID = 0, imageWidth = 1920, imageHight = 1080):
    global camera, newcameramtx, camera_matrix, dist_coefs, detector, arucoDict, arucoParams
 
    f = open("C:/coding/git/EUROBOT_2025/script/data_wa.json")
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
    gray = cv2.normalize(gray, None, 1.0, 255, cv2.NORM_MINMAX, dtype = cv2.CV_8U)
    gray = cv2.medianBlur(gray, 3)
    ret, gray = cv2.threshold(gray, 200,220,cv2.THRESH_BINARY)
    imgToProduse = gray

def markers_detection():
    tvecDictionary = {}
    transMatrixDictionary = {}
    corners, ids, rejected = cv2.aruco.detectMarkers(imgToProduse, arucoDict, parameters=arucoParams)
    
    
    if ids is not None:
        ids = list(map(lambda x: x[0], ids))
        for i in range(len(ids)):
            if ids[i] in range(10):
                marker_length = 0.069
            else:
                marker_length = 0.1

            rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, cameraMatrix=camera_matrix, distCoeffs= None)
            tvecDictionary[ids[i]] = tvec[0][0]
            
            transMatrixDictionary[ids[i]] = cv2.Rodrigues(rvec)[0]
            
            cv2.drawFrameAxes(img, camera_matrix, dist_coefs, rvec, tvec, length=0.1)
            
            cv2.putText(img, str(ids[i]),(int(corners[i][0][0][0]), int(corners[i][0][0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.aruco.drawDetectedMarkers(img, corners)
    
    return ids, transMatrixDictionary, tvecDictionary

def imgDrawing(imageWidth = 1920, imageHight = 1080):

    cv2.namedWindow("Object_detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Object_detection", width=imageWidth, height=imageHight)
    cv2.imshow("Object_detection", img)
    cv2.waitKey(1)

