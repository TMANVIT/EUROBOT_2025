import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

def read_config(config):
    # Load YAML configuration
    with open(config, 'r') as file:
        data = yaml.safe_load(file)  
    return data  

class Camera():
    def __init__(self, config_path: str):
        self.config = read_config(config_path)
        self.camera_matrix = np.array(self.config["camera_matrix"], dtype=np.float64)
        self.dist_coefs = np.array(self.config["dist_coeff"], dtype=np.float64)
        self.newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coefs, (1600, 896), 0.5, (1600, 896))
        self.robot_id = self.config["robot_id"]

        # Настройка параметров детекции ArUco
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        # Оптимизированные параметры для уменьшения шума и повышения точности
        self.arucoParams.adaptiveThreshWinSizeMin = 3      # Минимальный размер окна адаптивной пороговой обработки
        self.arucoParams.adaptiveThreshWinSizeMax = 23     # Максимальный размер окна адаптивной пороговой обработки
        self.arucoParams.adaptiveThreshWinSizeStep = 10    # Шаг изменения размера окна
        self.arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # Уточнение углов с субпиксельной точностью
        self.arucoParams.cornerRefinementWinSize = 5       # Размер окна для уточнения углов
        self.arucoParams.minMarkerPerimeterRate = 0.1      # Минимальный периметр маркера относительно изображения
        self.arucoParams.polygonalApproxAccuracyRate = 0.05 # Точность аппроксимации контура маркера
        self.arucoParams.markerBorderBits = 1              # Толщина границы маркера (по умолчанию 1)

        self.RotSideDict = {
            55: R.from_euler('xyz', [0.0, -90.0, 0.0], degrees=True).as_matrix(),
            56: R.from_euler('xyz', [-90.0, 0.0, 0.0], degrees=True).as_matrix(),
            57: R.from_euler('xyz', [0.0, 90.0, 0.0], degrees=True).as_matrix(),
            58: R.from_euler('xyz', [90.0, 0.0, 0.0], degrees=True).as_matrix()
        }
        self.TvecSideDict = {
            55: [-0.025, 0.0, 0.025],
            56: [0.0, 0.025, 0.025],
            57: [0.025, 0.0, 0.025],
            58: [0.0, -0.025, 0.025]
        }

    def prepare_image(self, img):
        # Улучшенная подготовка изображения для уменьшения шума
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        # gray = clahe.apply(gray)
        # gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 1)


        
        return gray
    
    def detect_markers(self, img):
        tvecDictionary = {}
        transMatrixDictionary = {}
        weightsDictionary = {}
        imgToProduse = self.prepare_image(img)
        corners, ids, rejected = self.detector.detectMarkers(imgToProduse)  # Используем оптимизированный детектор
    
        if ids is not None:
            ids = list(map(lambda x: x[0], ids))
            for i in range(len(ids)):
                if ids[i] in range(10):
                    marker_length = 0.069
                else:
                    marker_length = 0.1

                rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, 
                                                                            cameraMatrix=self.camera_matrix, 
                                                                            distCoeffs=self.dist_coefs)
                tvecDictionary[ids[i]] = tvec[0][0]
                transMatrixDictionary[ids[i]] = cv2.Rodrigues(rvec)[0]
                weightsDictionary[ids[i]] = abs(transMatrixDictionary[ids[i]][2][2])
        
        return ids, transMatrixDictionary, tvecDictionary, weightsDictionary
    
    def t_matrix_building(self, ids, tvecDict):
        tmatrix = None
        center = None

        if set([20, 21, 22, 23]).issubset(ids):
            center = (tvecDict[22] + tvecDict[20] + tvecDict[23] + tvecDict[21]) / 4
            
            xvec = (tvecDict[22] + tvecDict[20] - tvecDict[23] - tvecDict[21]) * -1
            xvec = xvec / np.linalg.norm(xvec)
            yvec = (tvecDict[22] + tvecDict[23] - tvecDict[20] - tvecDict[21]) * -1
            yvec = yvec / np.linalg.norm(yvec)
            zvec = np.cross(xvec, yvec)
            zvec = zvec / np.linalg.norm(zvec)  # Нормализация zvec для стабильности
            
            tmatrix = np.array([xvec, yvec, zvec])

        return tmatrix, center 
        
    def robots_tracking(self, ids, transMatrixDict, tvecDict, weightsDictionary, tmatrix, center):
        ourRobot = False
        tvecArray = np.array()
        rvecArray = np.array()
        weightsArray = np.array()
        for i in ids:
            if i in list(range(11))+self.RotSideDict.keys():
                rvec = tvecDict[i] - center
                robotCoord = [np.dot(rvec, tmatrix[0]), np.dot(rvec, tmatrix[1]), np.dot(rvec, tmatrix[2])]
                robotTransMatrix = np.linalg.inv(tmatrix) @ transMatrixDict[i]
                if i in self.RotSideDict.keys():
                    robotCoord += self.TvecSideDict[i]
                    robotTransMatrix = robotTransMatrix @ self.RotSideDict[i]
                robotTransMatrix[0][2] = 0.0
                robotTransMatrix[0] = robotTransMatrix[0] / np.linalg.norm(robotTransMatrix[0])
                robotTransMatrix[1][2] = 0.0
                robotTransMatrix[1] = robotTransMatrix[1] / np.linalg.norm(robotTransMatrix[1])
                robotTransMatrix[2] = [0.0, 0.0, 1.0]
                
                tvecArray.append(robotCoord)
                rvecArray.append(robotTransMatrix)
                weightsArray.append(weightsDictionary[i])

                
                if i == self.robot_id or i in self.RotSideDict.keys():
                    ourRobot = True

        if not ourRobot:
            return None, None, None
        
        if len(tvecArray) == 1:
            r = R.from_matrix(rvecArray[0])
            quaternion = r.as_quat()
            robotCoordAver = tvecArray[0]
        else:
            weightsArray = weightsArray/sum(weightsArray)
            robotCoordAver = np.mean(tvecArray)
            AverTrans = np.average(rvecArray, axis=0, weights=weightsArray)
            r = R.from_matrix(AverTrans)
            quaternion = r.as_quat()

        return robotCoordAver, quaternion, ourRobot
            
        