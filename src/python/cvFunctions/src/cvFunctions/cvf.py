import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

def read_config(config_path):
    with open(config_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

class Camera:
    def __init__(self, config_path: str):
        # Загрузка конфигурации
        self.config = read_config(config_path)
        self.camera_matrix = np.array(self.config["camera_matrix"], dtype=np.float64)
        self.dist_coefs = np.array(self.config["dist_coeff"], dtype=np.float64)
        self.newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coefs, (1600, 896), 0.5, (1600, 896))
        self.robot_id = self.config["robot_id"]  # ID маркера нашего робота (в диапазоне 1–10)

        # Настройка детектора ArUco
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        # Матрицы перехода для боковых маркеров нашего робота
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

        # Координаты маркеров поля
        self.field_markers = {
            20: np.array([-0.9, 0.4, 0.0]),
            21: np.array([0.9, 0.4, 0.0]),
            22: np.array([-0.9, -0.4, 0.0]),
            23: np.array([0.9, -0.4, 0.0])
        }

    def prepare_image(self, img):
        """Подготовка изображения для детекции маркеров."""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 1)
        return gray

    def detect_markers(self, img):
        """Детекция маркеров на изображении."""
        img_prepared = self.prepare_image(img)
        corners, ids, _ = self.detector.detectMarkers(img_prepared)
        if ids is not None:
            ids = list(map(lambda x: x[0], ids))
        return ids, corners

    def t_matrix_building(self, ids, corners):
        """Определение системы координат поля с помощью маркеров 20–23."""
        if not ids or not set([20, 21, 22, 23]).issubset(ids):
            return None, None

        object_points = []
        image_points = []
        marker_length = 0.1  # Размер маркеров поля
        for marker_id in [20, 21, 22, 23]:
            idx = ids.index(marker_id)
            obj_points = np.array([
                [-marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, -marker_length / 2, 0],
                [-marker_length / 2, -marker_length / 2, 0]
            ]) + self.field_markers[marker_id]
            object_points.extend(obj_points)
            image_points.extend(corners[idx][0])

        object_points = np.array(object_points)
        image_points = np.array(image_points)

        success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coefs, flags=cv2.SOLVEPNP_ITERATIVE)
        if success:
            tmatrix = cv2.Rodrigues(rvec)[0]  # Матрица вращения камеры относительно поля
            center = tvec.flatten()  # Позиция камеры относительно поля
            return tmatrix, center
        return None, None

    def estimate_robot_pose(self, ids, corners, tmatrix, center, is_our_robot=True):
        """Оценка позы робота (нашего или врага) с вычислением ковариации."""
        if not ids or tmatrix is None or center is None:
            return None, None, None

        object_points = []
        image_points = []
        for i, marker_id in enumerate(ids):
            # Определяем, какие маркеры учитывать
            if is_our_robot:
                if marker_id != self.robot_id and marker_id not in self.RotSideDict:
                    continue
            else:
                if not (1 <= marker_id <= 10 and marker_id != self.robot_id):
                    continue

            # Определение размера маркера
            marker_length = 0.07 if 1 <= marker_id <= 10 else 0.05  # 0.07 для 1–10, 0.05 для RotSideDict

            # Базовые 3D точки маркера
            obj_points = np.array([
                [-marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, -marker_length / 2, 0],
                [-marker_length / 2, -marker_length / 2, 0]
            ])

            # Для боковых маркеров нашего робота применяем преобразование
            if marker_id in self.RotSideDict:
                obj_points = (self.RotSideDict[marker_id] @ obj_points.T).T + self.TvecSideDict[marker_id]

            object_points.extend(obj_points)
            image_points.extend(corners[i][0])

        if not object_points:
            return None, None, None

        object_points = np.array(object_points)
        image_points = np.array(image_points)

        success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coefs, flags=cv2.SOLVEPNP_ITERATIVE)
        if success:
            # Преобразование в систему координат поля
            robot_tvec = np.dot(tmatrix, tvec.flatten()) + center  # Позиция робота
            robot_rot_matrix = tmatrix @ cv2.Rodrigues(rvec)[0]  # Ориентация робота
            quat = R.from_matrix(robot_rot_matrix).as_quat()  # Кватернион ориентации

            # Вычисление ковариации
            _, jac = cv2.projectPoints(object_points, rvec, tvec, self.camera_matrix, self.dist_coefs)
            J = jac[:, :6]  # Якобиан по 6 параметрам (rvec, tvec)
            sigma2 = 1.0  # Предполагаемая дисперсия шума углов в пикселях (1 пиксель²)
            cov = np.linalg.pinv(J.T @ J) * sigma2  # Ковариация позы
            return robot_tvec, quat, cov
        return None, None, None

    def robots_tracking(self, img):
        """Отслеживание нашего робота и врага без фильтра Калмана."""
        ids, corners = self.detect_markers(img)
        tmatrix, center = self.t_matrix_building(ids, corners)

        # Оценка позы нашего робота
        our_tvec, our_quat, our_cov = self.estimate_robot_pose(ids, corners, tmatrix, center, is_our_robot=True)
        robotCoordAver = our_tvec
        quaternion = our_quat

        # Оценка позы врага (только один враг)
        enemy_tvec, enemy_quat, enemy_cov = self.estimate_robot_pose(ids, corners, tmatrix, center, is_our_robot=False)
        enemyCoord = enemy_tvec
        enemyQuat = enemy_quat

        return robotCoordAver, quaternion, enemyCoord, enemyQuat  # Ковариации можно использовать отдельно