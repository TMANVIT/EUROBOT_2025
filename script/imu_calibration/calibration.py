#!/usr/bin/env python3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosbag2_py._storage import StorageFilter
import numpy as np
from sensor_msgs.msg import Imu

BIAS_VELOCITY = [-0.02058635, -0.00458323, -0.02784844]
# BIAS_VELOCITY = [0.02080604 0.00754197 0.0263029] 


BIAS_ACCEL = [-0.14627597,  0.03286619, 10.41976869]

def extract_imu_data(bag_path, topic_name):
    """
    Extracts IMU data (angular velocity and linear acceleration) from a specified topic in a ROS 2 bag file.

    Args:
        bag_path (str): Path to the ROS 2 bag file.
        topic_name (str): Name of the IMU topic (e.g., '/imu/data').

    Returns:
        tuple: Two numpy arrays:
            - angular_velocities: Array of angular velocity measurements (shape: N x 3).
            - linear_accelerations: Array of linear acceleration measurements (shape: N x 3).
    """
    # Configure storage and converter options for reading the bag
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    # Initialize the bag reader
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Set a filter to read only the specified topic
    storage_filter = StorageFilter(topics=[topic_name])
    reader.set_filter(storage_filter)

    # Lists to store IMU data
    angular_velocities = []
    linear_accelerations = []
    orientations = []

    # Read messages from the bag
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == topic_name:
            # Deserialize the IMU message
            msg_type = get_message('sensor_msgs/msg/Imu')
            msg = deserialize_message(data, msg_type)

            # Extract angular velocity (gyroscope data)
            angular_velocity = [
                msg.angular_velocity.x - BIAS_VELOCITY[0],
                msg.angular_velocity.y - BIAS_VELOCITY[1],
                msg.angular_velocity.z - BIAS_VELOCITY[2]
            ]
            angular_velocities.append(angular_velocity)

            # Extract linear acceleration (accelerometer data)
            linear_acceleration = [
                msg.linear_acceleration.x - BIAS_ACCEL[0],
                msg.linear_acceleration.y - BIAS_ACCEL[1],
                msg.linear_acceleration.z - BIAS_ACCEL[2]
            ]
            linear_accelerations.append(linear_acceleration)
            
            orientation = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]
            orientations.append(orientation)

    # Convert lists to numpy arrays for easier computation
    return np.array(angular_velocities), np.array(linear_accelerations), np.array(orientations)

def compute_covariance(data):
    """
    Computes the covariance matrix for the given data.

    Args:
        data (numpy.ndarray): Input data (shape: N x M, where N is the number of samples and M is the number of features).

    Returns:
        numpy.ndarray: Covariance matrix (shape: M x M).
    """
    return np.cov(data, rowvar=False)

def main():
    """
    Main function to extract IMU data from a ROS 2 bag file and compute its covariance matrices.
    """
    # Path to the ROS 2 bag file and the IMU topic name
    bag_path = '/home/timur/EUROBOT_2025/data/rosbag2_2025_04_13-15_32_08'
    topic_name = '/imu/data'

    # Extract IMU data from the bag
    angular_velocities, linear_accelerations, orientations = extract_imu_data(bag_path, topic_name)

    # Compute covariance matrices
    angular_velocity_cov = compute_covariance(angular_velocities)
    linear_acceleration_cov = compute_covariance(linear_accelerations)
    orientation_cov = compute_covariance(orientations) 
    
    bias_velocity = np.mean(angular_velocities, axis=0)
    bias_accel = np.mean(linear_accelerations, axis=0)
    bias_orient = np.mean(orientations, axis=0)

    # Print the results
    print("Covariance matrix for angular velocity (gyroscope):")
    print(angular_velocity_cov)

    print("\nCovariance matrix for linear acceleration (accelerometer):")
    print(linear_acceleration_cov)
    
    print("\nCovariance matrix for orientation:")
    print(orientation_cov)
    
    print("\nbias_velocity:")
    print(bias_velocity)
    print("\nbias_accel:")
    print(bias_accel)
    print("\nbias_orient:")
    print(bias_orient)

if __name__ == '__main__':
    main()