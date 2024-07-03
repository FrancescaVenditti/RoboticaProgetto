import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.serialization import serialize_message
from geometry_msgs.msg import Pose, PoseArray
import rcl_interfaces.msg
import rosbag2_py
import numpy as np
import os
import shutil
import time

class TrajectoryNode(Node):
    def __init__(self, min_length, offset, gamma):
        super().__init__('trajectory_node')
        self.declare_parameter('min_length', min_length)
        self.declare_parameter('num_points', 10)
        self.declare_parameter('offset', offset)
        self.declare_parameter('gamma', gamma)
        
        self.generate_and_write_trajectory()

    def get_rotation_matrix(self, gamma):
        rotation_matrix_x = np.array([
            [1, 0, 0],
            [0, np.cos(gamma[0]), -np.sin(gamma[0])],
            [0, np.sin(gamma[0]), np.cos(gamma[0])]
        ])

        rotation_matrix_y = np.array([
            [np.cos(gamma[1]), 0, np.sin(gamma[1])],
            [0, 1, 0],
            [-np.sin(gamma[1]), 0, np.cos(gamma[1])]
        ])

        rotation_matrix_z = np.array([
            [np.cos(gamma[2]), -np.sin(gamma[2]), 0],
            [np.sin(gamma[2]), np.cos(gamma[2]), 0],
            [0, 0, 1]
        ])

        return np.dot(rotation_matrix_x, np.dot(rotation_matrix_y, rotation_matrix_z))

    @staticmethod
    def generate_straight_trajectory(min_length, num_points, offset, rotation_matrix):
        # Generare punti linearmente interpolati lungo una retta
        points = np.zeros((num_points, 3))  # num_points punti, 3 coordinate (x, y, z)
        start_point = np.array([0.0, 0.0, 0.0])
        end_point = np.array([min_length, 0.0, 0.0])

        for i in range(num_points):
            t = i / float(num_points - 1)
            point = (1 - t) * start_point + t * end_point
            points[i] = point
        
        # Applicare la rotazione e l'offset a tutti i punti
        rotated_points = np.dot(rotation_matrix, points.T).T + offset
        
        # Estrarre le coordinate x, y e z
        x = rotated_points[:, 0]
        y = rotated_points[:, 1]
        z = rotated_points[:, 2]

        return x, y, z

    def generate_and_write_trajectory(self):
        # Get parameters
        min_length = self.get_parameter('min_length').value
        num_points = self.get_parameter('num_points').value
        offset = self.get_parameter('offset').value
        gamma = self.get_parameter('gamma').value

        # Calculate rotation matrix from gamma
        rotation_matrix = self.get_rotation_matrix(gamma)
        
        # Generate straight trajectory
        x, y, z = self.generate_straight_trajectory(min_length, num_points, np.array(offset), rotation_matrix)

        # Generate a unique filename based on the current time
        time_stamp = Clock().now()
        filename = f'straight_trajectory_{min_length}.bag'
        
        # Verifica se il file bag esiste e cancellalo se presente
        if os.path.isdir(filename):
            shutil.rmtree(filename)
            
        # Create PoseArray
        poses = PoseArray()
        for i in range(len(x)):
            pose = Pose()
            pose.position.x = x[i]
            pose.position.y = y[i]
            pose.position.z = z[i]
            # Assuming a fixed orientation (can be parameterized as well)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            poses.poses.append(pose)

        # Setup ROS Bag
        writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py.StorageOptions(uri=filename, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        writer.open(storage_options, converter_options)
        
        # Create a topic for PoseArray data
        topic_info = rosbag2_py.TopicMetadata(name='straight_trajectory', type='geometry_msgs/msg/PoseArray', serialization_format='cdr')
        writer.create_topic(topic_info)

        # Write PoseArray data to the Bag file
        writer.write('straight_trajectory', serialize_message(poses), time_stamp.nanoseconds)

def main(args=None):
    rclpy.init(args=args)

    try:
        min_length = float(input("Enter the minimum length of the straight trajectory: "))
        if min_length < 0.6:
            raise ValueError("Minimum length must be at least 0.6 meters.")
    except ValueError as e:
        print(f"Invalid input: {e}")
        rclpy.shutdown()
        return

    offset_input = input("Enter the offset as x,y,z: ").split(',')
    try:
        offset = [float(val) for val in offset_input]
        if len(offset) != 3:
            raise ValueError("Invalid input. Please enter exactly three numerical values separated by commas.")
    except (ValueError, IndexError):
        print("Invalid input. Please enter exactly three numerical values separated by commas.")
        rclpy.shutdown()
        return

    gamma_input = input("Enter the angle (in radians) as x,y,z: ").split(',')
    try:
        gamma = [float(val) for val in gamma_input]
        if len(gamma) != 3:
            raise ValueError("Invalid input. Please enter exactly three numerical values separated by commas.")
    except (ValueError, IndexError):
        print("Invalid input. Please enter exactly three numerical values separated by commas.")
        rclpy.shutdown()
        return

    trajectory_node = TrajectoryNode(min_length, offset, gamma)

    # Genera e scrivi la traiettoria, poi distruggi il nodo e chiudi rclpy
    trajectory_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
