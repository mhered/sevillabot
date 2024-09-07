import numpy as np
import os 

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from utils_inertia import scale_mesh_data, print_inertial


def extract_voxel_data(file_path):
    """
    Calculate the inertial properties of a voxel model from a file.

    Args:
        file_path (str): The path to the file containing the voxel model data.
    Returns:
        dict: A dictionary containing the following inertial properties:
            - max_dimensions (list): The maximum dimensions of the voxel model in each axis.
            - volume (int): The total number of voxels in the model.
            - center_of_mass (tuple): The coordinates of the center of mass of the voxel model.
            - inertia_tensor (dict): The components of the inertia tensor of the voxel model, including:
                - i_xx (float): The moment of inertia around the x-axis.
                - i_yy (float): The moment of inertia around the y-axis.
                - i_zz (float): The moment of inertia around the z-axis.
                - i_xy (float): The product of inertia between the x and y axes.
                - i_xz (float): The product of inertia between the x and z axes.
                - i_yz (float): The product of inertia between the y and z axes.
    """


    try:
        # Attempt to load the data from the file
        data = np.loadtxt(file_path, unpack=True, delimiter=",", dtype=int)
        
        # Check if the data has the expected number of columns
        if data.shape[0] < 3:
            raise ValueError("Data file does not contain enough columns for x, y, z coordinates.")
        
        # Attempt to unpack the data into x, y, z
        try:
            x, y, z = data[[0, 2, 1]]
        except IndexError as e:
            raise IndexError("Error unpacking the data into x, y, z coordinates.") from e

    except IOError as e:
        raise IOError(f"Failed to load the file at {file_path}. Please check if the file exists and is accessible.") from e
    except ValueError as e:
        raise ValueError("Data could not be loaded or parsed. Please check the file format.") from e
    except Exception as e:
        raise RuntimeError("An unexpected error occurred while loading the voxel data.") from e

    # Calculate Center of Mass 
    x = x - np.mean(x)
    y = y - np.mean(y)
    z = z - np.mean(z)
    Center_of_Mass = np.mean(x), np.mean(y), np.mean(z)

    # Quick 3D Plot of all voxels
    # fig = plt.figure()
    # ax = fig.add_subplot(projection="3d")
    # ax.scatter(x, y, z)
    # ax.view_init(elev=20)
    # plt.show()

    N = data.shape[1]
    i_xx = sum(y**2 + z**2)
    i_yy = sum(x**2 + z**2) 
    i_zz = sum(x**2 + y**2) 
    i_xy = -sum(x * y) 
    i_yz = -sum(y * z) 
    i_xz = -sum(x * z) 

    inertial_data = {}

    inertial_data["max_dimensions"] = [
        max(x) - min(x),
        max(y) - min(y),
        max(z) - min(z),
    ]

    inertial_data["volume"] = N

    inertial_data["center_of_mass"] = Center_of_Mass

    inertial_data["inertia_tensor"] = {
        "i_xx": i_xx,
        "i_yy": i_yy,
        "i_zz": i_zz,
        "i_xy": i_xy,
        "i_xz": i_xz,
        "i_yz": i_yz,
    }

    return inertial_data


if __name__ == "__main__":
    # file_path = "../test_samples/bushing_voxel_125x300x125.txt"
    # file_path = "../test_samples/bushing_voxel_167x400x167.txt"
    
    #file_path = "../test_samples/box_90x60x30_voxel_9x3x6.txt"
    #file_path = "../test_samples/box_90x60x30_voxel_90x30x60.txt"
    file_path = "../test_samples/box_90x60x30_voxel_900x300x600.txt"

    file_path = os.path.join(os.path.dirname(__file__), file_path)
    inertial_data = extract_voxel_data(file_path)
    print("output for " + file_path)

    print("{")
    for label, value in inertial_data.items():
        print(f"\t'{label}': {str(value)},")
    print("}")

    print_inertial(scale_mesh_data(inertial_data, {'measured_mass_kg': 0.2, 'dimension_choice': 'z', 'measured_dimension_m': 0.06}))

