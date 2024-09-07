#!/usr/bin/env python

import re
import os

# Temporarily add the '../src' directory to sys.path
os.sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from utils_inertia import scale_mesh_data, print_inertial

def extract_meshlab_data(file_path):
    """
    Extracts data from a MeshLab-generated text file
    :param file_path: Path to the text file
    :return: Dictionary containing the extracted data
    """

    with open(file_path, 'r') as file:
        data = file.read()
    
    mesh_data = {}

    # Extracting dimensions from Mesh Bounding Box Size
    match = re.search(r'Mesh Bounding Box Size ([\d.]+) ([\d.]+) ([\d.]+)', data)
    if match:
        mesh_data['max_dimensions'] = [float(match.group(1)), float(match.group(2)), float(match.group(3))]

    # Extracting volume from Mesh Volume
    match = re.search(r'Mesh Volume is ([\d.]+)', data)
    if match:
        mesh_data['volume'] = float(match.group(1))

    # Extracting Center of Mass
    match = re.search(r'Center of Mass is ([\d.-]+) ([\d.-]+) ([\d.-]+)', data)
    if match:
        mesh_data['center_of_mass'] = [float(match.group(1)), float(match.group(2)), float(match.group(3))]

     # Extracting Inertia Tensor
    inertia_match = re.search(r'Inertia Tensor is :\s*\|\s*([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s*\|\s*\|\s*([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s*\|\s*\|\s*([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s*\|', data)
    if inertia_match:
        i_xx = float(inertia_match.group(1))
        i_xy = float(inertia_match.group(2))
        i_xz = float(inertia_match.group(3))
        i_yy = float(inertia_match.group(5))
        i_yz = float(inertia_match.group(6))
        i_zz = float(inertia_match.group(9))
        mesh_data['inertia_tensor'] = {
            'i_xx': i_xx,
            'i_yy': i_yy,
            'i_zz': i_zz,
            'i_xy': i_xy,
            'i_xz': i_xz,
            'i_yz': i_yz
        }

    return mesh_data


if __name__ == '__main__':
    file_path = os.path.join(os.path.dirname(__file__), '../test_samples/bushing_meshlab_data.txt')
    sample_data = extract_meshlab_data(file_path)
    print_inertial(scale_mesh_data( sample_data, {'measured_mass_kg': 0.2, 'dimension_choice': 'z', 'measured_dimension_m': 0.06}))
