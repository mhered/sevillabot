#!/usr/bin/env python

import os
import csv
import datetime

# Temporarily add the '../src' directory to sys.path
os.sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from utils_inertia import scale_mesh_data, print_inertial

def extract_freecad_data(file_path):
    """
    Extracts data from a FreeCAD-generated CSV file
    :param file_path: Path to the CSV file
    :return: Dictionary containing the extracted data
    """

    result = {}
    last_block_data = {}
    latest_timestamp = None
    in_last_block = False

    with open(file_path, 'r') as file:
        reader = csv.reader(file, delimiter='\t')
        
        for row in reader:
            if len(row) > 0 and row[0].startswith("________________________"):
                # Start of a new block, reset last_block_data
                last_block_data = {}
                in_last_block = True
            elif len(row) > 0 and '/' in row[0] and ':' in row[0]:
                # This is the timestamp line

                # Extract the timestamp from the first cell
                # Parse the timestamp from the current block
                current_timestamp = datetime.datetime.strptime(row[0], "%d/%m/%Y %H:%M:%S")

                # Compare with the latest stored timestamp
                if latest_timestamp is None or current_timestamp > latest_timestamp:
                    latest_timestamp = current_timestamp
                    in_last_block = True
                    last_block_data = {}  # Reset the data for the new block if it is more recent
                else:
                    in_last_block = False  # Skip processing this block if it's older

            elif in_last_block:
                if len(row)<1: 
                    continue # skip empty rows
                elif row[0].startswith("Volume of the form"):
                    # Extracting volume from Mesh Volume
                    last_block_data['volume'] = float(row[1].split()[0])  
                elif row[0].startswith("Mass"):
                    continue
                    # last_block_data['mass'] = float(row[1].split()[0])
                elif row[0].startswith("Center of mass"):
                    # Extracting Center of Mass
                    last_block_data['center_of_mass'] = [
                        float(row[2].split()[0]),
                        float(row[5].split()[0]),
                        float(row[8].split()[0])
                    ]
                elif row[0].startswith("Overall dimensions"):
                    # Extracting dimensions from Mesh Bounding Box Size
                    last_block_data['max_dimensions'] = [
                        float(row[2].split()[0]),
                        float(row[5].split()[0]),
                        float(row[8].split()[0])
                    ]
                elif row[0].startswith("Matrix of inertia"):
                    # Extracting Inertia Tensor
                    # process the next 3 rows for inertia tensor
                    ix_row = next(reader)
                    iy_row = next(reader)
                    iz_row = next(reader)

                    last_block_data['inertia_tensor'] = {
                        'i_xx': float(ix_row[1].split()[0]),
                        'i_xy': float(ix_row[3].split()[0]),
                        'i_xz': float(ix_row[5].split()[0]),
                        'i_yy': float(iy_row[3].split()[0]),
                        'i_yz': float(iy_row[5].split()[0]),
                        'i_zz': float(iz_row[5].split()[0])
                    }

    # After the loop, last_block_data should contain the info from the last block
    result = last_block_data
    return result


if __name__ == '__main__':
    file_path = os.path.join(os.path.dirname(__file__), '../test_samples/bushing_freecad_data.csv')
    sample_data = extract_freecad_data(file_path)
    print_inertial(scale_mesh_data( sample_data)) #, {'measured_mass_kg': 0.2, 'dimension_choice': 'z', 'measured_dimension_m': 0.06}))
