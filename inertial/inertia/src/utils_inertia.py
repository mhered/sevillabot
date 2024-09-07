#!/usr/bin/env python

def interactive_input(max_dimensions, testing=False):
    measured_mass_kg = float(input("Enter the measured mass (in kg): "))

    if testing==False:
        print(f"Mesh max_dimensions vector: {max_dimensions}")
    
    dimension_choice = input("Enter the dimension to scale (x, y, z)?: ").strip().lower()

    if dimension_choice not in ['x', 'y', 'z']:
        raise ValueError("Invalid dimension choice. Please enter 'x', 'y', or 'z'.")

    measured_dimension_m = float(input("Enter the measured dimension for scale (in m) : "))
    
    result = {
        'measured_mass_kg': measured_mass_kg, 
        'dimension_choice': dimension_choice, 
        'measured_dimension_m': measured_dimension_m
        }
    return result


def scale_mesh_data(mesh_data, params=None, testing=False):
    if params is None:
        params = interactive_input(mesh_data['max_dimensions'],testing)

    measured_mass_kg = params['measured_mass_kg']
    dimension_choice = params['dimension_choice']

    if dimension_choice == 'x':
        original_dimension = mesh_data['max_dimensions'][0]
    elif dimension_choice == 'y':
        original_dimension = mesh_data['max_dimensions'][1]
    elif dimension_choice == 'z':
        original_dimension = mesh_data['max_dimensions'][2]
    else:
        raise ValueError("Invalid dimension choice. Please enter 'x', 'y', or 'z'.")

    measured_dimension_m = params['measured_dimension_m']
    
    scale_factor = measured_dimension_m / original_dimension

    density_kg_m3 = measured_mass_kg / (mesh_data['volume'] * scale_factor**3)

    scaled_mesh_data = mesh_data.copy()
    scaled_mesh_data['density_kg_m3'] = density_kg_m3
    scaled_mesh_data['scale_factor'] = scale_factor

    scaled_mesh_data['mass_kg'] = measured_mass_kg
    
    scaled_mesh_data['center_of_mass_m'] = [coord * scale_factor for coord in mesh_data['center_of_mass']] 
    
    # Scaling the inertia tensor components
    scaled_mesh_data['inertia_tensor_kg_m2'] = scaled_mesh_data['inertia_tensor']
    for item in ['i_xx', 'i_yy', 'i_zz', 'i_xy', 'i_xz', 'i_yz']:
        scaled_mesh_data['inertia_tensor_kg_m2'][item] *= density_kg_m3 * scale_factor**5


    return scaled_mesh_data


def print_inertial(scaled_mesh_data):
    # Extracting and formatting the center of mass
    center_of_mass = scaled_mesh_data.get('center_of_mass_m', [0, 0, 0])
    xyz = f'{center_of_mass[0]:.5e} {center_of_mass[1]:.5e} {center_of_mass[2]:.5e}'
    
    # Extracting the volume (used as mass here)
    volume = scaled_mesh_data.get('mass_kg', 0)
    
    # Extracting and formatting the inertia tensor components
    inertia_tensor = scaled_mesh_data.get('inertia_tensor_kg_m2', {})
    i_xx = inertia_tensor.get('i_xx', 0)
    i_yy = inertia_tensor.get('i_yy', 0)
    i_zz = inertia_tensor.get('i_zz', 0)
    i_xy = inertia_tensor.get('i_xy', 0)
    i_xz = inertia_tensor.get('i_xz', 0)
    i_yz = inertia_tensor.get('i_yz', 0)
    
    # Printing in the specified format
    print(f'<inertial>')
    print(f'    <origin xyz="{xyz}" rpy="0 0 0" />')
    print(f'    <mass value="{volume:.5e}"/>')
    print(f'    <inertia ixx="{i_xx:.7e}" ixy="{i_xy:.7e}" ixz="{i_xz:.7e}" iyy="{i_yy:.7e}" iyz="{i_yz:.7e}" izz="{i_zz:.7e}"/>')
    print(f'</inertial>')
