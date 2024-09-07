import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def calculate_rotation_matrix(point1, point2, point3):
    
    # Calculate vectors v1 and v2
    v1 = point2 - point1
    v2 = point3 - point1
    
    # Calculate the normalized normal vector to the plane
    normal = np.cross(v1, v2)
    normal = normal / np.linalg.norm(normal)
    
    # The target normal vector (pointing in the positive z direction)
    n_target = np.array([0, 0, 1])
    
    # Calculate the normalized axis of rotation (cross product of normal vector and z-axis )
    rotation_axis = np.cross(normal, n_target)
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    
    # Calculate the angle between the normal vector and z-axis (dot product)
    cos_theta = np.dot(normal, n_target)

    # Handle special cases
    if np.isclose(cos_theta, 1):
        return np.eye(3)  # No rotation needed
    elif np.isclose(cos_theta, -1):
        # 180-degree rotation around any axis perpendicular to z-axis and n_norm
        return -np.eye(3)
    
    # Calculate the rotation angle (theta) using arccos
    theta = np.arccos(cos_theta)
    
    # Adjust the angle to ensure it's the smallest possible rotation
    if theta > np.pi / 2:
        theta = np.pi - theta
        rotation_axis = -rotation_axis  # Flip the axis to rotate in the opposite direction

    # Construct the skew-symmetric matrix for the rotation axis
    K = np.array([
        [0, -rotation_axis[2], rotation_axis[1]],
        [rotation_axis[2], 0, -rotation_axis[0]],
        [-rotation_axis[1], rotation_axis[0], 0]
    ])
    
    # Construct the rotation matrix using Rodrigues' rotation formula
    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
    
    return R

def transform_point(R, point):
    # Apply the rotation matrix R
    return np.dot(R, point)

def inverse_transform_point(R, new_point):
    # Apply the inverse (transpose) rotation matrix to get the original coordinates
    return np.dot(R.T, new_point)

def calculate_distance(point1, point2):
    
    # Calculate the Euclidean distance between two points
    return np.linalg.norm(point2 - point1)

def set_axes_equal(ax):
    """Set 3D plot axes to have equal scale so that the points appear properly scaled."""
    
    # Get the current limits of the axes
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    
    # Compute the range for each axis
    x_range = x_limits[1] - x_limits[0]
    y_range = y_limits[1] - y_limits[0]
    z_range = z_limits[1] - z_limits[0]
    
    # Find the maximum range
    max_range = max(x_range, y_range, z_range)
    
    # Set the center of each axis
    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)
    
    # Update the axes limits to be equal around the center
    ax.set_xlim3d([x_middle - max_range / 2, x_middle + max_range / 2])
    ax.set_ylim3d([y_middle - max_range / 2, y_middle + max_range / 2])
    ax.set_zlim3d([z_middle - max_range / 2, z_middle + max_range / 2])

def plot_3d_triangle(ax, point1, point2, point3, point_color='red', area_color='cyan'):
    # Unpack the points into coordinates
    x = [point1[0], point2[0], point3[0]]
    y = [point1[1], point2[1], point3[1]]
    z = [point1[2], point2[2], point3[2]]
    
    # Plot the triangle
    verts = [list(zip(x, y, z))]
    ax.add_collection3d(Poly3DCollection(verts, color=area_color, alpha=0.7))
    
    # Plot the vertices
    ax.scatter(x, y, z, color=point_color, s=50)
    
    # Set labels for the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Update aspect ratio
    set_axes_equal(ax)

def plot_3d_point(ax, point, point_color='blue'):
    
    # Plot the point
    ax.scatter(point[0], point[1], point[2], color=point_color, s=50)
    
    # Optionally, you could label the point for clarity (uncomment if needed)
    ax.text(point[0], point[1], point[2], f'({point[0]}, {point[1]}, {point[2]})')

    # Ensure the axis labels are set (for consistency with other plots)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

def plot_triangles(point1, point2, point3, point4, new_point1, new_point2, new_point3, new_point4):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plot_3d_triangle(ax, point1, point2, point3, point_color='blue', area_color='cyan')
    plot_3d_point(ax, point4, point_color='blue')
    plot_3d_triangle(ax, new_point1, new_point2, new_point3, point_color='red', area_color='magenta')
    plot_3d_point(ax, new_point4, point_color='red')
    ax.set_title('Original and Rotated Triangles')
    # Display the plot
    plt.show(block=False)

def calculate_rotation_matrix_lift_vertex(A, B, C, height):
    # Calculate vectors B -> C and B -> A
    BC = C - B
    BA = A - B
    
    # Calculate the distance h from A to the line BC
    h = np.linalg.norm(np.cross(BA, BC)) / np.linalg.norm(BC)
    
    # Consistency check: Ensure height <= h
    if abs(height) > h:
        raise ValueError(f"Height d={height} is greater than the distance from A to the line BC. Maximum allowable height is {h}.")
    
    # Calculate the rotation axis (the unit vector along BC)
    rotation_axis = BC / np.linalg.norm(BC)
    
    # Calculate the current normal vector to the plane (pointing upwards)
    normal_original = np.cross(BC, BA)
    if normal_original[2] < 0:  # Ensure the normal is pointing upwards
        normal_original = -normal_original
    normal_original /= np.linalg.norm(normal_original)

    # The desired normal vector after the lift
    desired_normal = np.array([0, 0, 1])  # We want to lift A upwards

    # Calculate the rotation angle theta
    theta = np.arcsin(height / h)

        # Adjust theta if the current normal is already pointing upward
    # We want to ensure the smallest positive rotation that achieves the desired height
    if np.dot(normal_original, desired_normal) < 0:
        theta = -theta
    
    # Construct the skew-symmetric matrix for the rotation axis
    K = np.array([
        [0, -rotation_axis[2], rotation_axis[1]],
        [rotation_axis[2], 0, -rotation_axis[0]],
        [-rotation_axis[1], rotation_axis[0], 0]
    ])
    
    # Construct the rotation matrix using Rodrigues' rotation formula
    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
    
    return R

def check_distances_preserved(point1, point2, point3, point4, new_point1, new_point2, new_point3, new_point4):
    # check distances are preserved after rotation
    check1 = calculate_distance(point1, point2) - calculate_distance(new_point1, new_point2)
    check2 = calculate_distance(point1, point3) - calculate_distance(new_point1, new_point3)
    check3 = calculate_distance(point2, point3) - calculate_distance(new_point2, new_point3)

    check4 = calculate_distance(point1, point4) - calculate_distance(new_point1, new_point4)
    check5 = calculate_distance(point2, point4) - calculate_distance(new_point2, new_point4)
    check6 = calculate_distance(point3, point4) - calculate_distance(new_point3, new_point4)

    checks = [check1, check2, check3, check4, check5, check6]

    THRESHOLD = 1e-10
    if any(abs(c)> THRESHOLD for c in checks) :
        print("Distances are not preserved after rotation.")
    else:
        print("Distances are preserved after rotation.")

def tilt(new_point1, height, new_point2, new_point3, new_point4):

        # tilt lifting point1 by a distance height
        R_tilt1=calculate_rotation_matrix_lift_vertex(new_point1, new_point2, new_point3, height)

        # Apply the rotation to point1
        new_point1_tilt1 = np.dot(R_tilt1, new_point1 - new_point2) + new_point2  # Rotate point1 around the axis point2-point3
        new_point2_tilt1 = new_point2  # point2 remains fixed
        new_point3_tilt1 = new_point3 # point3 remains fixed
        new_point4_tilt1 = np.dot(R_tilt1, new_point4 - new_point2) + new_point2  # Rotate point4 around the axis point2-point3

        # show the points, they should all lie in the XY plane
        print("New coordinates of point 1:", new_point1_tilt1)
        print("New coordinates of point 2:", new_point2_tilt1)
        print("New coordinates of point 3:", new_point3_tilt1)

        check_distances_preserved(new_point1, new_point2, new_point3, new_point4, 
                                new_point1_tilt1, new_point2_tilt1, new_point3_tilt1, new_point4_tilt1)

        # Display the plot
        plot_triangles(new_point1, new_point2, new_point3, new_point4,
                        new_point1_tilt1, new_point2_tilt1, new_point3_tilt1, new_point4_tilt1)

if __name__ == "__main__":
    # Example usage:
    # take three points in 3D space representing the contact points for weighing the object in CAD coordinates
    # point1 = np.array([0, 1, 0])
    # point2 = np.array([1, 1, 0])
    # point3 = np.array([0, 1, 2])

    point1 = np.array([10, 260, 0])
    point2 = np.array([210, 260, 0])
    point3 = np.array([110, 30, 10])

    # optional: add a fourth point to check that distances are preserved after rotation
    point4 = (point1 +point2 +point3 )/3

    # calculate the rotation matrix to align the triangle with the XY plane
    rotation_matrix = calculate_rotation_matrix(point1, point2, point3)
    print("Rotation Matrix:")
    print(rotation_matrix)

    # calculate the new coordinates of the points after rotation
    new_point1 = transform_point(rotation_matrix, point1)
    new_point2 = transform_point(rotation_matrix, point2)
    new_point3 = transform_point(rotation_matrix, point3)
    new_point4 = transform_point(rotation_matrix, point4)

    # show the points, they should all lie in the XY plane
    print("New coordinates of point 1:", new_point1)
    print("New coordinates of point 2:", new_point2)
    print("New coordinates of point 3:", new_point3)

    check_distances_preserved(point1, point2, point3, point4, new_point1, new_point2, new_point3, new_point4)


    # check inverse transformation calculating the original coordinates of the points
    print("Original coordinates of point 1:", inverse_transform_point(rotation_matrix, new_point1))
    print("Original coordinates of point 2:", inverse_transform_point(rotation_matrix, new_point2))
    print("Original coordinates of point 3:", inverse_transform_point(rotation_matrix, new_point3))


    # Plot the original and rotated triangles
    # plot_triangles(point1, point2, point3, point4, new_point1, new_point2, new_point3, new_point4)

    height = 80  # height to lift

    tilt(new_point1, height, new_point3, new_point2, new_point4)
    tilt(new_point2, height, new_point1, new_point3, new_point4)
    tilt(new_point3, height, new_point2, new_point1, new_point4)

    # pending:
    # - generate synthetic case of coords, cog and weighings
    # - create function that calculates cog based on coords and weighings with 3 scales
    # - run 3 times, 
    # - average results

    print("Original coordinates of point 4:", inverse_transform_point(rotation_matrix, new_point4))


    plt.show()