import numpy as np

import numpy as np

def calculate_new_scale_readings(l, w, h, mass, lift_height):
    total_weight = mass

    # Original center of mass
    x_cm = l / 2
    y_cm = w / 2
    z_cm = h / 2

    # New coordinates for point A after lifting
    z_A_prime = lift_height

    # New height of the center of mass
    z_cm_new = (z_cm + z_A_prime) / 2

    # Force equilibrium equation: F_A' + F_B' + F_M' = mg
    # Moment equilibrium around the edge BC (line connecting B and C)
    
    # New force distribution
    A_matrix = np.array([
        [l, l/2],
        [0, w]
    ])
    b_vector = np.array([total_weight * x_cm, total_weight * y_cm])

    # Solve for the new forces on B and M
    F_B_prime, F_M_prime = np.linalg.solve(A_matrix, b_vector)

    # Calculate the new force on A (now A' lifted)
    F_A_prime = total_weight - F_B_prime - F_M_prime

    return F_A_prime, F_B_prime, F_M_prime

# Example usage:
length = 100  # cm
width = 120   # cm
height = 40   # cm
mass = 100    # kg
lift_height = 30  # cm (lifting A by 30 cm)

F_A_prime, F_B_prime, F_M_prime = calculate_new_scale_readings(length, width, height, mass, lift_height)

print(f"New force on Scale A' (F_A'): {F_A_prime:.2f} Kg")
print(f"New force on Scale B (F_B'): {F_B_prime:.2f} Kg")
print(f"New force on Scale M (F_M'): {F_M_prime:.2f} Kg")
