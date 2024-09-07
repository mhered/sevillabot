import numpy as np

def inertia_tensor_bushing(r_in, r_out, h, m):
    # Calculate moments of inertia of a bushing of mass m, height r, inner radius r_in and outer radius r_out
    i_zz = 0.5 * m * (r_in**2 + r_out**2)
    i_xx = i_yy = (1/12) * m * (3 * (r_in**2 + r_out**2) + h**2)
    
    # Construct the inertia tensor
    inertia_tensor = np.array([
        [i_xx, 0, 0],
        [0, i_yy, 0],
        [0, 0, i_zz]
    ])
    
    return inertia_tensor

if __name__ == "__main__":
    r1 = 1/2*15.0e-3  # Inner radius in m
    r2 = 1/2*25.0e-3  # Outer radius in m
    h = 60.0e-3   # Height in m
    m = 0.2  # Mass in kg

    inertia_tensor = inertia_tensor_bushing(r1, r2, h, m)
    print("Inertia Tensor:\n", inertia_tensor)

# Inertia Tensor calculated by ChatGPT:
#  [[7.0625e-05 0.0000e+00 0.0000e+00]
#  [0.0000e+00 7.0625e-05 0.0000e+00]
#  [0.0000e+00 0.0000e+00 2.1250e-05]]
