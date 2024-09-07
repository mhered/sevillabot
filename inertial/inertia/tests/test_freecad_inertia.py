#!/usr/bin/env python

import unittest
import os
import numpy as np

# Temporarily add the '../src' directory to sys.path
os.sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from src.freecad_inertia import extract_freecad_data


class TestExtractFreecadData(unittest.TestCase):
    
    def setUp(self):
        # Path to the test sample file
        self.sample_file_path = os.path.join(os.path.dirname(__file__), '../test_samples/bushing_freecad_data.csv')
    
    def test_extract_freecad_data(self):
        
        # Theoretical values for comparison
        expected_dimensions = [25.0, 25.0, 60.0]  # Adjust based on the expected bounding box size
        expected_volume = 18849.56
        expected_center_of_mass = [0.0, 0.0, 30.0]
        expected_inertia_tensor = {
            'i_xx': 6656250.51,
            'i_yy': 6657125.51,
            'i_zz': 2002753.93,
            'i_xy': 0.0,
            'i_xz': 0.0,
            'i_yz': 0.0
        }

        # Call the function to extract data from the sample file
        result = extract_freecad_data(self.sample_file_path)
        
        # Relative tolerance for floating point comparisons
        rel_tolerance = 1e-2 # 1%
        abs_tolerance = 1e-1 # 0.1

        # Assert the extracted dimensions
        np.testing.assert_allclose(result['max_dimensions'], expected_dimensions, rtol=rel_tolerance, atol=abs_tolerance)
        
        # Assert the extracted center of mass
        np.testing.assert_allclose(result['center_of_mass'], expected_center_of_mass, rtol=rel_tolerance, atol=abs_tolerance)
        
        # Assert the extracted volume
        np.testing.assert_allclose(result['volume'], expected_volume, rtol=rel_tolerance, atol=abs_tolerance)

        # Assert the extracted inertia tensor

        # Convert to arrays respecting the order of keys
        keys_order = ['i_xx', 'i_yy', 'i_zz', 'i_xy', 'i_xz', 'i_yz']

        expected_values = np.array([expected_inertia_tensor[key] for key in keys_order])
        result_values = np.array([result['inertia_tensor'][key] for key in keys_order])

        # element-wise comparison with relative tolerance
        np.testing.assert_allclose(result_values, expected_values, rtol=rel_tolerance, atol=abs_tolerance)

        # Alternatively, check the Frobenius norm of the difference with relative tolerance
        difference_norm = np.linalg.norm(result_values - expected_values)
        self.assertLess(difference_norm, rel_tolerance * np.linalg.norm(expected_values))

if __name__ == '__main__':
    unittest.main()
