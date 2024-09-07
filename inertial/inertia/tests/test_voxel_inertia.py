#!/usr/bin/env python

import unittest
import numpy as np
import tempfile
import os
from src.voxel_inertia import extract_voxel_data
from src.utils_inertia import scale_mesh_data

# Temporarily add the '../src' directory to sys.path
os.sys.path.append(os.path.join(os.path.dirname(__file__), "../src"))


class TestInertialVoxel(unittest.TestCase):

    def setUp(self):
        # Create a temporary file to hold test data
        self.test_file = tempfile.NamedTemporaryFile(delete=False, suffix=".csv")

    def tearDown(self):
        # Clean up the temporary file
        os.remove(self.test_file.name)

    def test_inertial_voxel_correct_output(self):
        # Test data: simple cubic shape
        test_data = np.array(
            [
                [0, 0, 0],
                [0, 1, 0],
                [1, 0, 0],
                [1, 1, 0],
                [0, 0, 1],
                [0, 1, 1],
                [1, 0, 1],
                [1, 1, 1],
            ]
        )

        np.savetxt(self.test_file.name, test_data, fmt="%d", delimiter=",")

        result = extract_voxel_data(self.test_file.name)

        expected_center_of_mass = (0, 0, 0)  # After centering, should be at origin
        expected_volume = 8  # 8 voxels
        expected_max_dimensions = [1, 1, 1]  # Each dimension is 1

        # Check if center of mass is calculated correctly
        self.assertEqual(result["center_of_mass"], expected_center_of_mass)

        # Check if volume is correct
        self.assertEqual(result["volume"], expected_volume)

        # Check if max dimensions are correct
        self.assertEqual(result["max_dimensions"], expected_max_dimensions)

        # Check inertia tensor calculation (specific values will depend on exact implementation)
        # In this simple case, just verify that keys are present
        self.assertIn("i_xx", result["inertia_tensor"])
        self.assertIn("i_yy", result["inertia_tensor"])
        self.assertIn("i_zz", result["inertia_tensor"])
        self.assertIn("i_xy", result["inertia_tensor"])
        self.assertIn("i_xz", result["inertia_tensor"])
        self.assertIn("i_yz", result["inertia_tensor"])

    def test_inertial_voxel_invalid_file(self):
        # Test with a non-existent file
        with self.assertRaises(IOError):
            extract_voxel_data("non_existent_file.csv")

    def test_inertial_voxel_invalid_data(self):
        # Write invalid data to the file
        with open(self.test_file.name, "w") as f:
            f.write("invalid,data,content")

        # Test with invalid data in file
        with self.assertRaises(ValueError):
            extract_voxel_data(self.test_file.name)

    def test_inertial_voxel_insufficient_columns(self):
        # Test with only two columns of data (invalid format)
        test_data = np.array([[0, 0], [1, 1]])

        np.savetxt(self.test_file.name, test_data, fmt="%d", delimiter=",")

        # Test with insufficient data columns
        with self.assertRaises(ValueError):
            extract_voxel_data(self.test_file.name)


class TestExtractMeshLabData(unittest.TestCase):

    def setUp(self):
        # Path to the test sample file
        self.sample_file_path = os.path.join(
            os.path.dirname(__file__), "../test_samples/bushing_voxel_167x400x167.txt"
        )

    def test_extract_voxel_data(self):

        # Theoretical values for comparison
        expected_dimensions = [167, 167, 400]
        expected_volume = 5.62e6
        expected_center_of_mass = [0.0, 0.0, 0.0]
        expected_inertia_tensor = {
            "i_xx": 1.5701e13,
            "i_yy": 1.5701e13,
            "i_zz": 4.7361e12,
            "i_xy": 0.0,
            "i_xz": 0.0,
            "i_yz": 0.0
        }

        # Call the function to extract data from the sample file
        result = extract_voxel_data(self.sample_file_path)

        # Relative tolerance for floating point comparisons
        rel_tolerance = 1e-2  # 1%
        abs_tolerance = 1e-1  # 0.1

        np.testing.assert_allclose(
            np.array(result["max_dimensions"]),
            expected_dimensions,
            rtol=rel_tolerance,
            atol=abs_tolerance,
        )

        # Assert the extracted center of mass
        np.testing.assert_allclose(
            np.array(result["center_of_mass"]),
            expected_center_of_mass,
            rtol=rel_tolerance,
            atol=abs_tolerance,
        )

        # Assert the extracted volume
        np.testing.assert_allclose(
            result["volume"],
            expected_volume, 
            rtol=rel_tolerance, 
            atol=abs_tolerance
        )

        # Assert the extracted inertia tensor

        # Convert to arrays respecting the order of keys
        keys_order = ["i_xx", "i_yy", "i_zz", "i_xy", "i_xz", "i_yz"]

        expected_values = np.array([expected_inertia_tensor[key] for key in keys_order])
        result_values = np.array([result["inertia_tensor"][key] for key in keys_order])

        # element-wise comparison with relative tolerance
        np.testing.assert_allclose(
            result_values, expected_values, rtol=rel_tolerance, atol=abs_tolerance
        )

        # Alternatively, check the Frobenius norm of the difference with relative tolerance
        difference_norm = np.linalg.norm(result_values - expected_values)
        self.assertLess(
            difference_norm, 40*rel_tolerance * np.linalg.norm(expected_values)
        )
        