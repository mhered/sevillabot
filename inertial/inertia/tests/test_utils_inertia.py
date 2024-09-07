#!/usr/bin/env python

import unittest
import os
from unittest.mock import patch

# Temporarily add the '../src' directory to sys.path
os.sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from src.utils_inertia import interactive_input, scale_mesh_data

class TestMeshScaling(unittest.TestCase):

    @patch('builtins.input', side_effect=['0.2', 'x', '0.06'])
    def test_interactive_input(self, mock_input):
        max_dimensions = [0.1, 0.2, 0.3]
        result = interactive_input(max_dimensions, testing=True)
        
        expected_result = {
            'measured_mass_kg': 0.2,
            'dimension_choice': 'x',
            'measured_dimension_m': 0.06
        }
        
        self.assertEqual(result, expected_result)

    @patch('builtins.input', side_effect=['0.2', 'invalid_dimension', '0.06'])
    def test_interactive_input_invalid_dimension(self, mock_input):
        max_dimensions = [0.1, 0.2, 0.3]
        with self.assertRaises(ValueError):
            interactive_input(max_dimensions, testing=True)

    def test_scale_mesh_data_with_params(self):
        mesh_data = {
            'max_dimensions': [0.1, 0.2, 0.3],
            'volume': 0.002,
            'center_of_mass': [0.05, 0.1, 0.15],
            'inertia_tensor': {
                'i_xx': 1e-4,
                'i_yy': 2e-4,
                'i_zz': 3e-4,
                'i_xy': 4e-5,
                'i_xz': 5e-5,
                'i_yz': 6e-5
            }
        }
        
        params = {
            'measured_mass_kg': 0.2,
            'dimension_choice': 'x',
            'measured_dimension_m': 0.06
        }
        
        scaled_data = scale_mesh_data(mesh_data, params)
        
        expected_scale_factor = 0.06 / 0.1
        expected_density_kg_m3 = 0.2 / (0.002 * expected_scale_factor ** 3)
        expected_inertia_tensor = {
            'i_xx': 1e-4 * expected_density_kg_m3 * expected_scale_factor ** 5,
            'i_yy': 2e-4 * expected_density_kg_m3 * expected_scale_factor ** 5,
            'i_zz': 3e-4 * expected_density_kg_m3 * expected_scale_factor ** 5,
            'i_xy': 4e-5 * expected_density_kg_m3 * expected_scale_factor ** 5,
            'i_xz': 5e-5 * expected_density_kg_m3 * expected_scale_factor ** 5,
            'i_yz': 6e-5 * expected_density_kg_m3 * expected_scale_factor ** 5
        }
        
        self.assertEqual(scaled_data['mass_kg'], 0.2)
        self.assertAlmostEqual(scaled_data['density_kg_m3'], expected_density_kg_m3)
        self.assertAlmostEqual(scaled_data['scale_factor'], expected_scale_factor)
        self.assertAlmostEqual(scaled_data['center_of_mass_m'], [0.05 * expected_scale_factor, 0.1 * expected_scale_factor, 0.15 * expected_scale_factor])

        for key in expected_inertia_tensor:
            self.assertAlmostEqual(scaled_data['inertia_tensor_kg_m2'][key], expected_inertia_tensor[key], places=6)

    @patch('builtins.input', side_effect=['0.2', 'x', '0.06'])
    def test_scale_mesh_data_with_interactive_input(self, mock_input):
        mesh_data = {
            'max_dimensions': [0.1, 0.2, 0.3],
            'volume': 0.002,
            'center_of_mass': [0.05, 0.1, 0.15],
            'inertia_tensor': {
                'i_xx': 1e-4,
                'i_yy': 2e-4,
                'i_zz': 3e-4,
                'i_xy': 4e-5,
                'i_xz': 5e-5,
                'i_yz': 6e-5
            }
        }
        
        scaled_data = scale_mesh_data(mesh_data,testing=True)
        
        expected_scale_factor = 0.06 / 0.1
        expected_density_kg_m3 = 0.2 / (0.002 * expected_scale_factor ** 3)
        expected_inertia_tensor = {
            'i_xx': 1e-4 * expected_density_kg_m3 * expected_scale_factor ** 5,
            'i_yy': 2e-4 * expected_density_kg_m3 * expected_scale_factor ** 5,
            'i_zz': 3e-4 * expected_density_kg_m3 * expected_scale_factor ** 5,
            'i_xy': 4e-5 * expected_density_kg_m3 * expected_scale_factor ** 5,
            'i_xz': 5e-5 * expected_density_kg_m3 * expected_scale_factor ** 5,
            'i_yz': 6e-5 * expected_density_kg_m3 * expected_scale_factor ** 5
        }
        
        self.assertEqual(scaled_data['mass_kg'], 0.2)

        self.assertAlmostEqual(scaled_data['density_kg_m3'], expected_density_kg_m3)
        self.assertAlmostEqual(scaled_data['scale_factor'], expected_scale_factor)
        self.assertAlmostEqual(scaled_data['center_of_mass_m'], [0.05 * expected_scale_factor, 0.1 * expected_scale_factor, 0.15 * expected_scale_factor])

        for key in expected_inertia_tensor:
            self.assertAlmostEqual(scaled_data['inertia_tensor_kg_m2'][key], expected_inertia_tensor[key], places=6)


if __name__ == '__main__':
    unittest.main()
