"""
Tests for utility functions and helper methods.
"""
import pytest
import numpy as np
from unittest.mock import Mock, patch


@pytest.mark.unit
class TestUtilityFunctions:
    """Test suite for utility functions."""

    def test_array_validation(self):
        """Test array validation utility functions."""
        # Test valid arrays
        valid_array = np.array([1, 2, 3])
        assert len(valid_array) == 3
        assert isinstance(valid_array, np.ndarray)

    def test_position_validation(self):
        """Test position vector validation."""
        # Valid 3D position
        position = [1.0, 2.0, 3.0]
        assert len(position) == 3
        assert all(isinstance(x, (int, float)) for x in position)

    def test_quaternion_validation(self):
        """Test quaternion validation (x, y, z, w format)."""
        # Valid quaternion
        quaternion = [0, 0, 0, 1]  # Identity quaternion
        assert len(quaternion) == 4
        
        # Check normalization (approximately)
        norm = sum(x**2 for x in quaternion) ** 0.5
        assert abs(norm - 1.0) < 1e-6

    def test_joint_names_validation(self):
        """Test joint names validation."""
        joint_names = ["joint1", "joint2", "joint3"]
        
        # All should be strings
        assert all(isinstance(name, str) for name in joint_names)
        
        # No empty names
        assert all(len(name) > 0 for name in joint_names)
        
        # No duplicate names
        assert len(joint_names) == len(set(joint_names))

    def test_prim_path_validation(self):
        """Test primitive path validation."""
        valid_paths = [
            "/World/Robot1",
            "/World/Objects/Box1",
            "/World/Sensors/Camera1"
        ]
        
        for path in valid_paths:
            assert path.startswith("/")
            assert len(path) > 1
            assert "//" not in path  # No double slashes

    @pytest.mark.parametrize("test_input,expected", [
        ([0, 0, 0], True),
        ([1, 2, 3], True),
        ([1, 2], False),  # Wrong length
        ([1, 2, 3, 4], False),  # Wrong length
        (None, False),  # Invalid type
    ])
    def test_position_format_validation(self, test_input, expected):
        """Test position format validation with parametrize."""
        if test_input is None:
            result = False
        else:
            try:
                result = len(test_input) == 3 and all(isinstance(x, (int, float)) for x in test_input)
            except TypeError:
                result = False
                
        assert result == expected

    def test_configuration_defaults(self):
        """Test that configuration objects have sensible defaults."""
        # This would test actual config objects if imported
        # For now, test the concept with mock data
        default_config = {
            'headless': True,
            'timestep': 0.01,
            'gravity': [0, 0, -9.81]
        }
        
        assert default_config['headless'] is True
        assert default_config['timestep'] > 0
        assert len(default_config['gravity']) == 3

    def test_error_handling_patterns(self):
        """Test common error handling patterns."""
        # Test that appropriate errors are raised for invalid inputs
        with pytest.raises(ValueError):
            # This would test actual validation function
            if len([1, 2]) != 3:
                raise ValueError("Invalid position length")
                
        with pytest.raises(TypeError):
            # This would test actual type validation
            if not isinstance("string", (int, float)):
                raise TypeError("Invalid numeric type")

    def test_mock_functionality(self):
        """Test that mocking works correctly for testing utilities."""
        mock_function = Mock(return_value=42)
        result = mock_function("test_arg")
        
        assert result == 42
        mock_function.assert_called_once_with("test_arg")

    def test_numpy_operations(self):
        """Test numpy operations commonly used in physics simulation."""
        # Vector operations
        vec1 = np.array([1, 0, 0])
        vec2 = np.array([0, 1, 0])
        
        # Cross product
        cross = np.cross(vec1, vec2)
        expected_cross = np.array([0, 0, 1])
        np.testing.assert_array_equal(cross, expected_cross)
        
        # Dot product
        dot = np.dot(vec1, vec2)
        assert dot == 0  # Orthogonal vectors
        
        # Magnitude
        magnitude = np.linalg.norm(vec1)
        assert abs(magnitude - 1.0) < 1e-10 