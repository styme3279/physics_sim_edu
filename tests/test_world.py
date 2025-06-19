"""
Tests for World class.
"""
import pytest
from unittest.mock import Mock, patch, mock_open
from pathlib import Path


@pytest.mark.unit
class TestWorld:
    """Test suite for World class."""

    @patch('builtins.open', new_callable=mock_open, read_data='<mujoco></mujoco>')
    @patch('pathlib.Path.exists')
    def test_world_initialization(self, mock_exists, mock_file):
        """Test world initialization with XML loading."""
        mock_exists.return_value = True
        
        from physics_simulator.world.world import MujocoWorld
        
        world = MujocoWorld()
        assert world is not None

    @patch('builtins.open', new_callable=mock_open, read_data='<mujoco><worldbody></worldbody></mujoco>')
    @patch('pathlib.Path.exists')
    def test_get_model_mujoco_mode(self, mock_exists, mock_file):
        """Test getting model in MuJoCo mode."""
        mock_exists.return_value = True
        
        from physics_simulator.world.world import MujocoWorld
        
        world = MujocoWorld()
        model = str(world)
        
        # Should return the XML string content
        assert isinstance(model, str)
        assert "<mujoco>" in model

    def test_invalid_mode_raises_error(self):
        """Test that invalid mode raises ValueError."""
        with patch('builtins.open', new_callable=mock_open), \
             patch('pathlib.Path.exists', return_value=True):
            
            from physics_simulator.world.world import MujocoWorld
            world = MujocoWorld()
            
            # Test update_attribute with invalid tag
            with pytest.raises(ValueError, match="Invalid tag"):
                world.update_attribute("test", "value", tag="invalid_tag")

    @patch('builtins.open', new_callable=mock_open)
    @patch('pathlib.Path.exists')
    def test_missing_xml_file_handling(self, mock_exists, mock_file):
        """Test handling of missing XML file."""
        mock_exists.return_value = False
        
        from physics_simulator.world.world import MujocoWorld
        
        # Should handle missing file gracefully or raise appropriate error
        with pytest.raises((FileNotFoundError, IOError)):
            MujocoWorld()

    def test_world_with_custom_xml_path(self):
        """Test world initialization with custom XML path."""
        custom_xml = "<mujoco><worldbody><geom name='ground' type='plane' size='10 10 0.1'/></worldbody></mujoco>"
        
        with patch('builtins.open', new_callable=mock_open, read_data=custom_xml), \
             patch('pathlib.Path.exists', return_value=True):
            
            from physics_simulator.world.world import MujocoWorld
            
            world = MujocoWorld()
            model = str(world)
            
            assert "plane" in model
            assert "ground" in model 