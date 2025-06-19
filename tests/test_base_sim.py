"""
Tests for BaseSim abstract base class.
"""
import pytest
from abc import ABC
from unittest.mock import Mock
from physics_simulator.simulator.base_sim import BaseSim
from synthnova_config import PhysicsSimulatorConfig


@pytest.mark.unit
class TestBaseSim:
    """Test suite for BaseSim abstract base class."""

    def test_is_abstract_class(self):
        """Test that BaseSim is an abstract class."""
        assert issubclass(BaseSim, ABC)
        
        # Should not be able to instantiate directly
        with pytest.raises(TypeError):
            BaseSim(PhysicsSimulatorConfig())

    def test_abstract_methods_exist(self):
        """Test that all required abstract methods are defined."""
        abstract_methods = {
            '_load_logger',
            '_load_simulator', 
            'step',
            'is_running',
            'loop',
            'reset',
            'close',
            'add_default_scene',
            'add_robot',
            'get_robot',
            'remove_robot',
            'add_object',
            'get_object', 
            'remove_object',
            'add_sensor',
            'get_sensor',
            'remove_sensor',
            'get_robot_state',
            'get_object_state',
            'get_sensor_state'
        }
        
        # Check that these methods are marked as abstract
        base_sim_abstract_methods = set(BaseSim.__abstractmethods__)
        assert abstract_methods.issubset(base_sim_abstract_methods)

    def test_concrete_implementation_required(self):
        """Test that concrete implementation must implement all abstract methods."""
        
        # Create a concrete class that doesn't implement all methods
        class IncompleteSimulator(BaseSim):
            def step(self):
                pass
            
            def is_running(self):
                return True
        
        # Should raise TypeError due to unimplemented abstract methods
        with pytest.raises(TypeError):
            IncompleteSimulator(PhysicsSimulatorConfig())

    def test_complete_concrete_implementation(self):
        """Test that a complete concrete implementation can be instantiated."""
        
        class CompleteSimulator(BaseSim):
            def __init__(self, config):
                super().__init__(config)
                
            def _load_logger(self, logger_config):
                return Mock()
                
            def _load_simulator(self, mujoco_config):
                return Mock()
                
            def step(self):
                pass
                
            def is_running(self):
                return True
                
            def loop(self):
                pass
                
            def reset(self):
                pass
                
            def close(self):
                pass
                
            def add_default_scene(self):
                pass
                
            def add_robot(self, robot_config):
                pass
                
            def get_robot(self, prim_path):
                return Mock()
                
            def remove_robot(self, prim_path):
                pass
                
            def add_object(self, object_config):
                pass
                
            def get_object(self, prim_path):
                return Mock()
                
            def remove_object(self, prim_path):
                pass
                
            def add_sensor(self, sensor_config):
                pass
                
            def get_sensor(self, prim_path):
                return Mock()
                
            def remove_sensor(self, prim_path):
                pass
                
            def get_robot_state(self, prim_path):
                return {}
                
            def get_object_state(self, prim_path):
                return {}
                
            def get_sensor_state(self, prim_path):
                return {}
        
        # Should be able to instantiate a complete implementation
        config = PhysicsSimulatorConfig()
        simulator = CompleteSimulator(config)
        assert isinstance(simulator, BaseSim)

    def test_initialization_signature(self):
        """Test that __init__ method has correct signature."""
        import inspect
        
        init_signature = inspect.signature(BaseSim.__init__)
        parameters = list(init_signature.parameters.keys())
        
        # Should have self and physics_simulator_config parameters
        assert len(parameters) == 2
        assert parameters[0] == 'self'
        assert parameters[1] == 'physics_simulator_config' 