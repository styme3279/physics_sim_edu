from pydantic import BaseModel, Field, ConfigDict
from typing import Optional, List, Dict, Any, Union

"""
Configuration classes for the physics simulator
"""


class BaseConfig(BaseModel):
    """Base configuration class with common settings"""
    model_config = ConfigDict(
        validate_assignment=True,  # Enable runtime validation of field assignments
        extra="forbid",  # Prevent accidental addition of undefined fields
        frozen=False,  # Allow modification of configuration after initialization
    )


class SimulationConfig(BaseConfig):
    """Configuration for simulation parameters"""
    timestep: float = Field(
        default=0.01,
        description="Simulation timestep in seconds"
    )
    substeps: int = Field(
        default=1,
        description="Number of simulation substeps per timestep"
    )


class RenderConfig(BaseConfig):
    """Configuration for rendering parameters"""
    enabled: bool = Field(
        default=True,
        description="Whether to enable rendering"
    )
    window_width: int = Field(
        default=1280,
        description="Width of the rendering window in pixels"
    )
    window_height: int = Field(
        default=720,
        description="Height of the rendering window in pixels"
    )
    

class PhysicsConfig(BaseConfig):
    """Configuration for physics parameters"""
    gravity: List[float] = Field(
        default=[0, 0, -9.81],
        description="Gravity vector [x, y, z] in m/s^2"
    )


class ConfigManager:
    """
    Manager for handling configuration objects
    
    This class provides methods for loading, validating, and 
    accessing configuration objects.
    """
    
    def __init__(self):
        self._configs = {}
        
    def register_config(self, name: str, config: BaseConfig):
        """
        Register a configuration object
        
        Args:
            name: Name of the configuration
            config: Configuration object
        """
        self._configs[name] = config
        
    def get_config(self, name: str) -> BaseConfig:
        """
        Get a configuration object by name
        
        Args:
            name: Name of the configuration
            
        Returns:
            The configuration object
            
        Raises:
            KeyError: If the configuration does not exist
        """
        if name not in self._configs:
            raise KeyError(f"Configuration '{name}' not found")
        return self._configs[name]
        
    def update_config(self, name: str, updates: Dict[str, Any]):
        """
        Update a configuration object
        
        Args:
            name: Name of the configuration
            updates: Dictionary of updates to apply
            
        Raises:
            KeyError: If the configuration does not exist
        """
        config = self.get_config(name)
        for key, value in updates.items():
            setattr(config, key, value) 