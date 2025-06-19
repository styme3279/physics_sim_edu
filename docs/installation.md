# Installation

## Requirements

* Python 3.10+
* MuJoCo physics engine
* Compatible with Ubuntu 22.04, macOS

## Quick Installation

1. **Install the package:**

   ```bash
   git clone <repository-url>
   cd physics_simulator
   pip install -e .
   ```

2. **Verify installation:**

   ```python
   from physics_simulator import PhysicsSimulator
   from synthnova_config import PhysicsSimulatorConfig
   
   # Test basic import
   config = PhysicsSimulatorConfig()
   sim = PhysicsSimulator(config)
   print("Installation successful!")
   ```

## Getting Started

After installation, try the basic example:

```python
from physics_simulator import PhysicsSimulator
from synthnova_config import PhysicsSimulatorConfig

# Create and run basic simulation
config = PhysicsSimulatorConfig()
sim = PhysicsSimulator(config)
sim.add_default_scene()
sim.initialize()
sim.close()
```

## Next Steps

* See [Examples](examples.md) for usage examples
* Check [API](api.md) for complete API reference
* Read [Configuration](configuration.md) for configuration options 