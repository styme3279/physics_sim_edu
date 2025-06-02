.. Physics Simulator Documentation master file, created by
   sphinx-quickstart on Fri Nov 15 11:30:06 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

SynthNova Physics Simulator Edu
=====================================

Physics Simulator is a MuJoCo-based simulation package that provides accurate physics simulations, high-level robot control interfaces, and comprehensive sensor systems for educational purposes.

**Key Features:**

* MuJoCo physics engine integration
* High-level GalbotInterface for modular robot control
* RGB/depth cameras and sensor simulation
* Support for robots, objects, and custom physics callbacks

**Quick Start:**

.. code-block:: python

   from physics_simulator import PhysicsSimulator
   from synthnova_config import PhysicsSimulatorConfig
   
   # Create and initialize simulator
   config = PhysicsSimulatorConfig()
   sim = PhysicsSimulator(config)
   sim.add_default_scene()
   sim.initialize()
   
   # Run simulation
   sim.loop()

.. toctree::
   :maxdepth: 2
   :caption: Documentation
   :hidden:

   📄 Overview <rsts/overview>
   🛠️ Installation <rsts/installation>
   ⚙️ Configuration <rsts/configuration>
   💡 Examples <rsts/examples>
   🖥️ API Documentation <rsts/api>
   🔧 Troubleshooting <rsts/troubleshooting>
   📜 License <rsts/license>
