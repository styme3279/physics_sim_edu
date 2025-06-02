.. _Installation:

Installation
============

Requirements
------------

* Python 3.10+
* MuJoCo physics engine
* Compatible with Ubuntu 22.04, macOS

Quick Installation
------------------

1. **Install the package:**

   .. code-block:: bash

      git clone https://github.com/galbot-ioai/physics_sim_edu.git
      cd physics_sim_edu
      pip install -e ./src/synthnova_config
      pip install -e .

2. **Verify installation:**

   .. code-block:: python

      from physics_simulator import PhysicsSimulator
      from synthnova_config import PhysicsSimulatorConfig
      
      # Test basic import
      config = PhysicsSimulatorConfig()
      sim = PhysicsSimulator(config)
      print("Installation successful!")

Getting Started
---------------

After installation, try the basic example:

.. code-block:: python

   from physics_simulator import PhysicsSimulator
   from synthnova_config import PhysicsSimulatorConfig
   
   # Create and run basic simulation
   config = PhysicsSimulatorConfig()
   sim = PhysicsSimulator(config)
   sim.add_default_scene()
   sim.initialize()
   sim.close()

Next Steps
----------

* See :ref:`Examples` for usage examples
* Check :ref:`API` for complete API reference
* Read :ref:`Configuration` for configuration options
