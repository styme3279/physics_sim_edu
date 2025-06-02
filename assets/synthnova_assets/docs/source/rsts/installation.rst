.. _Installation:

Installation
============

This guide provides step-by-step instructions for installing the SynthNova assets.

1. Clone the Repository:

   .. code-block:: bash

      # Clone the repository
      git clone ssh://git@git.galbot.com:6043/synth_nova/basic/synthnova_assets.git --depth 1
      cd synthnova_assets

2. Install SynthNova Assets

   .. code-block:: bash

      pip install .

3. Add sn_assets to Environment Path:

   .. code-block:: bash

      which assets
      echo "alias sn_assets=\"$(which sn_assets)\"" >> ~/.bashrc
      source ~/.bashrc

   .. note::

      If ``which sn_assets`` does not return a valid path, ensure that you are currently in the environment
      where ``synthnova_assets`` was installed. 
      
      For example, if you installed it using a different Python environment
      (e.g., via ``~/miniconda3/envs/your_env/bin/ -m pip install .`` from another virtual environment), you may need to locate the actual path manually, such as ``~/miniconda3/envs/your_env/bin/sn_assets``.