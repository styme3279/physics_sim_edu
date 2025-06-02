Examples
========

This is a simple example of how to download assets using **SynthNova Assets**:

.. code-block:: bash

   sn_assets download -u <url1>,<url2>,... -o /path/to/save

To update (force re-download and overwrite existing assets):

.. code-block:: bash

   sn_assets update -u <url1>,<url2>,... -o /path/to/save

How **download** and **update** differ
--------------------------------------

Both commands fetch assets from the SynthNova cloud platform, but their behaviors differ when local files already exist:

1. **download**
   
   - Checks if the target zip file or extracted contents already exist.
   - If they do, the operation is skipped to avoid overwriting.
   - Useful when you want to avoid duplicate downloads and preserve local modifications.

2. **update**
   
   - Always downloads the asset from the cloud, even if a local version exists.
   - Overwrites existing files and folders with the latest versions.
   - Suitable for syncing your local data with the most up-to-date remote assets.


Download Example
-----------------

This example demonstrates how to use a Python program to download assets from the Digital Asset Cloud Platform.

.. literalinclude:: ../../../examples/download.py
   :language: python
   :caption: download.py


Update Example
-----------------

This example demonstrates how to use a Python program to update assets from the Digital Asset Cloud Platform.

.. literalinclude:: ../../../examples/download.py
   :language: python
   :caption: download.py
