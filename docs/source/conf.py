# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'SynthNova Physics Simulator Edu'
copyright = '2023-2025, Galbot Corporation.'
author = 'Chenyu Cao'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration


extensions = [
    'sphinx.ext.githubpages',  
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx_autodoc_typehints',
    
    # 'breathe',
]
autodoc_typehints = 'signature'

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output


html_theme = "sphinxawesome_theme"
# html_theme = 'alabaster'
# Select theme for both light and dark mode
pygments_style = "default"
# Select a different theme for dark mode
pygments_style_dark = "github-dark"

html_static_path = ['_static']

html_theme_options = {
    "logo_light": "_static/galbot_web_logo.jpg",
    "logo_dark": "_static/galbot_web_logo_for_night.jpg",
    "show_prev_next": True,
}

html_favicon = '_static/galbot_web_logo.jpg'
html_show_sphinx = False
autodoc_member_order = 'bysource'

# import sys
# from pathlib import Path
# sys.path.insert(0, str(Path(__file__).parent.parent.absolute()))


import os
import sys
sys.path.insert(0, os.path.abspath('../../src/physics_simulator/simulator/'))