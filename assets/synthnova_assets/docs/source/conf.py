# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'SynthNova Assets'
copyright = '2023-2025, Galbot Corporation.'
author = 'Herman Ye, Junjie Jia'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

import sys
import os
from pathlib import Path
import site

# Add conda environment paths
conda_prefix = os.environ.get('CONDA_PREFIX')
if conda_prefix:
    # Add conda environment's site-packages
    site_packages = os.path.join(conda_prefix, 'lib', 'python' + '.'.join(map(str, sys.version_info[:2])), 'site-packages')
    if os.path.exists(site_packages):
        sys.path.insert(0, site_packages)
    
    # Add conda environment's lib directory
    lib_path = os.path.join(conda_prefix, 'lib')
    if os.path.exists(lib_path):
        sys.path.insert(0, lib_path)

# Add parent directory to sys.path for synthnova_assets
parent_path = str((Path(__file__).parent.parent.parent.parent).resolve())
sys.path.insert(0, parent_path)

# Add src directory to sys.path
src_path = str((Path(__file__).parent.parent.parent / "src").resolve())
sys.path.insert(0, src_path)
extensions = [
    'sphinx.ext.githubpages',  
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.todo',
    'sphinx_autodoc_typehints',


    "sphinx.ext.githubpages",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.todo",
    # "sphinx.ext.viewcode",
    "sphinx_autodoc_typehints",
    # "sphinx.ext.autosummary",
    # 'breathe',
]
autodoc_typehints = "signature"

templates_path = ["_templates"]
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# html_theme = 'alabaster'
html_theme = "sphinxawesome_theme"
pygments_style = "default"
# Select a different theme for dark mode
pygments_style_dark = "github-dark"
html_static_path = ['_static']

html_theme_options = {
    "logo_light": "_static/synthnova_logo.svg",
    "logo_dark": "_static/synthnova_logo.svg",
    "show_prev_next": True,
}

html_favicon = "_static/synthnova_logo.svg"
html_show_sphinx = False