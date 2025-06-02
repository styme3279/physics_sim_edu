pip install sphinx --upgrade
pip install sphinx-autodoc-typehints
pip install sphinxawesome_theme
make clean
make html
xdg-open build/html/index.html