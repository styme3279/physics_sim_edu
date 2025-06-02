pip install sphinx --upgrade
pip install sphinx-autodoc-typehints
pip install sphinxawesome_theme
make clean
make html

# Choose the correct open command based on operating system
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS system
    open build/html/index.html
else
    # Linux system
    xdg-open build/html/index.html || echo "Unable to open browser, please manually open build/html/index.html"
fi