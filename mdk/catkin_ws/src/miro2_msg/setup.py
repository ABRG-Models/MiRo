### SETUP PYTHON PACKAGES

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
	packages=['miro2_msg'],
	package_dir={'': ''}
)
setup(**setup_args)

