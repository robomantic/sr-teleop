from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['sr_remappers'],
    scripts=['scripts/js_to_jtp.py'],
    package_dir={'':'src'}
)

setup(**d)
