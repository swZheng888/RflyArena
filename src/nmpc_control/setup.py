from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['nmpc_control', 'nmpc_control.utils','quadrotor_msgs'],
    package_dir={'': 'src'}
)

setup(**d)