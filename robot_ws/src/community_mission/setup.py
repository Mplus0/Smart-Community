from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup(**generate_distutils_setup(packages=['community_mission_runtime', 'community_mission_speech', 'community_mission_video'], package_dir={'': 'src'}))
