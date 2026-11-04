from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup(**generate_distutils_setup(packages=['community_perception_fall', 'community_perception_ocr', 'community_perception_yolo'], package_dir={'': 'src'}))
