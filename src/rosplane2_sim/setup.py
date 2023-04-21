import os # Operating system library
from glob import glob # Handles file path names
from setuptools import setup # Facilitates the building of packages

package_name = 'ros2plane_sim'

# Path of the current directory
cur_directory_path = os.path.abspath(os.path.dirname(__file__))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Path to the launch file
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),

        # Path to the world file
        (os.path.join('share', package_name,'worlds/'), glob('./worlds/*')),


        # Path to the world file (i.e. warehouse + global environment)
        (os.path.join('share', package_name,'rosplane2_sim/'), glob('.py*')),

        # Path to the fixwing.xml
        (os.path.join('share', package_name,'launch/'), glob('launch/*launch.[pyx][yma]*')),
         ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blayer',
    maintainer_email='blayer@byu.edi',
    description='trying to get the sim to run',
    license='na',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

