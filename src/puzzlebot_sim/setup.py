from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
         (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
         (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
         (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
         (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='israamaciaas',
    maintainer_email='a01027029@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'puzzlebot_sim_node = puzzlebot_sim.puzzlebot_sim_node:main',
            'joint_pub = puzzlebot_sim.joint_pub:main'
        ],
    },
)
