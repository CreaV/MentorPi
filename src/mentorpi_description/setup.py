import os
from glob import glob
from setuptools import setup

package_name = 'mentorpi_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.*'))),
        (os.path.join('share', package_name, 'meshes/mecanum'), glob(os.path.join('meshes/mecanum', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CreaV',
    maintainer_email='zhiheng.luo2@gmail.com',
    description='URDF/xacro and meshes for the MentorPi mecanum robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
