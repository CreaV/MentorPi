from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mentorpi_supervisor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'foxglove_layout'), glob('foxglove_layout/*.json')),
        (os.path.join('share', package_name, 'web'),
            [f for f in glob('web/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'web', 'vendor'),
            glob('web/vendor/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='zhiheng.luo2@gmail.com',
    description='Mode-switching supervisor for MentorPi remote operation.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'supervisor_node = mentorpi_supervisor.supervisor_node:main',
        ],
    },
)
