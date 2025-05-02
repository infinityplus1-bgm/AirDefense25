import os
from glob import glob
from setuptools import setup

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juka',
    maintainer_email='jamgad23@gmail.com',
    description='Air defense simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
		'spawn_balloons = simulation.spawn_multiple_balloons:main',
        	'spawn_camera = simulation.camera_subscriber:main',
        	'balloon_mover = simulation.balloon_mover:main',
        ],
    },
)   
