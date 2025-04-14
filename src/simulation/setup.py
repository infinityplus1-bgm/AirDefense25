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
    maintainer_email='your_email@example.com',
    description='Air defense simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
		'spawn_multiple_balloons = simulation.spawn_multiple_balloons:main'
        ],
    },
)

