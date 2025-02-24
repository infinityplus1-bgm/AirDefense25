from setuptools import find_packages, setup

package_name = 'cvpy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teknofest',
    maintainer_email='teknofest@test.local',
    description='this is a testing package for ros2 integration with opencv',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = cvpy.image_publisher:main',
            'image_subscriber = cvpy.image_subscriber:main',
        ],
    },
)
    