from setuptools import find_packages, setup

package_name = 'ui'

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
    maintainer='infinityplus1',
    maintainer_email='infinityplus1.teknofest@gmail.com',
    description='UI interface for our teams airdefense project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui = ui.image_subscriber:main',
            'publisher = ui.image_publisher:main',
        ],
    },
)
