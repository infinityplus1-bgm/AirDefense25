from setuptools import find_packages, setup

package_name = 'main'

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
    maintainer='infinityplusone',
    maintainer_email='infinityplusone@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera = main.camera:main",
            "camera_test = main.camera_test:main"
<<<<<<< HEAD
=======
            "yolo_detection = main.detection.yolo_detection:main",
>>>>>>> b352603 (yolo_detection node need to check by anas)
        ],
    },
)
