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
            "camera = main.camera.camera:main",
            "camera_test = main.camera.camera_test:main",
            "yolo_detection = main.detection.yolo_detection:main",
            "detections_overlay_test = main.detection.detections_overlay_test:main"
        ],
    },
)
