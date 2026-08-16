from setuptools import find_packages, setup

package_name = 'mobile_manipulator_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ship the detector weights with the package so `ros2 run` never
        # depends on Ultralytics reaching the network mid-demo, and so the
        # phase gate is reproducible offline.
        ('share/' + package_name + '/models', ['models/yolov8n.pt']),
    ],
    install_requires=['setuptools'],
    # tests_require is here ONLY to make colcon collect the tests, and is NOT
    # how this package declares its test dependencies — those are <test_depend>
    # entries in package.xml, which is what rosdep and the build farm read.
    # Without this line colcon falls back to `setup.py test` (unittest), which
    # collects nothing from test/ and reports "Ran 0 tests ... OK": a green
    # `colcon test` that ran no tests at all.
    tests_require=['pytest'],

    zip_safe=True,
    maintainer='zsh',
    maintainer_email='zain.alabidin.shbani@gmail.com',
    description=(
        'YOLOv8 perception node for the mobile manipulator. '
        'Subscribes to the RealSense D435i RGB+depth streams, runs '
        'Ultralytics YOLOv8 inference on synchronised frame pairs, and '
        'broadcasts camera_color_optical_frame -> object_target_frame for '
        'the orchestrator.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_perception_node ='
            ' mobile_manipulator_perception.yolo_perception_node:main',
            'phase7_look_pose ='
            ' mobile_manipulator_perception.phase7_look_pose:main',
            'phase7_target_check ='
            ' mobile_manipulator_perception.phase7_target_check:main',
        ],
    },
)
