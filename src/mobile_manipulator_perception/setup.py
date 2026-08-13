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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zsh',
    maintainer_email='zain.alabidin.shbani@gmail.com',
    description=(
        'YOLOv8 perception node for the mobile manipulator. '
        'Subscribes to RealSense D435i RGB+depth streams, runs '
        'Ultralytics YOLOv8 inference, and broadcasts '
        'object_target_frame TF. Populated in Phase 7.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Phase 7 will register:
            # 'perception_node = mobile_manipulator_perception.perception_node:main',
        ],
    },
)
