from setuptools import find_packages, setup

package_name = 'mobile_manipulator_orchestrator'

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
        'State-machine orchestrator for the mobile manipulator '
        'warehouse demo. Implements HOME→NAV_TO_PICK→PERCEIVE→'
        'APPROACH_ARM→GRASP→NAV_TO_DROP→PLACE_ARM→RELEASE→'
        'RETURN_HOME with Nav2, MoveIt2, and gripper. Phase 8.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'warehouse_orchestrator = '
            'mobile_manipulator_orchestrator.warehouse_orchestrator:main',
        ],
    },
)
