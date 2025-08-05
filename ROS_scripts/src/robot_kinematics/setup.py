from setuptools import find_packages, setup

package_name = 'robot_kinematics'

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
    maintainer='echelon',
    maintainer_email='echelon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_publisher = robot_kinematics.ik_publisher:main',
            'pose_gui_node = robot_kinematics.pose_gui_node:main'
        ],
    },
)
