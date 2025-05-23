from setuptools import find_packages, setup

package_name = 'trajectory_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + "sonoma_waypoints.txt"]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/sonoma_waypoints.txt']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonny',
    maintainer_email='jonnymoreira03@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = trajectory_control.pure_pursuit:main',
            'waypoint_publisher = trajectory_control.waypoint_publisher:main',
        ],
    },
)
