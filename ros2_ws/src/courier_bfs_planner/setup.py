from setuptools import find_packages, setup

package_name = 'courier_bfs_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='BFS path planner for Courier Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bfs_node = courier_bfs_planner.bfs_node:main',
        ],
    },
)
