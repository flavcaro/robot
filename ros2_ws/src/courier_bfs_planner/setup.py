from setuptools import setup

package_name = 'courier_bfs_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='BFS path planner for Courier Robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bfs_node = courier_bfs_planner.bfs_node:main',  # calls main() in bfs_node.py
        ],
    },
)
