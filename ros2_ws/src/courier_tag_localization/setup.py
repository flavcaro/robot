from setuptools import setup

package_name = 'courier_tag_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='AprilTag localization for Courier Robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tag_localization_node = courier_tag_localization.tag_localization_node:main',
        ],
    },
)
