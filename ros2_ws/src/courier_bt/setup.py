from setuptools import setup

package_name = 'courier_bt'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'py_trees'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Behavior Tree for Courier Robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'courier_bt_node = courier_bt.courier_bt_node:main',  # your Python main function
        ],
    },
)
