from setuptools import setup

package_name = 'py_srvcli'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for Python-based service, client, and subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plate_server = py_srvcli.plate_server:main',
            'plate_subscriber = py_srvcli.plate_subscriber:main',
            'plate_client = py_srvcli.plate_client:main',
        ],
    },
)
