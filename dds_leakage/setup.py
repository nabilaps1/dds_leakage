from setuptools import find_packages, setup

package_name = 'dds_leakage'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nabila',
    maintainer_email='depaulanabila@gmail.com',
    description='Trajectory inference from cmd_vel and scan observed over DDS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = dds_leakage.controller:main',
            'observer = dds_leakage.observer:main',
            'ground_truth_recorder = dds_leakage.ground_truth_recorder:main',
        ],
    },
)
