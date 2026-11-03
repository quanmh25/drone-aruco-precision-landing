from setuptools import find_packages, setup

package_name = 'landing_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', ['launch/run.launch.py']),
        ('share/' + package_name + '/config', ['config/ground_truth.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quanmh25',
    maintainer_email='maihoangquan250205@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_node = landing_control.mission_node:main',
        ],
    },
)
