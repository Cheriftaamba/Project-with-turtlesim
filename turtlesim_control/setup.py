from setuptools import find_packages, setup

package_name = 'turtlesim_controler'

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
    maintainer='sherif',
    maintainer_email='sherif@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_square_controller = turtlesim_control.turtle_square_controller:main';
            'turtle_square_controller_1 = turtlesim_control.turtle_square_controller_1:main'
        ],
    },
)
