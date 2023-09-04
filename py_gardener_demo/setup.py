from setuptools import setup
from setuptools import find_packages

package_name = 'client_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
        ('share/' + package_name + '/resource', [
            'resource/tree.xml',
        ]),
        ('share/' + package_name + '/actions', [
            'actions/Forward.py',
            'actions/Backward.py',
            'actions/Turn.py',
            'actions/CheckObstacle.py'
        ]),
    ],
    install_requires=['setuptools', 'py_trees', 'py_trees_ros', 'std_msgs', 'geometry_msgs', 'sensor_msgs', 'py_gardener'],
    zip_safe=True,
    author='Óscar Martínez',
    author_email='oscar.robotics@tutanota.com',
    maintainer='Óscar Martínez',
    maintainer_email='oscar.robotics@tutanota.com',
    keywords=['ROS2', 'py_trees'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='An translator to achieve BT.cpp behavior with py_trees',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = client_demo.demo:demo_main',
        ],
    },
)
