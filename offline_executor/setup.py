from setuptools import setup
from setuptools import find_packages

package_name = 'offline_executor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', [
            'resource/basic_tree.xml',
        ]),
        ('share/' + package_name + '/actions', [
            'actions/Forward.py',
            'actions/Turn.py',
            'actions/CheckObstacle.py'
        ]),
    ],
    install_requires=['setuptools', 'py_trees', 'py_trees_ros', 'std_msgs', 'geometry_msgs', 'sensor_msgs', 'tree_translator'],
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
    description='Offline execution example',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = offline_executor.execute:execute_main',
        ],
    },
)
