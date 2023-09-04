from setuptools import setup
from setuptools import find_packages

package_name = 'py_gardener'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/py_gardener', [
            'py_gardener/gn_parser.py',
            'py_gardener/gn_factory.py',
            'py_gardener/gn_tools.py'
        ]),
    ],
    install_requires=['setuptools', 'py_trees', 'py_trees_ros', 'std_msgs', 'geometry_msgs', 'sensor_msgs'],
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
)
