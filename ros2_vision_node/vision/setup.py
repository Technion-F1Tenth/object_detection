from setuptools import setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doof-wagon',
    maintainer_email='doof-wagon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "object_detection_node = object_detection.object_detection_node:main"
        ],
    },
    package_data={
        'object_detection': ['object_detection/*']  # Include all files in ./object_detection/object_detection
    }
)
