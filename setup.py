from setuptools import setup

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), ('share/' + package_name, ['camera_calibration.yml'])
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
            'img_subscriber = vision.webcam_sub:main'
        ],
    },
)
