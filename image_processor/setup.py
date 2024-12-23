from setuptools import setup

package_name = 'image_processor'

setup(
    name=package_name,
    version='3.8.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mathias Jensen',
    maintainer_email='matje21@student.sdu.dk',
    description='A package for processing images in ROS 2.',
    license='mp4d',
    entry_points={
        'console_scripts': [
            'image_processor_node = image_processor.image_processor_node:main',
        ],
    },
)

