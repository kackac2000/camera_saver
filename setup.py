from setuptools import setup

package_name = 'camera_saver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'opencv-python', 
    ],
    zip_safe=True,
    author='Kacper',
    author_email='your_email@example.com',  
    maintainer='kmagier',
    maintainer_email='your_email@example.com',  
    description='',
    license='Apache 2.0', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_saver_node = camera_saver.image_saver_node:main', 
        ],
    },
)