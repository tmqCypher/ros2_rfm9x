from setuptools import setup

package_name = 'rfm9x_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Quinn Cypher',
    maintainer_email='tmqCypher.git@gmail.com',
    description='A ROS 2 node for the Adafruit RFM9x radio modules',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rfm9x_controller = rfm9x_controller.RFM9xController:main',
        ],
    },
)
