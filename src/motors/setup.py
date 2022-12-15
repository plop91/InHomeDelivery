from setuptools import setup

package_name = 'motors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'lgpio', 'simple_pid', 'numpy'],
    zip_safe=True,
    maintainer='ian',
    maintainer_email='ian@sodersjerna.com',
    description='Motor driver for InHomeDelivery Robot',
    license='GNU3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors = motors.motors_node:main'
        ],
    },
)
