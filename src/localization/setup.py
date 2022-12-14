import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools', 'shapely', 'numpy'],
    zip_safe=True,
    maintainer='ian',
    maintainer_email='ian@sodersjerna.com',
    description='Localization package for InHomeDelivery Robot',
    license='GNU3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pf = localization.particle_filter_node:main',
            'state_publisher = localization.state_publisher:main'
        ],
    },
)
