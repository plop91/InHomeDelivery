from setuptools import setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'inputs'],
    zip_safe=True,
    maintainer='ian',
    maintainer_email='ian@sodersjerna.com',
    description='Controller for InHomeDelivery Robot',
    license='GNU3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = controller.controller_node:main'
        ],
    },
)
