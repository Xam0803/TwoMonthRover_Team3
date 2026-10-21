from setuptools import find_packages, setup

package_name = 'rover_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover3',
    maintainer_email='rover3@todo.todo',
    description='Axiom Rover Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'orangepi_node = rover_pkg.rover_onboard_node:main'
        ],
    },
)
