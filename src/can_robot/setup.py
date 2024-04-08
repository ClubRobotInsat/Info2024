from setuptools import find_packages, setup

package_name = 'can_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_rx = can_robot.can_rx:main',
            'can_raw_rx = can_robot.can_raw_rx:main',
            'can_raw_tx = can_robot.can_raw_tx:main',
            'can_tx = can_robot.can_tx:main'
        ],
    },
)
