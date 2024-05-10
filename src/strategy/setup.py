from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'strategy'
# submodule = 'strategy/homologation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('lib/' + package_name, ['strategy/homologation.py']),
       ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'strategy = strategy.strategy:main',
            'homologation = strategy.homologation:main',
            'test_rotation = strategy.test_rotation:main',
            'match_1_bleu = strategy.match_1_bleu:main',
            'match_1_jaune= strategy.match_1_jaune:main'
        ],
    },
)
