from setuptools import setup
import os
from glob import glob

package_name = 'hawkeye_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='carbon225@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gods_hand = hawkeye_sim.gods_hand:main',
            'gods_eye = hawkeye_sim.gods_eye:main',
            'logical_camera = hawkeye_sim.logical_camera:main',
        ],
    },
)
