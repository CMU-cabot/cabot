import os
from glob import glob
from setuptools import setup

package_name = 'odriver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cabot',
    maintainer_email='k940805k.lab@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odriver_node = odriver.odriver_node:main'
        ],
    },
)
