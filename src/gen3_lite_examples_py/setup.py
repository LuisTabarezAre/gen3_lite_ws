from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gen3_lite_examples_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='luis.tabarez@alumnos.udg.mx',
    description='Gen3 Lite examples pkg (FK, IK, DK)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "forward_kinematics=gen3_lite_examples_py.fk_node:main",
            "differential_kinematics=gen3_lite_examples_py.dk_node:main"
        ],
    },
)
