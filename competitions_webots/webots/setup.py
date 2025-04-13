from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'webots'
files = package_name + "/*.py"

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name+'/resource/meshes/arm165/manipulator/dae/', glob('resource/meshes/arm165/manipulator/dae/*')),
        ('share/' + package_name+'/resource/meshes/arm165/gripper/stl/', glob('resource/meshes/arm165/gripper/stl/*')),
        ('share/' + package_name+'/resource/meshes/arm165/gripper/dae/', glob('resource/meshes/arm165/gripper/dae/*')),
        ('share/' + package_name+'/resource/meshes/palletizer/dae/', glob('resource/meshes/palletizer/dae/*')),
        ('share/' + package_name+'/resource/meshes/palletizer/stl/', glob('resource/meshes/palletizer/stl/*')),
        ('share/' + package_name+'/resource/worlds', glob(os.path.join('resource/worlds', '*.wbt*'))),
        ('share/' + package_name+'/resource/protos/', glob(os.path.join('resource/protos/', '*.proto*'))),
        ('share/' + package_name+'/resource/assets/', glob('resource/assets/*')),
        ('share/' + package_name+'/resource/urdf/', glob('resource/urdf/*'))
    ],
    install_requires=['setuptools', "wheel",  "Cython"],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='vinokurov1768@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'conveyor_driver = webots.conveyor_driver:main',
            'lamp_driver = webots.lamp_driver:main',
            'button_driver = webots.button_driver:main',
            'controller_driver = webots.controller_driver:main',
            'simulation_supervisor = webots.simulation_supervisor:main',
        ],
    },
)
