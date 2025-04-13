from setuptools import find_packages, setup

package_name = 'lamp_package'
files = package_name + "/*.py"

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/lamp_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/lamp.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/lamp_robot.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    data_files=data_files,
    install_requires=['setuptools', "wheel",  "Cython"],
    zip_safe=True,
    maintainer='sofya',
    maintainer_email='sofya@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lamp_driver = lamp_package.lamp_driver:main'
        ],
    },
)
