from setuptools import find_packages, setup
from glob import glob

package_name = 'manipulators_control'
submodules = 'manipulators_control/pymoveit2'
files = package_name + "/*.py"

setup(
    # ext_modules=cythonize(files,compiler_directives={'language_level' : "3"},force=True,quiet=True),
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name + '/', glob('manipulators_control/config_csv/*'))
    ],
    # install_requires=['setuptools', "wheel",  "Cython"],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='vinokurov1768@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'angle_robot_control = manipulators_control.angle_robot_control:main',
            'palletizer_robot_control = manipulators_control.palletizer_robot_control:main',
            'test_node = manipulators_control.test_node:main',
            'palletizer_test = manipulators_control.palletizer_test:main',
            'angle_test = manipulators_control.angle_test:main',
            'two_manip_test = manipulators_control.two_manip_test:main',
            'multirobot_test = manipulators_control.multirobot_test:main',
            'technosphere_test = manipulators_control.technosphere_test:main',
            'udp_recv_send = manipulators_control.udp_recv_send:main',
        ],
    },
)
