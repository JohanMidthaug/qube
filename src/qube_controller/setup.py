from setuptools import find_packages, setup

package_name = 'qube_controller'

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
    maintainer='chrirsk',
    maintainer_email='chrirsk@stud.ntnu.no',
    description='Qube_PID_Controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qube_controller_node = qube_controller.qube_controller_node:main',
        ],
    },
)
