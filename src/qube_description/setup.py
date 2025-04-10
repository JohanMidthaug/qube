from setuptools import find_packages, setup

package_name = 'qube_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/qube_config.rviz']),
        ('share/' + package_name + '/launch', ['launch/view_qube.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/qube.urdf.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/qube.macro.xacro']),
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
        ],
    },
)
