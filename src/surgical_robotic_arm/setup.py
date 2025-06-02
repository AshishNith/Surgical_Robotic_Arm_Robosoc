from setuptools import setup

package_name = 'surgical_robotic_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/robotic_arm.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mr Ranjan',
    maintainer_email='you@example.com',
    description='Simulated robotic arm with IK',
    license='MIT',
    entry_points={
        'console_scripts': [
            'inverse_kinematics = surgical_robotic_arm.inverse_kinematics:main'
        ],
    },
)
