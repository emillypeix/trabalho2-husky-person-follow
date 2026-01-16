from setuptools import setup

package_name = 'husky_person_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py', 'launch/demo_gz.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emilly',
    maintainer_email='emilly@email.com',
    description='Person following with lateral stop using FSM (mock + Gazebo)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_follow_fsm = husky_person_follow.state_machine:main',
            'person_detector = husky_person_follow.person_detector:main',
            'person_detector_gz = husky_person_follow.person_detector_gz:main',
        ],
    },
)
