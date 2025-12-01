from setuptools import setup

package_name = 'ur3e_robotiq_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruchit',
    maintainer_email='ruchit9898@gmail.com',
    description='UR3e + Robotiq pick-and-place demo (Python)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place_demo = ur3e_robotiq_pick_place.pick_place:main',
        ],
    },
)
