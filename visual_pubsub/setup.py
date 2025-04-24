from setuptools import find_packages, setup

package_name = 'visual_pubsub'

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
    maintainer='andres',
    maintainer_email='andres@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse_kinematics = visual_pubsub.inverse_kinematics:main',
            'direct_kinematics = visual_pubsub.direct_kinematics:main',
        ],
    },
)
