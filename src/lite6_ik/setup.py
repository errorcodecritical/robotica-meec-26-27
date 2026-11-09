from setuptools import find_packages, setup

package_name = 'lite6_ik'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/lw2_targets.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cazevedo',
    maintainer_email='cazevedo@ipn.pt',
    description='LW2: algebraic inverse kinematics of the UFACTORY Lite 6 (Robotics 02000537).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_node = lite6_ik.inverse_kinematics:main',
            'ik_check = lite6_ik.ik_check:main',
            'ik_study = lite6_ik.ik_study:main',
        ],
    },
)
