from setuptools import setup

package_name = 'robotiq_urcap_control'

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
    maintainer='rosi',
    maintainer_email='khairilaiman1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmodel_urcap_driver = robotiq_urcap_control.cmodel_urcap_driver:main',
            'robotiq_example_client = robotiq_urcap_control.robotiq_example_client:main'
        ],
    },
)
