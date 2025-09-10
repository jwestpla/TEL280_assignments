from setuptools import find_packages, setup

package_name = 'assignment1_pkg'

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
    maintainer='TEL280 gruppe 3',
    maintainer_email='daimen2210@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "laserscan_subscriber = assignment1_pkg.laserscan_subscriber:main",
        ],
    },
)
