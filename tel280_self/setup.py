from setuptools import find_packages, setup

package_name = 'tel280_self'

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
    maintainer='magnus',
    maintainer_email='magnus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "laserscan = tel280_self.laserscan_sub:main",
            "wall = tel280_self.wall_following:main"
        ],
    },
)
