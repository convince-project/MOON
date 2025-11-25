from setuptools import find_packages, setup

package_name = 'simexec'

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
    maintainer='k',
    maintainer_email='karim.pedemonte@edu.unige.it',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'exec = {package_name}.exec:main',
            f'hold = {package_name}.hold:main',
            f'move_drop = {package_name}.move_drop:main',
            f'pick_drop = {package_name}.pick_drop:main',
        ],
    },
)
