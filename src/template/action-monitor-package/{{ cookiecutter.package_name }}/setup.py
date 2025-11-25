from setuptools import setup

package_name = '{{ cookiecutter.package_name }}'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='{{ cookiecutter.maintainer_name }}',
    maintainer_email='{{ cookiecutter.maintainer_email }}',
    description='{{ cookiecutter.description }}',
    license='{{ cookiecutter.license }}',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '{{ cookiecutter.node_name }} = {{ cookiecutter.package_name }}.{{ cookiecutter.node_name }}:main',
        ],
    },
)
