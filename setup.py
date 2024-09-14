from setuptools import find_packages, setup

package_name = 'my_web_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        '': ['templates/*.html', 'static/css/*.css', 'static/js/*.js'],
    },
    install_requires=['setuptools','python3-flask'],
    zip_safe=True,
    maintainer='corvus',
    maintainer_email='wagneraron@outlook.de',
    description='ROS2 Node hosting a Flask website',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_node = my_web_package.web_node:main'
        ],
    },
)
