from setuptools import find_packages, setup

package_name = 'my_web_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/my_web_package/launch', ['launch/wgg.launch.py','launch/n10.launch.py','launch/test.launch.py']),
    ],
    package_data={
        '': ['templates/*.html', 'static/css/*.css', 'static/js/*.js','static/images/*.png'],
    },
    install_requires=['setuptools','python3-flask','setuptools','launch'],
    zip_safe=True,
    maintainer='corvus',
    maintainer_email='wagneraron@outlook.de',
    description='ROS2 Node hosting a Flask website',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_node = my_web_package.web_app:main',
            'service_test = my_web_package.service_test:main',
            'service_bridge = my_web_package.service_bridge:main'
        ],
    },
)
