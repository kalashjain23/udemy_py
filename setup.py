from setuptools import setup

package_name = 'udemy_py'

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
    maintainer='kalash',
    maintainer_email='kalashjain513@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_python = udemy_py.basic_python_node:main',
            'robot_news_station = udemy_py.robot_news_station:main',
            'smartphone = udemy_py.smartphone:main',
            'number_publisher = udemy_py.number_publisher:main',
            'number_counter = udemy_py.number_counter:main',
            'add_two_ints_server = udemy_py.add_two_ints_server:main',
            'add_two_ints_client = udemy_py.add_two_ints_client:main'
        ],
    },
)
