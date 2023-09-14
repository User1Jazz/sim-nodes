from setuptools import setup

package_name = 'sim_nodes'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kristijan Segulja',
    maintainer_email='40493610@live.napier.ac.uk',
    description='Package containing the nodes that can receive and send data to the simulator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning = sim_nodes.path_planning:main'
        ],
    },
)
