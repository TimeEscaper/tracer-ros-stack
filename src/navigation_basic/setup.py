from setuptools import setup

package_name = 'navigation_basic'
nodes_subpackage = 'navigation_basic/nodes'
controllers_subpackage = 'navigation_basic/controllers'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, nodes_subpackage, controllers_subpackage],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Timur Akhtyamov',
    maintainer_email='rentgeny05@gmail.com',
    description='Nodes for robot navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_goal_reaching_mpc  = navigation_basic.nodes.simple_goal_reaching_mpc:main'
        ],
    },
)
