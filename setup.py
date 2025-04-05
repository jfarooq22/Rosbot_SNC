from setuptools import setup

package_name = 'rosbot_search_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
        'launch/search_nav.launch.py',
        'launch/nav.launch.py'  
    ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Search and Navigation challenge for ROSBot Pro 3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = rosbot_search_nav.navigation_node:main',
            'hazard_detector_node = rosbot_search_nav.hazard_detector_node:main',
            'tracking_node = rosbot_search_nav.tracking_node:main',
        ],
    },
)
