from setuptools import find_packages, setup

package_name = 'explorer_navigator'

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
    maintainer='Ricardo Berumen',
    maintainer_email='A01641180@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator_to_goal = explorer_navigator.navigator_to_goal:main'
        ],
    },
)
