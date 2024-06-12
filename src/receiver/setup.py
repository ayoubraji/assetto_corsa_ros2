from setuptools import setup

package_name = 'receiver'

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
    maintainer='Francesco Moretti',
    maintainer_email='cesco.moretti@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
                'receiver = receiver.recv:main',
                'receiver_tester = receiver.recv_tester:main'         
        ],
    },
)
