from setuptools import setup
import os
from glob import glob

package_name = 'simulator_output'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francesco Moretti',
    maintainer_email='cesco.moretti@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
                'simulator_output_script = simulator_output.simulator_output_script:main',
                'data_visualizator = simulator_output.data_visualizator:main',
                'simulator_output_IAC = simulator_output.simulator_output_IAC:main'
        ],
    },
)
