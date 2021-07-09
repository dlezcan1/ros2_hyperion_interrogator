from setuptools import setup
import glob

package_name = 'hyperion_interrogator'
submodules = '{}/hyperionlib'.format(package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dimitri Lezcano',#
    maintainer_email='dlezcan1@jhu.edu',
    description='Hyperion FBG Interrogator Publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hyperion_talker = hyperion_interrogator.hyperion_talker:main',
            'hyperion_streamer = hyperion_interrogator.peak_streamer:main',
            'reconnect = hyperion_interrogator.reconnect_client:main',
            'calibrate_sensors = hyperion_interrogator.sensor_calibrate_client:main'
        ],
    },
)
