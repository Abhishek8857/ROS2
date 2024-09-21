from setuptools import setup, find_packages

package_name = 'follower'

data_files = []

data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/launch', ['launch/maze_world.launch.py']))
data_files.append(('share/' + package_name + '/world', ['world/maze_world.world']))
data_files.append(('share/' + package_name + '/follower', ['follower/camera_feed.py',
                                                           'follower/detection.py',
                                                           'follower/follow.py', 
                                                           'follower/controller.py']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhishek',
    maintainer_email='abhishek@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_feed = follower.camera_feed:main',
            'detection = follower.detection:main',
            'controller = follower.controller:main', 
            'follow = follower.follow:main'
        ],
    },
)
