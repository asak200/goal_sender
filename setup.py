from setuptools import find_packages, setup

package_name = 'goal_sender'

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
    maintainer='asak',
    maintainer_email='abdoorahmansk2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_send_goal = goal_sender.send_client:main',
            'image_saver_node = map_drawer.save_map:main',
            'map_updater = map_drawer.map_feature_drawer:main',
            'qr_pose_pub = my_services.qr_pub_srv:main',
            'get_pose = my_services.posetion_request_srv:main',
            'serial_com1 = my_services.serial_com1:main',

        ],
    },
)
