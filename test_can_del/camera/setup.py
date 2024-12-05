from setuptools import find_packages, setup

package_name = 'camera'

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
    maintainer='iclab',
    maintainer_email='iclab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = camera.camera:main',
            'H65_camera = camera.H65_camera:main',
            'take_image = camera.take_image:main',
            'image_pub = camera.image_pub:main',
            'video_pub = camera.video_pub:main',
            'camera_compress = camera.camera_compress:main',
        ],
    },
)
