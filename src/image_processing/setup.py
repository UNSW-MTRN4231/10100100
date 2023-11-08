from setuptools import find_packages, setup

package_name = 'image_processing'

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
    maintainer='mtrn',
    maintainer_email='z5259702@ad.unsw.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smile_detector_node = image_processing.smile_detector_node:main',
            'contour_detection_node = image_processing.contour_detection_node:main',
            'lines = image_processing.lines:main',
            'test_img_publisher = image_processing.test_img_publisher:main',
            'dummy_camera_info = image_processing.dummy_camera_info:main',
            'test_pub_robot_action = image_processing.test_pub_robot_action:main',
            'dummy_smile_pub = image_processing.dummy_smile_pub:main'
        ],
    },
)
