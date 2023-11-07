from setuptools import setup

package_name = 'launch_nodes'  # Replace with your actual package name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Description of your package',
    license='License of your package',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch.frontend.launch_extension:LaunchExtension'],
        'console_scripts': [
            'dummy_camera_info_node = your_package_name.dummy_camera_info_node:main',
            'test_img_publisher_node = your_package_name.test_img_publisher_node:main',
        ],
    },
)