from setuptools import setup

package_name = 'nxp_cv_cam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Benjamin Perseghetti',
    author_email='bperseghetti@rudislabs.com',
    maintainer='Benjamin Perseghetti',
    maintainer_email='bperseghetti@rudislabs.com',
    description='ROS2 node for publishing OpenCV captured frames on IMX8.',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nxp_cv_cam_node = nxp_cv_cam.nxp_cv_cam_node:main'
        ],
    },
)
