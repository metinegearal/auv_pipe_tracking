from setuptools import find_packages, setup

package_name = 'pipe_track_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='metin-ege',
    maintainer_email='metinegearal@gmail.com',
    description='Perception module for AUV pipe tracking using deep learning U-Net segmentation.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'segmentation = pipe_track_perception.segmentation:main',
            'planning = pipe_track_perception.planning:main',
        ],
    },
)