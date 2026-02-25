from setuptools import setup

package_name = 'rqt_uav_qgc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource',
            ['resource/UavQgc.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='luenberger',
    maintainer='luenberger',
    maintainer_email='sszy@mail.nwpu.edu.cn',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'A Python GUI plugin for UAV QGC.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'rqt_uav_qgc = ' + package_name + '.uav_qgc:main',
        ],
    },
)
