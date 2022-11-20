from setuptools import setup
from glob import glob

package_name = 'final_v1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133a final_v1 Code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test1 = final_v1.test1:main',
            'marker = final_v1.marker:main',
            'test2 = final_v1.test2:main',
            'marker2 = final_v1.marker2:main',
        ],
    },
)
