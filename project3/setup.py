from setuptools import find_packages, setup

package_name = 'project3'

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
    maintainer='student',
    maintainer_email='derin.uyar@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'part1.py = project3.part1:main',
            'part2.py = project3.part2:main',
            'part3.py = project3.part3:main',
            'part4.py = project3.part4:main',
        ],
    },
)
