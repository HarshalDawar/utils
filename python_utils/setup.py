from setuptools import find_packages, setup

package_name = 'python_utils'

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
    maintainer='harshal',
    maintainer_email='harshaldawar@gmail.com',
    description='General Python based utilities',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'topic_rate_prober = python_utils.topic_rate_prober:main',
            'random_pubs = python_utils.random_pubs:main'
        ],
    },
)
