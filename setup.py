from setuptools import setup

package_name = 'particle_bicycle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='memristor',
    maintainer_email='kpetrykin@gmail.com',
    description='Estimating position of a Robot Car (Bicycle Model) with a particle filter',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'particle_bicycle = particle_bicycle.particle_bicycle:main'
        ],
    },
)
