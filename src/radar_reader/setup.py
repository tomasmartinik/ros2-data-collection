from setuptools import setup

package_name = 'radar_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'radar_reader.basic',  # Přidání basic.py
        'radar_reader.radar_reader_node',
    ],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Radar reader node using Acconeer API',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_reader_node = radar_reader.radar_reader_node:main',
        ],
    },
)
