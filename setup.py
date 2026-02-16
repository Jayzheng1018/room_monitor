from setuptools import setup, find_packages

package_name = 'room_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/web/templates', ['room_monitor/web/templates/index.html']),
        ('share/' + package_name + '/web/static', [
            'room_monitor/web/static/app.js',
            'room_monitor/web/static/styles.css',
        ]),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'jinja2',
        'numpy',
        'opencv-python',
        'Pillow',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='room_monitor maintainer',
    maintainer_email='user@example.com',
    description='Web-based room monitoring/patrol node for Kobuki',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_master = room_monitor.robot_master:main',
        ],
    },
)
