from setuptools import setup

package_name = 'task_orchestrator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/task_orchestrator.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='purushottam',
    maintainer_email='purushottam@example.com',
    description='Task Orchestrator node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'task_orchestrator_node = task_orchestrator.task_orchestrator_node:main',
        ],
    },
)
