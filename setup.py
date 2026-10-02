import os
from glob import glob
from setuptools import setup

package_name = 'ur3_llm_control'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vũ Văn Hiệp',
    maintainer_email='230200742@vnu.edu.vn',
    description='UR3 LLM Control with Gripper and Overhead Vision',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'skill_executor = ur3_llm_control.skill_executor:main',
            'scene_publisher = ur3_llm_control.scene_publisher:main',
            'object_detector = ur3_llm_control.object_detector:main',
        ],
    },
)
