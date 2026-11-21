from setuptools import find_packages, setup

package_name = 'ai_inference'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ai_inference/launch', ['launch/ai_inference.launch.py']),
        ('share/ai_inference/config', ['config/sensor_config.yaml']),
        ('share/ai_inference/models', ['models/plate_yolov8n_320.onnx']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nahyun',
    maintainer_email='nh9202@gmail.com',
    description='A layer: AI inference',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ai_inference_node = ai_inference.ai_inference_node:main',
        ],
    },
)
