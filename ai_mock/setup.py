from setuptools import setup

package_name = 'ai_mock'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nahyeon',
    maintainer_email='nh9202@gmail.com',
    description='AI 계층 대역 노드',
    license='Apache-2.0',
    entry_points={'console_scripts': ['fake_ai_node = ai_mock.fake_ai_node:main']},
)
