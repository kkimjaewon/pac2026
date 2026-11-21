from setuptools import setup

package_name = 'b_mock'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeonju',
    maintainer_email='zxcasd1211@hanyang.ac.kr',
    description='B 계층 대역 노드 (C 개발용)',
    license='Apache-2.0',
    entry_points={'console_scripts': ['fake_b_node = b_mock.fake_b_node:main']},
)
