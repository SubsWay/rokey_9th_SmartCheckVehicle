from setuptools import find_packages, setup
from glob import glob

package_name = 'scv_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/best.pt']),
        ('share/' + package_name, ['README.md', 'rosdep.yaml']),
        ('share/' + package_name + '/docs', ['docs/shutter_obstacle.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'torch', 'ultralytics', 'Pillow>=9.2'],
    zip_safe=True,
    maintainer='doyoon-kim',
    maintainer_email='tnduaehowl37@gmail.com',
    description='로봇 PC 설비 검출 진단 노드 (InspectFacility 액션 연동 예정)',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'scv_detection_manager = scv_detection.scv_detection_manager:main',
            # 패키지 안의 test.py를 독립 명령 test로 등록한다.
            
        ],
    },
)
