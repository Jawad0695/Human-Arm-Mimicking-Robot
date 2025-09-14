from setuptools import setup
import os
from glob import glob

package_name = 'robotic_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         [f for f in glob('launch/*') if '__pycache__' not in f and not f.endswith('.pyc')]),
        (os.path.join('share', package_name, 'urdf'),
         [f for f in glob('urdf/*') if not f.endswith('.pyc')]),
        (os.path.join('share', package_name, 'config'),
         [f for f in glob('config/*') if not f.endswith('.pyc')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gursel',
    maintainer_email='gursel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
entry_points={
    'console_scripts': [
        'arm_controller_from_yolo = robotic_arm.arm_controller_from_yolo:main',
        'hand_to_arm = robotic_arm.hand_to_arm:main',  # 👈 Add this line
    ],
},
)

