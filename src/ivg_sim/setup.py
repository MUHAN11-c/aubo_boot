import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ivg_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # 世界与 GT manifest 安装到 share（generate_table_world 生成后重建生效）
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf') + glob('worlds/*.yaml')),
        # 四指夹爪剖分网格（scripts/split_gripper2.py 产物）+ 包装 xacro
        ('share/' + package_name + '/meshes/visual',
         glob('meshes/visual/*.stl')),
        ('share/' + package_name + '/meshes/collision',
         glob('meshes/collision/*.stl')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        # YCB model.sdf/model.config 安装；meshes 由 fetch_ycb.sh 恢复（不入 git）
        ('share/' + package_name + '/models', glob('models/LICENSE*') + glob('models/README.md')),
    ] + [
        ('share/' + package_name + '/models/' + os.path.basename(model_dir),
         glob(model_dir + '/model.sdf') + glob(model_dir + '/model.config'))
        for model_dir in sorted(glob('models/*', recursive=False))
        if glob(model_dir + '/model.sdf')
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=False,
    maintainer='mu',
    maintainer_email='2155413529@qq.com',
    description=(
        'IVG 抓取验证 Gazebo Harmonic 场景：桌台 + YCB 杂乱摆位 + 俯视 rgbd，'
        '点云经光学系中继喂 ivg_graspnet，评分对 GT 对象位姿核验抓取。'
    ),
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'generate_table_world = ivg_sim.generate_table_world:main',
            'cloud_relay = ivg_sim.cloud_relay:main',
            'score_grasps = ivg_sim.score_grasps:main',
            'execute_grasp = ivg_sim.execute_grasp:main',
        ],
    },
)
