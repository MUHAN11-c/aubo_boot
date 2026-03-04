# -*- coding: utf-8 -*-
"""
GraspNetAPI 示例：抓取评估。
对场景中的抓取结果进行评估，计算准确率等指标。
使用前请修改 graspnet_root 与 dump_folder 路径。
"""
__author__ = 'mhgou'
__version__ = '1.0'

import numpy as np
from graspnetAPI import GraspNetEval

####################################################################
graspnet_root = '/home/gmh/graspnet'       # GraspNet 数据集根路径
dump_folder = '/home/gmh/git/rgbd_graspnet/dump_affordance_iounan/'  # 预测结果输出目录
####################################################################

sceneId = 121
camera = 'kinect'

# 创建评估器：kinect 与 realsense 各一个（test 划分）
ge_k = GraspNetEval(root=graspnet_root, camera='kinect', split='test')
ge_r = GraspNetEval(root=graspnet_root, camera='realsense', split='test')

# 评估单个场景
print('Evaluating scene:{}, camera:{}'.format(sceneId, camera))
acc = ge_k.eval_scene(scene_id=sceneId, dump_folder=dump_folder)
np_acc = np.array(acc)
print('mean accuracy:{}'.format(np.mean(np_acc)))  # 打印平均准确率

# 以下为批量评估示例（按需取消注释）
# # 评估 kinect 全部数据
# print('Evaluating kinect')
# res, ap = ge_k.eval_all(dump_folder, proc=24)

# # 评估 realsense 的 'seen' 划分
# print('Evaluating realsense')
# res, ap = ge_r.eval_seen(dump_folder, proc=24)
