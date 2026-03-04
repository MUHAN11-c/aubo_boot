# -*- coding: utf-8 -*-
"""
GraspNetAPI 示例：从 6D 抓取生成矩形抓取。
批量将 6D 抓取转换为矩形抓取并保存为 .npy 文件，支持多进程。
使用前请修改 graspnet_root 与 NUM_PROCESS。
"""
__author__ = 'mhgou'
__version__ = '1.0'

from graspnetAPI import GraspNet
from graspnetAPI.graspnet import TOTAL_SCENE_NUM
import os
import numpy as np
from tqdm import tqdm

######################################################################
NUM_PROCESS = 24  # 使用的 CPU 核心数，请按机器配置修改
######################################################################


def generate_scene_rectangle_grasp(sceneId, dump_folder, camera):
    """
    为指定场景、相机生成矩形抓取并保存。
    :param sceneId: 场景 ID
    :param dump_folder: 输出目录
    :param camera: 相机类型 ('kinect' / 'realsense')
    """
    g = GraspNet(graspnet_root, camera=camera, split='all')
    objIds = g.getObjIds(sceneIds=sceneId)
    grasp_labels = g.loadGraspLabels(objIds)
    collision_labels = g.loadCollisionLabels(sceneIds=sceneId)

    # 创建 scene_xxxx/camera 目录
    scene_dir = os.path.join(dump_folder, 'scene_%04d' % sceneId)
    if not os.path.exists(scene_dir):
        os.mkdir(scene_dir)
    camera_dir = os.path.join(scene_dir, camera)
    if not os.path.exists(camera_dir):
        os.mkdir(camera_dir)

    # 对该场景下 256 个标注逐一生成矩形抓取
    for annId in tqdm(range(256), 'Scene:{}, Camera:{}'.format(sceneId, camera)):
        _6d_grasp = g.loadGrasp(sceneId=sceneId, annId=annId, format='6d', camera=camera,
                                grasp_labels=grasp_labels, collision_labels=collision_labels,
                                fric_coef_thresh=1.0)
        rect_grasp_group = _6d_grasp.to_rect_grasp_group(camera)
        rect_grasp_group.save_npy(os.path.join(camera_dir, '%04d.npy' % annId))


if __name__ == '__main__':
    ####################################################################
    graspnet_root = '/home/minghao/graspnet'  # GraspNet 数据集根路径，请按需修改
    ####################################################################

    dump_folder = 'rect_labels'
    if not os.path.exists(dump_folder):
        os.mkdir(dump_folder)

    if NUM_PROCESS > 1:
        # 多进程：对 120 个场景 × 2 种相机 并行生成
        from multiprocessing import Pool
        pool = Pool(24)
        for camera in ['realsense', 'kinect']:
            for sceneId in range(120):
                pool.apply_async(func=generate_scene_rectangle_grasp, args=(sceneId, dump_folder, camera))
        pool.close()
        pool.join()
    else:
        # 单进程运行：需先设置 sceneId、camera，例如 sceneId=0, camera='kinect'
        sceneId = 0
        camera = 'kinect'
        generate_scene_rectangle_grasp(sceneId, dump_folder, camera)
