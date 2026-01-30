#!/usr/bin/env python3
"""
智能姿态估计脚本
自动处理触发拍照、等待和姿态估计的完整流程
"""

import rclpy
from rclpy.node import Node
from percipio_camera_interface.srv import SoftwareTrigger
from interface.srv import EstimatePose
import time
import sys

class SmartEstimator(Node):
    def __init__(self):
        super().__init__('smart_estimator')
        
        # 创建服务客户端
        self.trigger_client = self.create_client(SoftwareTrigger, '/software_trigger')
        self.estimate_client = self.create_client(EstimatePose, '/estimate_pose')
        
    def wait_for_services(self, timeout_sec=5.0):
        """等待服务就绪"""
        print("⏳ 等待服务就绪...", flush=True)
        
        if not self.trigger_client.wait_for_service(timeout_sec=timeout_sec):
            print("❌ 软触发服务未就绪", flush=True)
            print("   提示: 请确保相机节点正在运行", flush=True)
            return False
        
        if not self.estimate_client.wait_for_service(timeout_sec=timeout_sec):
            print("❌ 姿态估计服务未就绪", flush=True)
            print("   提示: 请确保 visual_pose_estimation_python 节点正在运行", flush=True)
            return False
        
        print("✅ 所有服务已就绪\n", flush=True)
        return True
    
    def trigger_capture(self, camera_id='207000152740'):
        """触发相机拍照"""
        print(f"📷 触发相机拍照 (ID: {camera_id})...", flush=True)
        
        request = SoftwareTrigger.Request()
        request.camera_id = camera_id
        
        future = self.trigger_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                print(f"✅ 拍照成功: {response.message}\n", flush=True)
                return True
            else:
                print(f"❌ 拍照失败: {response.message}\n", flush=True)
                return False
        else:
            print("❌ 软触发服务调用超时\n", flush=True)
            return False
    
    def wait_for_images(self, wait_seconds=3):
        """等待图像发布到订阅话题"""
        print(f"⏳ 等待图像发布到话题 (等待 {wait_seconds} 秒)...", flush=True)
        
        for i in range(wait_seconds, 0, -1):
            print(f"   {i}...", flush=True)
            time.sleep(1)
        
        print("✅ 等待完成\n", flush=True)
    
    def estimate_pose(self, object_id):
        """执行姿态估计"""
        print(f"🎯 执行姿态估计 (工件ID: {object_id})...", flush=True)
        
        request = EstimatePose.Request()
        request.object_id = object_id
        
        start_time = time.time()
        future = self.estimate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        elapsed_time = time.time() - start_time
        
        if future.result() is not None:
            response = future.result()
            
            print(f"\n{'='*60}")
            print(f"  姿态估计结果")
            print(f"{'='*60}")
            print(f"✓ 成功检测数量: {response.success_num}")
            print(f"✓ 总处理时间: {elapsed_time:.3f} 秒")
            print(f"✓ 算法处理时间: {response.processing_time_sec:.3f} 秒")
            
            if response.success_num > 0:
                print(f"\n详细结果:")
                for i in range(response.success_num):
                    print(f"\n  物体 {i+1}:")
                    if i < len(response.confidence):
                        print(f"    置信度: {response.confidence[i]:.3f}")
                    
                    if i < len(response.position):
                        pos = response.position[i]
                        print(f"    图像位置: ({pos.x:.1f}, {pos.y:.1f})")
                    
                    if i < len(response.grab_position):
                        grab = response.grab_position[i]
                        print(f"    抓取位置: ({grab.position.x:.3f}, {grab.position.y:.3f}, {grab.position.z:.3f})")
                
                print(f"\n{'='*60}\n")
                return True
            else:
                print(f"\n⚠️  未检测到物体")
                print(f"\n可能原因:")
                print(f"  1. 图像未成功接收（检查相机话题）")
                print(f"  2. 深度阈值不合适（检查 configs/default.yaml）")
                print(f"  3. 工件不在视野内或姿态差异过大")
                print(f"  4. 模板数量不足（建议创建5-10个姿态）")
                print(f"\n{'='*60}\n")
                return False
        else:
            print("❌ 姿态估计服务调用超时\n", flush=True)
            return False
    
    def run_complete_flow(self, object_id='321124278577', camera_id='207000152740', wait_seconds=3):
        """运行完整的姿态估计流程"""
        print(f"\n{'='*60}")
        print(f"  智能姿态估计流程")
        print(f"{'='*60}\n")
        
        # 步骤1: 等待服务就绪
        if not self.wait_for_services():
            return False
        
        # 步骤2: 触发拍照
        if not self.trigger_capture(camera_id):
            return False
        
        # 步骤3: 等待图像发布
        self.wait_for_images(wait_seconds)
        
        # 步骤4: 姿态估计
        return self.estimate_pose(object_id)


def main():
    """主函数"""
    rclpy.init()
    
    try:
        node = SmartEstimator()
        
        # 从命令行参数获取配置
        object_id = sys.argv[1] if len(sys.argv) > 1 else '321124278577'
        camera_id = sys.argv[2] if len(sys.argv) > 2 else '207000152740'
        wait_seconds = int(sys.argv[3]) if len(sys.argv) > 3 else 3
        
        # 运行完整流程
        success = node.run_complete_flow(object_id, camera_id, wait_seconds)
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print('\n⚠️  用户中断')
        sys.exit(1)
    except Exception as e:
        print(f'\n❌ 错误: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
