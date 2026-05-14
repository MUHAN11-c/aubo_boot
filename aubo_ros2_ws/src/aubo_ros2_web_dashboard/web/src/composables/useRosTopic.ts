/**
 * useRosTopic — 单个 ROS 话题订阅 composable
 *
 * 替代旧版: subscription_binder.js (172行)
 *
 * 自动管理订阅生命周期:
 *   - onMounted  时订阅话题
 *   - onUnmounted 时取消订阅（防止组件卸载后收到消息导致内存泄露）
 *
 * 用法:
 *   const { data } = useRosTopic('/joint_states', 'sensor_msgs/msg/JointState', 30)
 *   watch(data, (msg) => { ... })
 */
import { useRos } from './useRos'
import { canonicalRosTopic } from '@/lib/utils'

export function useRosTopic(topic: string, msgType: string, maxHz?: number) {
  const { subscribe, unsubscribe, onRosJson } = useRos()
  const data = ref<any>(null)
  const topicName = canonicalRosTopic(topic)

  onMounted(() => {
    onRosJson(topicName, (msg: any) => { data.value = msg })
    subscribe(topicName, msgType, maxHz)
  })

  onUnmounted(() => { unsubscribe(topicName) })

  return { data: readonly(data) }
}
