#include <cstddef>
#include <iostream>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

int main()
{
  using namespace aubo_robot_namespace;
  std::cout
    << "ServiceInterface.size=" << sizeof(ServiceInterface) << '\n'
    << "JointStatus.size=" << sizeof(JointStatus) << '\n'
    << "JointStatus.align=" << alignof(JointStatus) << '\n'
    << "JointStatus.jointPosJ=" << offsetof(JointStatus, jointPosJ) << '\n'
    << "JointStatus.jointErrorNum=" << offsetof(JointStatus, jointErrorNum) << '\n'
    << "RobotDhPara.size=" << sizeof(RobotDhPara) << '\n'
    << "wayPoint_S.size=" << sizeof(wayPoint_S) << '\n'
    << "JointRangeOfMotion.size=" << sizeof(JointRangeOfMotion) << '\n'
    << "ToolInEndDesc.size=" << sizeof(ToolInEndDesc) << '\n';
}
