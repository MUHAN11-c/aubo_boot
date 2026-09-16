// laser 设置残留检查与复位（2026-09-16）
// 实测：laser 设置跨连接自动复位（auto=1/power=50）；曝光值跨连接残留——不对称行为
#include <cstdio>
#include <ctime>
#include <vector>

#include "TYApi.h"
#include "Utils.hpp"

int main(int argc, char** argv) {
    const bool reset = argc > 1 && std::string(argv[1]) == "reset";
    TYInitLib();
    std::vector<TY_DEVICE_BASE_INFO> selected;
    TY_STATUS s = selectDevice(TY_INTERFACE_ETHERNET, "", "169.254.10.110", 1, selected);
    if (s != TY_STATUS_OK || selected.empty()) { printf("device not found\n"); return 1; }
    TY_INTERFACE_HANDLE hIface;
    TY_DEV_HANDLE h;
    if (TYOpenInterface(selected[0].iface.id, &hIface) ||
        TYOpenDevice(hIface, selected[0].id, &h)) { printf("open fail\n"); return 1; }

    bool auto_ctrl = true;
    int32_t power = -1;
    TYGetBool(h, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, &auto_ctrl);
    TYGetInt(h, TY_COMPONENT_LASER, TY_INT_LASER_POWER, &power);
    printf("laser state: auto=%d power=%d\n", auto_ctrl ? 1 : 0, power);

    if (reset) {
        s = TYSetBool(h, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, true);
        printf("reset auto=true: %d\n", s);
        s = TYSetInt(h, TY_COMPONENT_LASER, TY_INT_LASER_POWER, 50);
        printf("reset power=50: %d\n", s);
        TYGetBool(h, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, &auto_ctrl);
        TYGetInt(h, TY_COMPONENT_LASER, TY_INT_LASER_POWER, &power);
        printf("after reset: auto=%d power=%d\n", auto_ctrl ? 1 : 0, power);
    }
    TYCloseDevice(h);
    TYCloseInterface(hIface);
    TYDeinitLib();
    return 0;
}
