// 0x1610 (image number) 写入/读回验证（2026-09-16 根因分析）
// 结论参考：空闲态写入被接受（18→5→2→9 读回一致），但流式期间帧率不变
#include <cstdio>
#include <ctime>
#include <vector>

#include "TYApi.h"
#include "Utils.hpp"

int main() {
    TYInitLib();
    std::vector<TY_DEVICE_BASE_INFO> selected;
    TY_STATUS s = selectDevice(TY_INTERFACE_ETHERNET, "", "169.254.10.110", 1, selected);
    if (s != TY_STATUS_OK || selected.empty()) { printf("device not found\n"); return 1; }
    TY_INTERFACE_HANDLE hIface;
    TY_DEV_HANDLE h;
    if (TYOpenInterface(selected[0].iface.id, &hIface) ||
        TYOpenDevice(hIface, selected[0].id, &h)) { printf("open fail\n"); return 1; }

    const uint32_t comp = TY_COMPONENT_DEPTH_CAM;
    const uint32_t fid = TY_INT_SGBM_IMAGE_NUM;
    int32_t v = 0;
    TYGetInt(h, comp, fid, &v);
    printf("image number before: %d\n", v);
    for (int32_t nv : {5}) {
        s = TYSetInt(h, comp, fid, nv);
        TYGetInt(h, comp, fid, &v);
        printf("write %d -> status %d, readback %d %s\n",
               nv, s, v, v == nv ? "(ACCEPTED)" : "(IGNORED)");
    }
    TYCloseDevice(h);
    TYCloseInterface(hIface);
    TYDeinitLib();
    return 0;
}
