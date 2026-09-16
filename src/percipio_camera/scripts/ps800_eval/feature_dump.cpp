// PS800-E1 参数读回工具（2026-09-16 帧率根因分析用）
// 扫描各组件 0x1000-0x4FFF 特征 ID（高 12 位是类型），打印有效特征的名称/可写性/当前值
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

#include "TYApi.h"
#include "Utils.hpp"

static const struct { uint32_t comp; const char* name; } COMPS[] = {
    {TY_COMPONENT_DEVICE, "Device"},
    {TY_COMPONENT_DEPTH_CAM, "Depth"},
    {TY_COMPONENT_IR_CAM_LEFT, "LeftIR"},
    {TY_COMPONENT_IR_CAM_RIGHT, "RightIR"},
    {TY_COMPONENT_RGB_CAM, "RGB"},
    {TY_COMPONENT_LASER, "Laser"},
};

int main(int argc, char** argv) {
    const char* ip = argc > 1 ? argv[1] : "169.254.10.110";
    TYInitLib();
    std::vector<TY_DEVICE_BASE_INFO> selected;
    TY_STATUS s = selectDevice(TY_INTERFACE_ETHERNET, "", ip, 1, selected);
    if (s != TY_STATUS_OK || selected.empty()) {
        printf("device not found: %d\n", s);
        return 1;
    }
    TY_INTERFACE_HANDLE hIface;
    TY_DEV_HANDLE h;
    s = TYOpenInterface(selected[0].iface.id, &hIface);
    if (s) { printf("open iface fail %d\n", s); return 1; }
    s = TYOpenDevice(hIface, selected[0].id, &h);
    if (s) { printf("open device fail %d\n", s); return 1; }
    printf("model=%s\n", selected[0].modelName);

    for (auto& c : COMPS) {
        for (uint32_t fid = 0x1000; fid <= 0x4FFF; fid++) {
            TY_FEATURE_INFO info;
            s = TYGetFeatureInfo(h, c.comp, fid, &info);
            if (s != TY_STATUS_OK || !info.isValid) continue;
            printf("[%s] 0x%04x %-28s access=%d runWrite=%d ",
                   c.name, fid, info.name,
                   static_cast<int>(info.accessMode),
                   info.writableAtRun ? 1 : 0);
            int32_t iv = 0; float fv = 0.f; bool bv = false;
            switch (fid & 0xF000) {
                case TY_FEATURE_INT:
                    if (TYGetInt(h, c.comp, fid, &iv) == TY_STATUS_OK) printf("int=%d", iv);
                    break;
                case TY_FEATURE_FLOAT:
                    if (TYGetFloat(h, c.comp, fid, &fv) == TY_STATUS_OK) printf("float=%g", fv);
                    break;
                case TY_FEATURE_BOOL:
                    if (TYGetBool(h, c.comp, fid, &bv) == TY_STATUS_OK) printf("bool=%d", bv ? 1 : 0);
                    break;
                case TY_FEATURE_ENUM:
                    if (TYGetEnum(h, c.comp, fid, reinterpret_cast<uint32_t*>(&iv)) == TY_STATUS_OK)
                        printf("enum=%d", iv);
                    break;
                default:
                    printf("(other)");
            }
            printf("\n");
        }
    }
    TYCloseDevice(h);
    TYCloseInterface(hIface);
    TYDeinitLib();
    return 0;
}
