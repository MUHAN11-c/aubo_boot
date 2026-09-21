// PS800-E1 温度探针（2026-09-20 可行性验证用）
// 回答「原始 SDK 温度能不能直接读到」。只读、不开流、不点激光，独占连接。
// 三条路径逐一探测：
//   A. Device 组件全特征扫描，打印名字含 Temp 的特性（GenICam 命名面）
//   B. TY_ENUM_TEMPERATURE_ID(0x0223) 选测点 + TYGetStruct(TY_STRUCT_TEMPERATURE)
//   C. TYGetStruct 按 1..N 条 TY_TEMP_DATA 探 struct 尺寸（B 失败时的兜底）
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include "TYApi.h"
#include "Utils.hpp"

static const uint32_t TEMP_ENUM_ID  = 0x0223 | TY_FEATURE_ENUM;
static const uint32_t TEMP_STRUCT_ID = 0x0224 | TY_FEATURE_STRUCT;

// TY_TEMPERATURE_ID_LIST 的名字（TYDefs.h:765 起）
static const char* TEMP_ID_NAME[] = {"Left", "Right", "Color", "CPU", "MainBoard"};

static const char* ok(TY_STATUS s) { return s == TY_STATUS_OK ? "OK" : TYErrorString(s); }

static void scan_temp_named_features(TY_DEV_HANDLE h) {
    printf("== A. Device 组件名字含 Temp 的特征 ==\n");
    int found = 0;
    for (uint32_t fid = 0x1000; fid <= 0x4FFF; fid++) {
        TY_FEATURE_INFO info;
        TY_STATUS s = TYGetFeatureInfo(h, TY_COMPONENT_DEVICE, fid, &info);
        if (s != TY_STATUS_OK || !info.isValid) continue;
        std::string name = info.name ? info.name : "";
        std::string lower = name;
        for (auto& ch : lower) ch = static_cast<char>(::tolower(ch));
        if (lower.find("temp") == std::string::npos) continue;
        found++;
        printf("[0x%04x] %-32s access=%d ", fid, name.c_str(),
               static_cast<int>(info.accessMode));
        int32_t iv = 0; float fv = 0.f; bool bv = false;
        char strbuf[64] = {0};
        switch (fid & 0xF000) {
            case TY_FEATURE_INT:
                if (TYGetInt(h, TY_COMPONENT_DEVICE, fid, &iv) == TY_STATUS_OK) printf("int=%d", iv);
                break;
            case TY_FEATURE_FLOAT:
                if (TYGetFloat(h, TY_COMPONENT_DEVICE, fid, &fv) == TY_STATUS_OK) printf("float=%g", fv);
                break;
            case TY_FEATURE_BOOL:
                if (TYGetBool(h, TY_COMPONENT_DEVICE, fid, &bv) == TY_STATUS_OK) printf("bool=%d", bv ? 1 : 0);
                break;
            case TY_FEATURE_ENUM:
                if (TYGetEnum(h, TY_COMPONENT_DEVICE, fid,
                              reinterpret_cast<uint32_t*>(&iv)) == TY_STATUS_OK) printf("enum=%d", iv);
                break;
            case TY_FEATURE_STRING:
                if (TYGetString(h, TY_COMPONENT_DEVICE, fid, strbuf, sizeof(strbuf)) == TY_STATUS_OK)
                    printf("str=%s", strbuf);
                break;
            default: printf("(struct/cmd)");
        }
        printf("\n");
    }
    if (!found) printf("(none)\n");
}

static void dump_temp_data(const TY_TEMP_DATA& d) {
    char name[17] = {0}, temp[17] = {0}, desc[17] = {0};
    memcpy(name, d.name, 16); memcpy(temp, d.temp, 16); memcpy(desc, d.desc, 16);
    printf("    id=%u name=%s temp=%s desc=%s\n", d.id, name, temp, desc);
}

static void probe_struct_path(TY_DEV_HANDLE h) {
    printf("== B. TY_ENUM_TEMPERATURE_ID 0x%04x + TY_STRUCT_TEMPERATURE 0x%04x ==\n",
           TEMP_ENUM_ID, TEMP_STRUCT_ID);

    TY_FEATURE_INFO info;
    TY_STATUS s = TYGetFeatureInfo(h, TY_COMPONENT_DEVICE, TEMP_ENUM_ID, &info);
    printf("enum feature info: %s isValid=%d\n", ok(s), info.isValid ? 1 : 0);
    s = TYGetFeatureInfo(h, TY_COMPONENT_DEVICE, TEMP_STRUCT_ID, &info);
    printf("struct feature info: %s isValid=%d\n", ok(s), info.isValid ? 1 : 0);

    // 0..4 对应 LEFT/RIGHT/COLOR/CPU/MAIN_BOARD（TY_TEMPERATURE_ID_LIST）
    for (uint32_t sel = 0; sel <= 4; sel++) {
        TY_STATUS es = TYSetEnum(h, TY_COMPONENT_DEVICE, TEMP_ENUM_ID, sel);
        if (es != TY_STATUS_OK) {
            printf("sel=%u TYSetEnum: %s（枚举路径不可用则试 C）\n", sel, TYErrorString(es));
            break;
        }
        TY_TEMP_DATA d;
        memset(&d, 0, sizeof(d));
        TY_STATUS gs = TYGetStruct(h, TY_COMPONENT_DEVICE, TEMP_STRUCT_ID, &d, sizeof(d));
        printf("sel=%u %-10s TYGetStruct(52B): %s", sel,
               TEMP_ID_NAME[sel], ok(gs));
        if (gs == TY_STATUS_OK) dump_temp_data(d);
        else printf("\n");
    }
}

static void probe_size_scan(TY_DEV_HANDLE h) {
    printf("== C. TY_STRUCT_TEMPERATURE 尺寸扫描（1..8 条 TY_TEMP_DATA）==\n");
    std::vector<TY_TEMP_DATA> buf(8);
    for (size_t n = 1; n <= 8; n++) {
        memset(buf.data(), 0, sizeof(TY_TEMP_DATA) * 8);
        TY_STATUS s = TYGetStruct(h, TY_COMPONENT_DEVICE, TEMP_STRUCT_ID,
                                  buf.data(), static_cast<uint32_t>(sizeof(TY_TEMP_DATA) * n));
        printf("n=%zu size=%zu: %s", n, sizeof(TY_TEMP_DATA) * n, ok(s));
        if (s == TY_STATUS_OK) {
            printf("\n");
            for (size_t i = 0; i < n; i++) dump_temp_data(buf[i]);
            return;
        }
        printf("\n");
    }
}

static void probe_device_xml(TY_DEV_HANDLE h) {
    printf("== D. 设备 XML 温度节点检查 ==\n");
    uint32_t size = 0;
    TY_STATUS s = TYGetDeviceXMLSize(h, &size);
    if (s != TY_STATUS_OK || size == 0) { printf("xml size fail: %s\n", ok(s)); return; }
    std::string xml(size, '\0');
    uint32_t out_size = size;
    s = TYGetDeviceXML(h, &xml[0], size, &out_size);
    if (s != TY_STATUS_OK) { printf("xml read fail: %s\n", ok(s)); return; }
    xml.resize(out_size);
    int hits = 0;
    // 逐行找含 Temp/temp 的行，GenICam XML 里特性名是 pFeatureName/String
    size_t pos = 0;
    while (pos < xml.size()) {
        size_t eol = xml.find('\n', pos);
        if (eol == std::string::npos) eol = xml.size();
        std::string line = xml.substr(pos, eol - pos);
        std::string lower = line;
        for (auto& ch : lower) ch = static_cast<char>(::tolower(ch));
        if (lower.find("temp") != std::string::npos) {
            printf("  %s\n", line.c_str());
            hits++;
        }
        pos = eol + 1;
    }
    if (!hits) printf("(xml %u 字节，无任何 Temp 节点)\n", size);
}

int main(int argc, char** argv) {
    const char* ip = argc > 1 ? argv[1] : "169.254.10.110";
    TYInitLib();
    std::vector<TY_DEVICE_BASE_INFO> selected;
    TY_STATUS s = selectDevice(TY_INTERFACE_ETHERNET, "", ip, 1, selected);
    if (s != TY_STATUS_OK || selected.empty()) {
        printf("device not found: %s\n", TYErrorString(s));
        return 1;
    }
    TY_INTERFACE_HANDLE hIface;
    TY_DEV_HANDLE h;
    s = TYOpenInterface(selected[0].iface.id, &hIface);
    if (s) { printf("open iface fail: %s\n", TYErrorString(s)); return 1; }
    s = TYOpenDevice(hIface, selected[0].id, &h);
    if (s) { printf("open device fail: %s\n", TYErrorString(s)); return 1; }
    printf("model=%s ip=%s\n", selected[0].modelName, ip);

    scan_temp_named_features(h);
    probe_struct_path(h);
    probe_size_scan(h);
    probe_device_xml(h);

    TYCloseDevice(h);
    TYCloseInterface(hIface);
    TYDeinitLib();
    return 0;
}
