// 裸 SDK IR 采集测试（2026-09-16）：绕开 ROS 驱动
// 关键发现：不解锁 laser 时独立 IR 输出为非光信号底噪（亮度不随曝光变）；
// laser/dual 模式先关自动控制并拉功率，可拿真实散斑；dual 输出 L+R 同帧硬件同步对
#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

#include "TYApi.h"
#include "Utils.hpp"

static void frame_stats(const TY_IMAGE_DATA& img, double& mean, double& std_dev) {
    const uint8_t* p = static_cast<const uint8_t*>(img.buffer);
    size_t n = static_cast<size_t>(img.width) * img.height;
    if (n == 0 || img.size < n) { mean = -1; std_dev = -1; return; }
    double sum = 0, sum2 = 0;
    for (size_t i = 0; i < n; i++) { sum += p[i]; sum2 += double(p[i]) * p[i]; }
    mean = sum / n;
    std_dev = sum2 / n - mean * mean;
    std_dev = std_dev > 0 ? __builtin_sqrt(std_dev) : 0;
}

int main(int argc, char** argv) {
    const int32_t exp_set = argc > 1 ? atoi(argv[1]) : 300;
    const std::string mode = argc > 2 ? argv[2] : "plain";
    TYInitLib();
    std::vector<TY_DEVICE_BASE_INFO> selected;
    TY_STATUS s = selectDevice(TY_INTERFACE_ETHERNET, "", "169.254.10.110", 1, selected);
    if (s != TY_STATUS_OK || selected.empty()) { printf("device not found\n"); return 1; }
    TY_INTERFACE_HANDLE hIface;
    TY_DEV_HANDLE h;
    if (TYOpenInterface(selected[0].iface.id, &hIface) ||
        TYOpenDevice(hIface, selected[0].id, &h)) { printf("open fail\n"); return 1; }

    if (mode == "laser" || mode == "both") {
        s = TYSetBool(h, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, false);
        printf("laser auto off: %d\n", s);
        for (int32_t pw : {100, 80, 60}) {
            s = TYSetInt(h, TY_COMPONENT_LASER, TY_INT_LASER_POWER, pw);
            int32_t cur = 0;
            TYGetInt(h, TY_COMPONENT_LASER, TY_INT_LASER_POWER, &cur);
            printf("laser power %d -> status %d readback %d\n", pw, s, cur);
            if (s == TY_STATUS_OK) break;
        }
    }
    if (mode == "flood" || mode == "both") {
        s = TYSetBool(h, TY_COMPONENT_DEVICE, TY_BOOL_IR_FLASHLIGHT, true);
        printf("ir floodlight on: %d\n", s);
    }
    const bool dual_ir = (mode == "dual");

    int32_t v = 0;
    TYGetInt(h, TY_COMPONENT_IR_CAM_LEFT, TY_INT_EXPOSURE_TIME, &v);
    printf("exposure before: %d\n", v);
    s = TYSetInt(h, TY_COMPONENT_IR_CAM_LEFT, TY_INT_EXPOSURE_TIME, exp_set);
    TYGetInt(h, TY_COMPONENT_IR_CAM_LEFT, TY_INT_EXPOSURE_TIME, &v);
    printf("set %d -> status %d, readback %d\n", exp_set, s, v);

    if (dual_ir) {
        s = TYSetBool(h, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, false);
        printf("laser auto off: %d\n", s);
        s = TYSetInt(h, TY_COMPONENT_LASER, TY_INT_LASER_POWER, 100);
        printf("laser power 100: %d\n", s);
    }
    const uint32_t comps = dual_ir
        ? (TY_COMPONENT_IR_CAM_LEFT | TY_COMPONENT_IR_CAM_RIGHT)
        : TY_COMPONENT_IR_CAM_LEFT;
    s = TYEnableComponents(h, comps);
    printf("enable IR comps(0x%x): %d\n", comps, s);
    if (s) { TYCloseDevice(h); TYCloseInterface(hIface); TYDeinitLib(); return 1; }

    uint32_t frame_size = 0;
    TYGetFrameBufferSize(h, &frame_size);
    std::vector<uint8_t> b0(frame_size), b1(frame_size);
    TYEnqueueBuffer(h, b0.data(), frame_size);
    TYEnqueueBuffer(h, b1.data(), frame_size);
    s = TYStartCapture(h);
    printf("start capture: %d\n", s);

    const int N = 10;
    std::vector<std::vector<double>> stats;
    std::vector<uint8_t> prev;
    double last_diff = -1;
    auto t0 = std::chrono::steady_clock::now();
    int got = 0;
    int groups_both = 0, groups_total = 0;
    for (int i = 0; i < N + 8 && got < N; i++) {
        TY_FRAME_DATA frame;
        s = TYFetchFrame(h, &frame, 3000);
        if (s != TY_STATUS_OK) { printf("fetch fail %d\n", s); break; }
        bool has_l = false, has_r = false;
        groups_total++;
        for (int k = 0; k < frame.validCount; k++) {
            const bool is_ir = frame.image[k].componentID == TY_COMPONENT_IR_CAM_LEFT ||
                               frame.image[k].componentID == TY_COMPONENT_IR_CAM_RIGHT;
            if (!is_ir || frame.image[k].status != TY_STATUS_OK) continue;
            if (frame.image[k].componentID == TY_COMPONENT_IR_CAM_LEFT) has_l = true;
            else has_r = true;
            double m, sd;
            frame_stats(frame.image[k], m, sd);
            stats.push_back({m, sd,
                double(frame.image[k].width), double(frame.image[k].height),
                double(frame.image[k].componentID == TY_COMPONENT_IR_CAM_LEFT ? 0 : 1)});
            const uint8_t* p = static_cast<const uint8_t*>(frame.image[k].buffer);
            size_t n = static_cast<size_t>(frame.image[k].width) * frame.image[k].height;
            if (!prev.empty() && prev.size() == n &&
                frame.image[k].componentID == TY_COMPONENT_IR_CAM_LEFT) {
                double dsum = 0;
                for (size_t j = 0; j < n; j++) dsum += std::abs(int(p[j]) - int(prev[j]));
                last_diff = dsum / n;
            }
            if (frame.image[k].componentID == TY_COMPONENT_IR_CAM_LEFT)
                prev.assign(p, p + n);
            got++;
        }
        if (has_l && has_r) groups_both++;
        TYEnqueueBuffer(h, frame.userBuffer, frame.bufferSize);
    }
    auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    TYStopCapture(h);
    TYGetInt(h, TY_COMPONENT_IR_CAM_LEFT, TY_INT_EXPOSURE_TIME, &v);
    printf("exposure after stream: %d\n", v);

    printf("got %d IR frames in %.2fs (%.2f fps) last_consecutive_diff=%.2f groups_both=%d/%d\n",
           got, dt, got > 1 ? (got - 1) / dt : 0.0, last_diff, groups_both, groups_total);
    for (size_t i = 0; i < stats.size(); i++) {
        printf("  f%zu[%s]: %.0fx%.0f mean=%.1f std=%.1f\n",
               i, stats[i][4] == 0 ? "L" : "R",
               stats[i][2], stats[i][3], stats[i][0], stats[i][1]);
    }
    if (!prev.empty()) {
        FILE* f = fopen("/tmp/raw_ir_laser.pgm", "wb");
        if (f) {
            fprintf(f, "P5\n%zu %zu\n255\n", prev.size() / 960, size_t(960));
            fwrite(prev.data(), 1, prev.size(), f);
            fclose(f);
            printf("saved /tmp/raw_ir_laser.pgm\n");
        }
    }
    TYCloseDevice(h);
    TYCloseInterface(hIface);
    TYDeinitLib();
    return 0;
}
