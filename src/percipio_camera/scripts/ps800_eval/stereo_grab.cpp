// 立体验证采集工具（2026-09-16）：ir 模式=强制激光双目对+标定；depth 模式=设备端 18 图案深度
// 用法: stereo_grab <ir|depth> <count> <outdir>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

#include "TYApi.h"
#include "Utils.hpp"

static void write_pgm(const char* path, const uint8_t* p, int w, int h) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    fwrite(p, 1, static_cast<size_t>(w) * h, f);
    fclose(f);
}

static void write_pgm16(const char* path, const uint16_t* p, int w, int h) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P5\n%d %d\n65535\n", w, h);
    std::vector<uint8_t> buf(static_cast<size_t>(w) * h * 2);
    for (size_t i = 0; i < buf.size() / 2; i++) {
        buf[2 * i] = static_cast<uint8_t>(p[i] >> 8);      // big-endian
        buf[2 * i + 1] = static_cast<uint8_t>(p[i] & 0xFF);
    }
    fwrite(buf.data(), 1, buf.size(), f);
    fclose(f);
}

static void dump_calib(TY_DEV_HANDLE h, uint32_t comp, const char* name, FILE* out) {
    TY_CAMERA_CALIB_INFO c;
    TY_STATUS s = TYGetStruct(h, comp, TY_STRUCT_CAM_CALIB_DATA, &c, sizeof(c));
    fprintf(out, "calib %s status %d size %dx%d\n", name, s, c.intrinsicWidth, c.intrinsicHeight);
    fprintf(out, "intrinsic");
    for (float v : c.intrinsic.data) fprintf(out, " %g", v);
    fprintf(out, "\nextrinsic");
    for (float v : c.extrinsic.data) fprintf(out, " %g", v);
    fprintf(out, "\ndistortion");
    for (float v : c.distortion.data) fprintf(out, " %g", v);
    fprintf(out, "\n\n");
}

int main(int argc, char** argv) {
    if (argc < 4) { printf("usage: stereo_grab <ir|depth> <count> <outdir>\n"); return 1; }
    const std::string mode = argv[1];
    const int want = atoi(argv[2]);
    const std::string outdir = argv[3];

    TYInitLib();
    std::vector<TY_DEVICE_BASE_INFO> selected;
    TY_STATUS s = selectDevice(TY_INTERFACE_ETHERNET, "", "169.254.10.110", 1, selected);
    if (s != TY_STATUS_OK || selected.empty()) { printf("device not found\n"); return 1; }
    TY_INTERFACE_HANDLE hIface;
    TY_DEV_HANDLE h;
    if (TYOpenInterface(selected[0].iface.id, &hIface) ||
        TYOpenDevice(hIface, selected[0].id, &h)) { printf("open fail\n"); return 1; }

    FILE* calib_f = fopen((outdir + "/calib.txt").c_str(), "w");

    if (mode == "ir") {
        dump_calib(h, TY_COMPONENT_IR_CAM_LEFT, "left_ir", calib_f);
        dump_calib(h, TY_COMPONENT_IR_CAM_RIGHT, "right_ir", calib_f);
        fclose(calib_f);

        TYSetBool(h, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, false);
        TYSetInt(h, TY_COMPONENT_LASER, TY_INT_LASER_POWER, 100);
        TYSetInt(h, TY_COMPONENT_IR_CAM_LEFT, TY_INT_EXPOSURE_TIME, 990);
        TYSetInt(h, TY_COMPONENT_IR_CAM_RIGHT, TY_INT_EXPOSURE_TIME, 990);
        int32_t e = 0;
        TYGetInt(h, TY_COMPONENT_IR_CAM_LEFT, TY_INT_EXPOSURE_TIME, &e); printf("exp L=%d ", e);
        TYGetInt(h, TY_COMPONENT_IR_CAM_RIGHT, TY_INT_EXPOSURE_TIME, &e); printf("R=%d\n", e);

        s = TYEnableComponents(h, TY_COMPONENT_IR_CAM_LEFT | TY_COMPONENT_IR_CAM_RIGHT);
        printf("enable dual IR: %d\n", s);

        uint32_t frame_size = 0;
        TYGetFrameBufferSize(h, &frame_size);
        std::vector<uint8_t> b0(frame_size), b1(frame_size);
        TYEnqueueBuffer(h, b0.data(), frame_size);
        TYEnqueueBuffer(h, b1.data(), frame_size);
        TYStartCapture(h);

        int pairs = 0;
        for (int i = 0; i < want * 3 + 10 && pairs < want; i++) {
            TY_FRAME_DATA frame;
            s = TYFetchFrame(h, &frame, 3000);
            if (s != TY_STATUS_OK) { printf("fetch fail %d\n", s); break; }
            const uint8_t* pl = nullptr;
            const uint8_t* pr = nullptr;
            int w = 0, hh = 0;
            for (int k = 0; k < frame.validCount; k++) {
                if (frame.image[k].status != TY_STATUS_OK) continue;
                if (frame.image[k].componentID == TY_COMPONENT_IR_CAM_LEFT) {
                    pl = static_cast<const uint8_t*>(frame.image[k].buffer);
                    w = frame.image[k].width; hh = frame.image[k].height;
                } else if (frame.image[k].componentID == TY_COMPONENT_IR_CAM_RIGHT) {
                    pr = static_cast<const uint8_t*>(frame.image[k].buffer);
                }
            }
            if (pl && pr) {
                char path[256];
                snprintf(path, sizeof(path), "%s/L%03d.pgm", outdir.c_str(), pairs);
                write_pgm(path, pl, w, hh);
                snprintf(path, sizeof(path), "%s/R%03d.pgm", outdir.c_str(), pairs);
                write_pgm(path, pr, w, hh);
                pairs++;
            }
            TYEnqueueBuffer(h, frame.userBuffer, frame.bufferSize);
        }
        TYStopCapture(h);
        printf("saved %d pairs\n", pairs);
    } else if (mode == "depth") {
        dump_calib(h, TY_COMPONENT_DEPTH_CAM, "depth", calib_f);
        float scale = 0.f;
        TYGetFloat(h, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &scale);
        fprintf(calib_f, "scale_unit_mm %g\n", scale);
        fclose(calib_f);

        s = TYEnableComponents(h, TY_COMPONENT_DEPTH_CAM);
        printf("enable depth: %d scale=%.3fmm\n", s, scale);

        uint32_t frame_size = 0;
        TYGetFrameBufferSize(h, &frame_size);
        std::vector<uint8_t> b0(frame_size), b1(frame_size);
        TYEnqueueBuffer(h, b0.data(), frame_size);
        TYEnqueueBuffer(h, b1.data(), frame_size);
        TYStartCapture(h);

        int saved = 0;
        for (int i = 0; i < want * 3 + 10 && saved < want; i++) {
            TY_FRAME_DATA frame;
            s = TYFetchFrame(h, &frame, 4000);
            if (s != TY_STATUS_OK) { printf("fetch fail %d\n", s); break; }
            for (int k = 0; k < frame.validCount; k++) {
                if (frame.image[k].status != TY_STATUS_OK) continue;
                if (frame.image[k].componentID == TY_COMPONENT_DEPTH_CAM) {
                    char path[256];
                    snprintf(path, sizeof(path), "%s/D%03d.pgm", outdir.c_str(), saved);
                    write_pgm16(path, static_cast<const uint16_t*>(frame.image[k].buffer),
                                frame.image[k].width, frame.image[k].height);
                    saved++;
                }
            }
            TYEnqueueBuffer(h, frame.userBuffer, frame.bufferSize);
        }
        TYStopCapture(h);
        printf("saved %d depth frames\n", saved);
    } else {
        fclose(calib_f);
        printf("unknown mode\n");
    }

    TYCloseDevice(h);
    TYCloseInterface(hIface);
    TYDeinitLib();
    return 0;
}
