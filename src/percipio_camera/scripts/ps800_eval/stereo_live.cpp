// 实时单图案彩色深度演示（2026-09-16）
// 单窗口 JET 彩色深度，色阶按场景 5%-95% 分位自动拉伸；q/ESC 退出；a 切 k 帧融合(1/2/4/8)
// 注意：TY 标定结构体是 float32，必须先 CV_32F 再 convertTo(CV_64F)，直接包装=NaN 全黑
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#include <opencv2/opencv.hpp>

#include "TYApi.h"
#include "Utils.hpp"

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    TYInitLib();
    std::vector<TY_DEVICE_BASE_INFO> selected;
    TY_STATUS s = selectDevice(TY_INTERFACE_ETHERNET, "", "169.254.10.110", 1, selected);
    if (s != TY_STATUS_OK || selected.empty()) { printf("device not found\n"); return 1; }
    TY_INTERFACE_HANDLE hIface;
    TY_DEV_HANDLE h;
    if (TYOpenInterface(selected[0].iface.id, &hIface) ||
        TYOpenDevice(hIface, selected[0].id, &h)) { printf("open fail\n"); return 1; }

    TY_CAMERA_CALIB_INFO cl, cr;
    TYGetStruct(h, TY_COMPONENT_IR_CAM_LEFT, TY_STRUCT_CAM_CALIB_DATA, &cl, sizeof(cl));
    TYGetStruct(h, TY_COMPONENT_IR_CAM_RIGHT, TY_STRUCT_CAM_CALIB_DATA, &cr, sizeof(cr));

    cv::Mat K1, K2, D1, D2, E;
    cv::Mat(3, 3, CV_32F, cl.intrinsic.data).convertTo(K1, CV_64F);
    cv::Mat(3, 3, CV_32F, cr.intrinsic.data).convertTo(K2, CV_64F);
    cv::Mat(1, 8, CV_32F, cl.distortion.data).convertTo(D1, CV_64F);
    cv::Mat(1, 8, CV_32F, cr.distortion.data).convertTo(D2, CV_64F);
    cv::Mat(4, 4, CV_32F, cr.extrinsic.data).convertTo(E, CV_64F);
    cv::Mat R = E(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat T = E(cv::Rect(3, 0, 1, 3)).clone();
    const cv::Size full(1280, 960);
    const cv::Size half(640, 480);

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2, full, R, T, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 0);
    const double f_half = P1.at<double>(0, 0) * 0.5;
    const double baseline = -P2.at<double>(0, 3) / P1.at<double>(0, 0);
    cv::Mat m1x, m1y, m2x, m2y;
    cv::initUndistortRectifyMap(K1, D1, R1, P1, full, CV_32FC1, m1x, m1y);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, full, CV_32FC1, m2x, m2y);
    printf("live: f_h=%.1fpx B=%.1fmm\n", f_half, baseline);

    TYSetBool(h, TY_COMPONENT_LASER, TY_BOOL_LASER_AUTO_CTRL, false);
    TYSetInt(h, TY_COMPONENT_LASER, TY_INT_LASER_POWER, 100);
    TYSetInt(h, TY_COMPONENT_IR_CAM_LEFT, TY_INT_EXPOSURE_TIME, 990);
    TYSetInt(h, TY_COMPONENT_IR_CAM_RIGHT, TY_INT_EXPOSURE_TIME, 990);
    s = TYEnableComponents(h, TY_COMPONENT_IR_CAM_LEFT | TY_COMPONENT_IR_CAM_RIGHT);
    if (s) { printf("enable dual IR fail %d\n", s); return 1; }

    uint32_t frame_size = 0;
    TYGetFrameBufferSize(h, &frame_size);
    std::vector<uint8_t> b0(frame_size), b1(frame_size);
    TYEnqueueBuffer(h, b0.data(), frame_size);
    TYEnqueueBuffer(h, b1.data(), frame_size);
    TYStartCapture(h);

    auto sgbm = cv::StereoSGBM::create(
        0, 128, 5, 200, 3200, 5, 31, 10, 100, 2, cv::StereoSGBM::MODE_SGBM_3WAY);

    int k_avg = 1;
    int frames_in_avg = 0;
    cv::Mat z_acc;
    double z_lo = 300, z_hi = 1200;

    auto t_start = std::chrono::steady_clock::now();
    int frames_shown = 0;
    double proc_ms_avg = 0;
    int proc_n = 0;

    while (true) {
        TY_FRAME_DATA frame;
        s = TYFetchFrame(h, &frame, 2000);
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
            auto t0 = std::chrono::steady_clock::now();
            cv::Mat L(hh, w, CV_8UC1, const_cast<uint8_t*>(pl));
            cv::Mat Rr(hh, w, CV_8UC1, const_cast<uint8_t*>(pr));
            cv::Mat Lr, Rrc;
            cv::remap(L, Lr, m1x, m1y, cv::INTER_LINEAR);
            cv::remap(Rr, Rrc, m2x, m2y, cv::INTER_LINEAR);
            cv::resize(Lr, Lr, half, 0, 0, cv::INTER_LINEAR);
            cv::resize(Rrc, Rrc, half, 0, 0, cv::INTER_LINEAR);

            cv::Mat disp;
            sgbm->compute(Lr, Rrc, disp);
            disp.convertTo(disp, CV_32F, 1.0 / 16.0);

            cv::Mat z(half, CV_32FC1);
            for (int y = 0; y < half.height; y++) {
                const float* dr = disp.ptr<float>(y);
                float* zr = z.ptr<float>(y);
                for (int x = 0; x < half.width; x++) {
                    zr[x] = dr[x] > 1.f
                        ? static_cast<float>(f_half * baseline / dr[x]) : 0.f;
                }
            }

            if (k_avg > 1) {
                if (z_acc.empty() || frames_in_avg == 0) {
                    z_acc = cv::Mat::zeros(half, CV_32FC1);
                    frames_in_avg = 0;
                }
                cv::add(z_acc, z, z_acc);
                frames_in_avg++;
                if (frames_in_avg < k_avg) {
                    TYEnqueueBuffer(h, frame.userBuffer, frame.bufferSize);
                    continue;
                }
                z_acc /= static_cast<double>(k_avg);
                z = z_acc.clone();
                frames_in_avg = 0;
            }

            int valid_cnt = 0, total = 0;
            std::vector<float> zs;
            for (int y = 0; y < half.height; y += 2) {
                const float* zr = z.ptr<float>(y);
                for (int x = 0; x < half.width; x += 2) {
                    total++;
                    if (zr[x] > 200.f && zr[x] < 2000.f) {
                        valid_cnt++;
                        zs.push_back(zr[x]);
                    }
                }
            }
            auto t1 = std::chrono::steady_clock::now();
            double proc_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            proc_ms_avg = (proc_ms_avg * proc_n + proc_ms) / (proc_n + 1);
            proc_n++;
            frames_shown++;
            double elapsed = std::chrono::duration<double>(t1 - t_start).count();

            if (frames_shown % 30 == 1 && !zs.empty()) {
                std::sort(zs.begin(), zs.end());
                z_lo = zs[zs.size() / 20];
                z_hi = zs[zs.size() * 19 / 20];
                if (z_hi - z_lo < 150) {
                    double mid = 0.5 * (z_lo + z_hi);
                    z_lo = mid - 75;
                    z_hi = mid + 75;
                }
                printf("STAT frames=%d display=%.1ffps proc=%.0fms k=%d "
                       "valid=%.0f%% z_range=%.0f-%.0fmm\n",
                       frames_shown, frames_shown / elapsed, proc_ms_avg, k_avg,
                       100.0 * valid_cnt / total, z_lo, z_hi);
            }

            cv::Mat vis(half, CV_8UC1, cv::Scalar(0));
            float scale = 255.f / static_cast<float>(z_hi - z_lo);
            for (int y = 0; y < half.height; y++) {
                const float* zr = z.ptr<float>(y);
                uint8_t* vr = vis.ptr<uint8_t>(y);
                for (int x = 0; x < half.width; x++) {
                    float v = zr[x];
                    vr[x] = (v > z_lo && v < z_hi)
                        ? static_cast<uint8_t>((v - z_lo) * scale) : 0;
                }
            }
            cv::Mat color;
            cv::applyColorMap(vis, color, cv::COLORMAP_JET);

            char text[160];
            snprintf(text, sizeof(text), "%.1f fps | %.0fms | avg-k=%d | %.0f-%.0fmm",
                     frames_shown / elapsed, proc_ms_avg, k_avg, z_lo, z_hi);
            cv::putText(color, text, cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
                        0.6, cv::Scalar(255, 255, 255), 2);
            snprintf(text, sizeof(text), "a:avg  q:quit");
            cv::putText(color, text, cv::Point(10, 48), cv::FONT_HERSHEY_SIMPLEX,
                        0.5, cv::Scalar(255, 255, 255), 1);

            if (frames_shown % 30 == 1) {
                cv::imwrite("/tmp/live_snapshot.png", color);
            }

            cv::imshow("PS800-E1 depth (color)", color);
            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) break;
            if (key == 'a') { k_avg = k_avg >= 8 ? 1 : k_avg * 2; frames_in_avg = 0; }
        }
        TYEnqueueBuffer(h, frame.userBuffer, frame.bufferSize);
    }

    TYStopCapture(h);
    TYCloseDevice(h);
    TYCloseInterface(hIface);
    TYDeinitLib();
    printf("live session: shown=%d avg_proc=%.0fms\n", frames_shown, proc_ms_avg);
    return 0;
}
