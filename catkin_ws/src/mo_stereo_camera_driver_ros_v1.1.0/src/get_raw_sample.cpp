#include "common_function.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
using namespace cv;

void usage() {
    printf("Usage:\nOpen camera by videoIdx: ./get_raw_sample [videoIdx]\n");
    printf("Locate camera automatically: ./get_raw_sample\n\n");
}

int32_t main(int32_t argc, char** argv) {
    ros::init(argc, argv, "get_raw_sample");
    ros::NodeHandle nh;

    // 發布左右眼原始 Bayer（16UC1）
    ros::Publisher pubLeft  = nh.advertise<sensor_msgs::Image>("/mo_camera/left/image_raw",  1);
    ros::Publisher pubRight = nh.advertise<sensor_msgs::Image>("/mo_camera/right/image_raw", 1);

    usage();
    printf("The version of SDK is %s\n", moGetSdkVersion());

    MO_CAMERA_HANDLE hCam = MO_INVALID_HANDLE;
    char caCameraPath[64];
    if (0 != cameraSampleInit(argc, argv, caCameraPath)) return -1;

    if (0 != moOpenUVCCameraByPath(caCameraPath, &hCam)) {
        printf("Open Camera Failed\n");
        return -1;
    }

    uint16_t w = 0, h = 0;
    moGetVideoResolution(hCam, &w, &h);
    printf("Resolution: %d x %d\n", w, h);

    // 設定 RAW 模式（Bayer + Bayer）
    if (0 != moSetVideoMode(hCam, MVM_RAW)) {
        printf("moSetVideoMode(MVM_RAW) failed\n");
        return -1;
    }

    // 補光燈
    moSetFilllightType(hCam, MFT_ON);

    uint64_t u64FrameNum    = 0;
    uint8_t* pu8FrameBuf    = NULL;
    uint8_t* pu8Left        = NULL;
    uint8_t* pu8Right       = NULL;
    double   fps            = 0.0;
    char     caFPS[24]      = {0};

    // 每幀大小：16bit Bayer = width * height * 2 bytes
    const uint32_t frameBytes = (uint32_t)w * h * 2;

    printf("Publishing raw Bayer images... Press Ctrl+C to quit\n");

    // 相機 20Hz，sleep 50ms 對齊，避免無謂的空轉浪費 CPU
    while (ros::ok()) {
        ms_sleep(50);

        // 取得影像幀
        if (0 != moGetCurrentFrame(hCam, &u64FrameNum, &pu8FrameBuf)) {
            printf("moGetCurrentFrame failed\n");
            continue;
        }

        // FPS
        if (0 == moGetRealTimeFPS(hCam, &fps))
            snprintf(caFPS, sizeof(caFPS), "FPS: %.1f", fps);
        else
            snprintf(caFPS, sizeof(caFPS), "FPS: Calculating...");

        printf("\rFrameNum: %lu  %s", u64FrameNum, caFPS);
        fflush(stdout);

        // 取得左右 Bayer（文件 3.2.17 moGetRawImage）
        if (0 != moGetRawImage(hCam, pu8FrameBuf, &pu8Left, &pu8Right)) {
            printf("\nmoGetRawImage failed\n");
            continue;
        }

        ros::Time stamp = ros::Time::now();

        // 16bit Bayer → 8bit：normalize 自動對應實際值域到 0~255
        // 比 convertTo(1/256) 正確，不會因為相機只用 10/12bit 而過亮
        auto publishMono8 = [&](uint8_t* data, const std::string& frameId, ros::Publisher& pub) {
            Mat raw16(h, w, CV_16UC1, data);
            Mat out8;
            normalize(raw16, out8, 0, 255, NORM_MINMAX, CV_8UC1);
            std_msgs::Header hdr;
            hdr.stamp    = stamp;
            hdr.frame_id = frameId;
            pub.publish(cv_bridge::CvImage(hdr, "mono8", out8).toImageMsg());
        };

        publishMono8(pu8Left,  "camera_left",  pubLeft);
        publishMono8(pu8Right, "camera_right", pubRight);

        ros::spinOnce();
    }

    moCloseCamera(&hCam);
    return 0;
}

