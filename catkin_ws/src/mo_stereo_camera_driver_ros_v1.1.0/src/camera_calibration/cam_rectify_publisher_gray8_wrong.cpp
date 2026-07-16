#include "common_function.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
using namespace cv;

bool FOR_DEBUG = false; 

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
    // Get Rectify Camera Frame
    // =======================================
    int32_t n32Result = 0;
    n32Result = moOpenUVCCameraByPath(caCameraPath, &hCameraHandle);
    if (0 != n32Result) {
        printf("Open Camera Failed %d\n", n32Result);
        return -1;
    }

    uint64_t u64ImageFrameNum = 0;
    uint8_t* pu8FrameBuffer = NULL;
    uint8_t* pu8OnlyY = NULL;
    uint8_t* pu8RectifiedRightYUVI420Img = NULL;
    double d8FPS = 0.0f;
    char caFPS[24] = {0};

    // Get video resolution
    uint16_t u16VideoFrameWidth = 0;
    uint16_t u16VideoFrameHeight = 0;

    n32Result = moGetVideoResolution(hCameraHandle, &u16VideoFrameWidth, &u16VideoFrameHeight);
    if (0 != n32Result) {
        printf("Get video resolution failed %d\n", n32Result);
        return -1;
    }

    // Set video mode to RECTIFIED mode
    n32Result = moSetVideoMode(hCameraHandle, MVM_RECTIFIED);
    if (0 != n32Result) {
        printf("Error: moSetVideoMode failed %d\n", n32Result);
        return -4;
    }
    printf("Press ENTER to quit the running process\n");

    while (0 == getchar_nb(FETCH_AND_DISPLAY_TIME_LENGTH)) {
        // Get current video frame
        n32Result = moGetCurrentFrame(hCameraHandle, &u64ImageFrameNum, &pu8FrameBuffer);
        if (0 != n32Result) {
            printf("Get current frame failed.Try Again...\n");
            continue;
    }

    // Get real-time FPS
    n32Result = moGetRealTimeFPS(hCameraHandle, &d8FPS);
    // Failed to get FPS(usually caused by insufficient frame counts during initial period)
    if (0 != n32Result) {
        sprintf(caFPS, "FPS: Calculating...");
    } else {
        sprintf(caFPS, "FPS: %f", d8FPS);
    }
    printf("\nFrameNum: %lu  %s\n", u64ImageFrameNum, caFPS);

    // Get rectified left and right image
    n32Result = moGetRectifiedImage(hCameraHandle, pu8FrameBuffer, &pu8OnlyY, &pu8RectifiedRightYUVI420Img);

    /*
    // Get rectified left and right image with timestamp
    uint64_t u64Timestamp;
    n32Result = moGetRectifiedImageWithTimestamp(hCameraHandle, pu8FrameBuffer, &pu8OnlyY,
                                                    &pu8RectifiedRightYUVI420Img, &u64Timestamp);
    printf("Timestamp: %lu\n", u64Timestamp);
    */

    // Get each frame ======================== camera frame will be at matLeftBGR and matRightBGR
    waitKey(FETCH_AND_DISPLAY_TIME_LENGTH);
        Mat matLeftBGR;

        # 已經拿到的frame是存在matLeftRectify變數了, 且rectify後取到的已經是gray mono8了
        Mat matLeftRectify(u16VideoFrameHeight, u16VideoFrameWidth, CV_8UC1, pu8OnlyY);
        cvtColor(matLeftRectify, matLeftBGR, COLOR_GRAY2BGR);
        // Mark fps at top left corner
        putText(matLeftBGR, string(caFPS), Point(50, 60), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
        imshow("LeftRectify", matLeftBGR);

        Mat matRightBGR;  // 1.5 = 12bits / 8bits
        Mat matRightRectify(u16VideoFrameHeight * 1.5, u16VideoFrameWidth, CV_8UC1, pu8RectifiedRightYUVI420Img);
        cvtColor(matRightRectify, matRightBGR, COLOR_YUV2BGR_I420);
        // Mark fps at top left corner
        putText(matRightBGR, string(caFPS), Point(50, 60), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
        imshow("RightRectify", matRightBGR);

    // =======================================
    // if (0 != moOpenUVCCameraByPath(caCameraPath, &hCam)) {
    //     printf("Open Camera Failed\n");
    //     return -1;
    // }

    // uint16_t w = 0, h = 0;
    // moGetVideoResolution(hCam, &w, &h);
    // printf("Resolution: %d x %d\n", w, h);

    // // 設定 RAW 模式（Bayer + Bayer）
    // if (0 != moSetVideoMode(hCam, MVM_RAW)) {
    //     printf("moSetVideoMode(MVM_RAW) failed\n");
    //     return -1;
    // }

    // // 補光燈
    // moSetFilllightType(hCam, MFT_ON);

    // uint64_t u64FrameNum    = 0;
    // uint8_t* pu8FrameBuf    = NULL;
    // uint8_t* pu8Left        = NULL;
    // uint8_t* pu8Right       = NULL;
    // double   fps            = 0.0;
    // char     caFPS[24]      = {0};

    // // 每幀大小：16bit Bayer = width * height * 2 bytes
    // const uint32_t frameBytes = (uint32_t)w * h * 2;

    // printf("Publishing raw Bayer images... Press Ctrl+C to quit\n");
    // if (FOR_DEBUG){
    //     //建立Image檢視視窗
    //     namedWindow("Left Raw", WINDOW_NORMAL);
    //     namedWindow("Right Raw", WINDOW_NORMAL);
    // }

    // // 相機 20Hz，sleep 50ms 對齊，避免無謂的空轉浪費 CPU
    // while (ros::ok()) {
    //     ms_sleep(50);

    //     // 取得影像幀
    //     if (0 != moGetCurrentFrame(hCam, &u64FrameNum, &pu8FrameBuf)) {
    //         printf("moGetCurrentFrame failed\n");
    //         continue;
    //     }

    //     // FPS
    //     if (0 == moGetRealTimeFPS(hCam, &fps))
    //         snprintf(caFPS, sizeof(caFPS), "FPS: %.1f", fps);
    //     else
    //         snprintf(caFPS, sizeof(caFPS), "FPS: Calculating...");

    //     printf("\rFrameNum: %lu  %s", u64FrameNum, caFPS);
    //     fflush(stdout);

    //     // 取得左右 Bayer（文件 3.2.17 moGetRawImage）
    //     if (0 != moGetRawImage(hCam, pu8FrameBuf, &pu8Left, &pu8Right)) {
    //         printf("\nmoGetRawImage failed\n");
    //         continue;
    //     }

    //     ros::Time stamp = ros::Time::now();

    //     // 16bit Bayer → 8bit：normalize 自動對應實際值域到 0~255
    //     // 比 convertTo(1/256) 正確，不會因為相機只用 10/12bit 而過亮
    //     auto publishMono8 = [&](uint8_t* data, const std::string& frameId, ros::Publisher& pub, const std::string& winName) {
            
    //         // // Camera 原始資料：16-bit Bayer
    //         // Mat raw16(h, w, CV_16UC1, data); //Camera原始資料得到的是16-bit Bayer raw
            
    //         // // Bayer → 灰階 (仍然是16-bit)
    //         // Mat gray16;
    //         // cvtColor(raw16, gray16, COLOR_BayerGB2GRAY);


    //         // // 16-bit gray → 8-bit gray //因為calibration要求要8bit 灰階影像最標準, 所以要轉成gray 8-bit才publish
    //         // Mat gray8; 
    //         // gray16.convertTo(gray8, CV_8UC1, 1.0 / 256.0);
            
    //         //normalize(raw16, out8, 0, 255, NORM_MINMAX, CV_8UC1);
    //         std_msgs::Header hdr;
    //         hdr.stamp    = stamp;
    //         hdr.frame_id = frameId;

    //         // publish前換成gray-mono8
    //         pub.publish(cv_bridge::CvImage(hdr, "mono8", gray8).toImageMsg());
    //         if(FOR_DEBUG){
    //             imshow(winName, gray8);
    //             waitKey(1); 
    //         }
    //     };

        publishMono8(pu8Left,  "camera_left",  pubLeft, "Left Raw");
        publishMono8(pu8Right, "camera_right", pubRight, "Right Raw");

        ros::spinOnce();
    }

    moCloseCamera(&hCam);
    return 0;
}

