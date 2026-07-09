#include "common_function.h"

// Enable USE_OPENCV define in CMakelist.txt
#ifdef USE_OPENCV
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
#endif  // USE_OPENCV

void usage() {
    printf("Usage:\nOpen camera by videoIdx: ./get_rgbd_sample [videoIdx]\n");
    printf("Locate camera automatically: ./get_rgbd_sample\n\n");
}

int32_t main(int32_t argc, char** argv) {
    usage();
    printf("The version of SDK is %s\n", moGetSdkVersion());

    MO_CAMERA_HANDLE hCameraHandle = MO_INVALID_HANDLE;

    int32_t n32Result = 0;
    char caCameraPath[64];
    // Locate camera device name by running arguments
    if (0 != cameraSampleInit(argc, argv, caCameraPath)) {
        return -1;
    }

    // Open camera by path
    n32Result = moOpenUVCCameraByPath(caCameraPath, &hCameraHandle);
    if (0 != n32Result) {
        printf("Open Camera Failed %d\n", n32Result);
        return -1;
    }

    // Get video resolution
    uint16_t u16VideoFrameWidth = 0;
    uint16_t u16VideoFrameHeight = 0;
    n32Result = moGetVideoResolution(hCameraHandle, &u16VideoFrameWidth, &u16VideoFrameHeight);
    if (0 != n32Result) {
        printf("Get video resolution failed %d\n", n32Result);
        return -1;
    }

    uint64_t u64ImageFrameNum = 0;
    uint8_t* pu8FrameBuffer = NULL;
    uint16_t* pu16RGBDDisparityData = NULL;
    uint8_t* pu8RGBDYUVI420Img = NULL;
    double d8FPS = 0.0f;
    char caFPS[24] = {0};

    // Check video mode
    mo_video_mode eVideoMode;
    n32Result = moGetVideoMode(hCameraHandle, &eVideoMode);
    if (MVM_RGBD != eVideoMode) {
        // Set video mode to RGBD
        n32Result = moSetVideoMode(hCameraHandle, MVM_RGBD);
        if (0 != n32Result) {
            printf("Set video mode failed %d\n", n32Result);
            return -1;
        }
    }

    float fBxf = 0.0f;
    float fBase = 0.0f;

    // Get Bxf and Baseline
    n32Result = moGetBxfAndBase(hCameraHandle, &fBxf, &fBase);
    if (n32Result == 0) {
        printf("Bxf: %f,Baseline: %f\n", fBxf, fBase);
    } else {
        printf("Get camera bxf base failed %d\n", n32Result);
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

        // Get RGBD image
        n32Result = moGetRGBDImage(hCameraHandle, pu8FrameBuffer, &pu16RGBDDisparityData, &pu8RGBDYUVI420Img);

        // Get RGBD image with time stamp;
        /* uint64_t u64Timestamp;
        n32Result = moGetRGBDImageWithTimestamp(hCameraHandle, pu8FrameBuffer, &pu16RGBDDisparityData,
                                                &pu8RGBDYUVI420Img, &u64Timestamp);
        printf("Timestamp: %lu\n", u64Timestamp); */

#ifdef USE_OPENCV
        waitKey(FETCH_AND_DISPLAY_TIME_LENGTH);
        Mat matYUV2BGR;  // 12bits / 8bits
        Mat matYUVI420(u16VideoFrameHeight * 1.5, u16VideoFrameWidth, CV_8UC1, pu8RGBDYUVI420Img);
        cvtColor(matYUVI420, matYUV2BGR, COLOR_YUV2BGR_I420);
        // Mark fps at top left corner
        putText(matYUV2BGR, string(caFPS), Point(50, 60), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
        imshow("RGB", matYUV2BGR);

        Mat matDisparity(u16VideoFrameHeight, u16VideoFrameWidth, CV_8UC3);
        GetDisparityImage(matDisparity.rows, matDisparity.cols, matDisparity.step, pu16RGBDDisparityData,
                          matDisparity.data);
        // Mark fps at top left corner
        putText(matDisparity, string(caFPS), Point(50, 60), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
        imshow("Disparity", matDisparity);
#endif  // USE_OPENCV
    }

    // Close camera
    moCloseCamera(&hCameraHandle);

    return n32Result;
}
