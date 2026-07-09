#include "common_function.h"

// Enable USE_OPENCV define in CMakelist.txt
#ifdef USE_OPENCV
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
#endif  // USE_OPENCV

void usage() {
    printf("Usage:\nOpen camera by videoIdx: ./get_dense_rgbd_sample [videoIdx]\n");
    printf("Locate camera automatically: ./get_dense_rgbd_sample\n\n");
}
int32_t main(int32_t argc, char** argv) {
    usage();
    printf("The version of SDK is %s\n", moGetSdkVersion());
    MO_CAMERA_HANDLE hCameraHandle = MO_INVALID_HANDLE;

    int32_t n32Result = 0;
    char caCameraPath[64];
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

    // Set video mode to RGBD_DENSE mode
    n32Result = moSetVideoMode(hCameraHandle, MVM_RGBD_DENSE);
    if (0 != n32Result) {
        printf("Error: moSetVideoMode failed %d\n", n32Result);
        return -4;
    }
    mo_video_mode eVideoMode;

    moGetVideoMode(hCameraHandle, &eVideoMode);
    printf("Current Video Mode %d\n", eVideoMode);

    uint64_t u64ImageFrameNum = 0;
    uint8_t* pu8FrameBuffer = NULL;
    uint16_t* pu16RGBDDisparityData = NULL;
    uint8_t* pu8RGBDYUVI420Img = NULL;

    printf("Press ENTER to quit the running process\n");

    while (0 == getchar_nb(FETCH_AND_DISPLAY_TIME_LENGTH)) {
        // Get current video frame
        n32Result = moGetCurrentFrame(hCameraHandle, &u64ImageFrameNum, &pu8FrameBuffer);
        if (0 != n32Result) {
            printf("Error: moGetCurrentFrame return %d\n", n32Result);
            return -5;
        }

        // Get rgbd image (The disparity data is now dense disparity data)
        n32Result = moGetRGBDImage(hCameraHandle, pu8FrameBuffer, &pu16RGBDDisparityData, &pu8RGBDYUVI420Img);
        if (0 != n32Result) {
            printf("Error: moGetRGBDImage return %d\n", n32Result);
            return -6;
        }

#ifdef USE_OPENCV
        char key = waitKey(FETCH_AND_DISPLAY_TIME_LENGTH);
        if (key == 'q') {
            moSetVideoMode(hCameraHandle, MVM_RGBD);
        } else if (key == 'w') {
            moSetVideoMode(hCameraHandle, MVM_RGBD_DENSE);
        } else if (key == 'e') {
            moSetVideoMode(hCameraHandle, MVM_RGBD_DENOISE);
        }
        Mat matYUV2BGR;  // 12bits / 8bits
        Mat matYUVI420(u16VideoFrameHeight * 1.5, u16VideoFrameWidth, CV_8UC1, pu8RGBDYUVI420Img);
        cvtColor(matYUVI420, matYUV2BGR, COLOR_YUV2BGR_I420);
        imshow("RGB", matYUV2BGR);

        Mat matDisparity(u16VideoFrameHeight, u16VideoFrameWidth, CV_8UC3);
        GetDisparityImage(matDisparity.rows, matDisparity.cols, matDisparity.step, pu16RGBDDisparityData,
                          matDisparity.data);
        imshow("Disparity", matDisparity);
#endif  // USE_OPENCV
    }

    // Close camera
    moCloseCamera(&hCameraHandle);

    return n32Result;
}
