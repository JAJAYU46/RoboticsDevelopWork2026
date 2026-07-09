#include "common_function.h"

// Enable USE_OPENCV define in CMakelist.txt
#ifdef USE_OPENCV
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
#endif  // USE_OPENCV
void usage() {
    printf("Usage:\nOpen camera by videoIdx: ./get_rectify_sample [videoIdx]\n");
    printf("Locate camera automatically: ./get_rectify_sample\n\n");
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
#ifdef USE_OPENCV
        waitKey(FETCH_AND_DISPLAY_TIME_LENGTH);
        Mat matLeftBGR;
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
#endif  // USE_OPENCV
    }
    // Close camera
    moCloseCamera(&hCameraHandle);

    return n32Result;
}
