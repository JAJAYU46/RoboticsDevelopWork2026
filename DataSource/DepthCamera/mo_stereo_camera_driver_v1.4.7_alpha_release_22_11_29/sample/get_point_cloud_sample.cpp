#include "common_function.h"
#include "mo_stereo_camera_driver_c_utilities.h"
void usage() {
    printf("Usage:\nOpen camera by videoIdx: ./get_point_cloud_sample [videoIdx]\n");
    printf("Locate camera automatically: ./get_point_cloud_sample\n\n");
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
    uint16_t* pu16RGBDDisparityData = NULL;
    uint8_t* pu8RGBDYUVI420Img = NULL;

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

    printf("Press ENTER to quit the running process\n");

    while (0 == getchar_nb(FETCH_AND_DISPLAY_TIME_LENGTH)) {
        // Get current video frame
        n32Result = moGetCurrentFrame(hCameraHandle, &u64ImageFrameNum, &pu8FrameBuffer);
        if (0 != n32Result) {
            printf("Get current frame failed.Try Again...\n");
            continue;
        }

        // Get RGBD Image
        n32Result = moGetRGBDImage(hCameraHandle, pu8FrameBuffer, &pu16RGBDDisparityData, &pu8RGBDYUVI420Img);
        if (0 != n32Result) {
            printf("Error: moGetRGBDImage return %d\n", n32Result);
            continue;
        }

        float* pfXAxisArray = NULL;
        float* pfYAxisArray = NULL;
        float* pfZAxisArray = NULL;
        uint32_t u32AxisArraySize = 0;

        // Get point cloud
        n32Result = moConvertDisparity2PointCloud(hCameraHandle, pu16RGBDDisparityData, &u32AxisArraySize,
                                                  &pfXAxisArray, &pfYAxisArray, &pfZAxisArray);
        if (0 != n32Result) {
            printf("Error: moConvertDisparity2PointCloud return %d\n", n32Result);
            return -6;
        }
        uint32_t show_points_limit = 50;
        if (show_points_limit < u32AxisArraySize) {
            printf("\nTotal Points %d\n", u32AxisArraySize);
            printf("Previous %d Points in [X,Y,Z]\n", show_points_limit);
            for (uint32_t u32Idx = 0; u32Idx < show_points_limit;  // u32AxisArraySize;
                 u32Idx++) {
                printf("[%f,%f,%f]\n", pfXAxisArray[u32Idx], pfYAxisArray[u32Idx], pfZAxisArray[u32Idx]);
            }

        } else {
            printf("Warning: Points is less than %d\n", show_points_limit);
        }
    }

    // Close camera
    moCloseCamera(&hCameraHandle);

    return n32Result;
}
