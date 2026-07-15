/**
 * @file get_multi_camera_sample.cpp.c
 * @author genghe-pc
 * @version 0.0.1
 * @date 06/17/2022 10:46:06
 */

/// headers
#include <unistd.h>  // usleep
#include <stdio.h>   // printf etc.
#include <stdlib.h>  // malloc, free etc.
#include <stdint.h>  // (u)intN_t
#include <string.h>  // memset
#include <pthread.h>
#include "mo_stereo_camera_driver_c.h"

#ifdef USE_OPENCV
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
#endif  // USE_OPENCV
moCameraList *pStCamList;
char cam1[] = "3010000001";  // Change to your camera id
char cam2[] = "3010000002";  // Change to your camera id

void *CameraProcess(void *arg) {
    char *sn = (char *)arg;
    printf("Processing sn:%s\n", sn);
    MO_CAMERA_HANDLE hCameraHandle = MO_INVALID_HANDLE;
    if (0 != moOpenUVCCameraBySN(pStCamList, sn, &hCameraHandle)) {
        printf("Open %s Failed\n", sn);
        return NULL;
    } else {
        printf("open %s success\n", sn);
    }
    uint64_t frameID = 0;
    uint8_t *p_data = NULL;
    uint16_t res_h = 0, res_w = 0;
    if (0 != moGetVideoResolution(hCameraHandle, &res_w, &res_h)) {
        printf("Warning:Get Cam[%s] Resolution resolution failed,try default 1280x720\n", sn);
        res_w = 1280;
        res_h = 720;
    } else {
        printf("Get Cam[%s] Resolution: %dx%d\n", sn, res_w, res_h);
    }
    while (1) {
        if (0 == moGetCurrentFrame(hCameraHandle, &frameID, &p_data)) {
            printf("Cam[%s] frameID %ld\n", sn, frameID);
        } else {
            continue;
        }
        uint16_t *pu16RGBDDisparityData = NULL;
        uint8_t *pu8RGBDYUVI420Img = NULL;
        if (0 != moGetRGBDImage(hCameraHandle, p_data, &pu16RGBDDisparityData, &pu8RGBDYUVI420Img)) {
            continue;
        }
#ifdef USE_OPENCV
        Mat matBGR;
        Mat matYUVI420(res_h * 1.5, res_w, CV_8UC1, pu8RGBDYUVI420Img);
        cvtColor(matYUVI420, matBGR, COLOR_YUV2BGR_I420);
        char windowname[128];
        sprintf(windowname, "%s Reference Img", sn);
        imshow(windowname, matBGR);
        char key = waitKey(10);
        if ('q' == key || 'Q' == key) {
            destroyWindow(windowname);
            break;
        }

#endif  // USE_OPENCV
    }
    printf("Closing cam %s\n", sn);
    moCloseCamera(&hCameraHandle);

    return NULL;
}

int main(int argc, char const *argv[]) {
    /* code */

    int32_t n32Result;
    printf("Current version:%s\n", moGetSdkVersion());
    n32Result = moQueryCameraList(&pStCamList);

    if (n32Result != 0) {
        printf("Query Camera Info Failed. ret[%d]\n", n32Result);
        return -1;
    }
    moPrintCameraList(pStCamList);
    printf("\n%s USB PATH %s\n", cam1, moGetCameraUsbPathNameBySn(pStCamList, cam1));
    printf("%s dev PATH %s\n", cam1, moGetCameraDeviceNameBySn(pStCamList, cam1));

    printf("\n%s USB PATH %s\n", cam2, moGetCameraUsbPathNameBySn(pStCamList, cam2));
    printf("%s dev PATH %s\n", cam2, moGetCameraDeviceNameBySn(pStCamList, cam2));

    pthread_t cam1_thread = -1;
    pthread_t cam2_thread = -1;

    pthread_create(&cam1_thread, NULL, CameraProcess, cam1);
    pthread_create(&cam2_thread, NULL, CameraProcess, cam2);
    pthread_join(cam1_thread, NULL);
    pthread_join(cam2_thread, NULL);

    moReleaseCameraList(&pStCamList);

    return 0;
}
