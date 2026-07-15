#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>

#include "mo_stereo_camera_driver_c.h"

int32_t main(int32_t argc, char** argv) {
    printf("The version of SDK is %s\n", moGetSdkVersion());

    int32_t n32Result = 0;
    MO_CAMERA_HANDLE hCameraHandle = MO_INVALID_HANDLE;
    moCameraList* pStCamList;

    /**< 1. Find camera */
    n32Result = moQueryCameraList(&pStCamList);
    if (0 != n32Result) {
        printf("Query Camera List Failed.Return val %d\n", n32Result);
        return -1;
    }
    moPrintCameraList(pStCamList);

    uint32_t u32WhichIndex = 0;

    if (1 < pStCamList->size) {
        printf("Please select a camera by index and use enter key to confirm\n");
        scanf("%d", &u32WhichIndex);

        if (u32WhichIndex >= pStCamList->size) {
            printf("Selected index [%d] out of range\n", u32WhichIndex);
            return -1;
        }
    }
    printf("\nSelected %d, dev %s\n", u32WhichIndex, pStCamList->pCameraInfos[u32WhichIndex].devName);
    do {
        /**< 2. Open Camera */
        n32Result = moOpenUVCCameraByPath(pStCamList->pCameraInfos[u32WhichIndex].devName, &hCameraHandle);
        if (0 != n32Result) {
            printf("Open Camera Failed.Return val %d\n", n32Result);
            break;
        }
        mo_video_param* pstVideoParam = NULL;
        uint8_t u8ArraySize = 0;

        /**< 3. Query video frame param */
        n32Result = moQuerySupportedVideoParam(hCameraHandle, &pstVideoParam, &u8ArraySize);
        if (0 != n32Result) {
            printf("Query supported video para failed.Return val %d\n", n32Result);
            break;
        }
        printf("\nmoQuerySupportedVideoParam found [%d] resolutions parameters\n", u8ArraySize);

        for (uint8_t u8Idx = 0; u8Idx < u8ArraySize; u8Idx++) {
            printf("\tIndex: %d -- Width: %d, Height: %d, DefaultFps: %d\n", pstVideoParam[u8Idx].u8VideoParamIndex,
                   pstVideoParam[u8Idx].u16ResolutionWidth, pstVideoParam[u8Idx].u16ResolutionHeight,
                   pstVideoParam[u8Idx].u8DefaultFPS);
        }
        printf("\nPlease select the index of desired resolution and use enter key to confirm\n");

        uint32_t video_para_idx = 0;
        scanf("%d", &video_para_idx);
        if (video_para_idx < u8ArraySize) {
            printf("Choiced Index: %d \n", video_para_idx);
        } else {
            printf("Selected index [%d] out of range\n", video_para_idx);
            break;
        }

        /**< 4. set video param */
        n32Result = moSetVideoParam(hCameraHandle, video_para_idx);
        if (0 != n32Result) {
            printf("Set Video Param Failed.Return val %d\n", n32Result);
            break;
        }
        /**< 5. Get video resolution */
        uint16_t resolution_width = 0, resolution_height = 0;

        n32Result = moGetVideoResolution(hCameraHandle, &resolution_width, &resolution_height);
        if (0 != n32Result) {
            printf("Get video resolution Failed.Return val %d\n", n32Result);
            break;
        }
        printf("Current Video Resultion %u x %u\n", resolution_width, resolution_height);

        /* code */
    } while (0);

    /**Close camera and release camera list*/
    moCloseCamera(&hCameraHandle);
    moReleaseCameraList(&pStCamList);

    return n32Result;
}
