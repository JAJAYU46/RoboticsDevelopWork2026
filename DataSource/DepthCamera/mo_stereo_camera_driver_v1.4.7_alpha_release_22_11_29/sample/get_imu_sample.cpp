#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <pthread.h>

#include "common_function.h"

bool m_bIsRunnig = true;

void* getIMGProcess(void* phCameraHandle) {
    int32_t n32Result = 0;
    uint64_t u64ImageFrameNum = 0;
    uint8_t* pu8FrameBuffer = NULL;

    while (m_bIsRunnig) {
        ms_sleep(FETCH_AND_DISPLAY_TIME_LENGTH);

        /**< get current video frame */
        n32Result = moGetCurrentFrame(*((MO_CAMERA_HANDLE*)phCameraHandle), &u64ImageFrameNum, &pu8FrameBuffer);
        if (0 != n32Result) {
            printf("Error: moGetCurrentFrame return %d\n", n32Result);
            continue;
        }

        printf("\nImage>>>\nFrameNum: %lu\n", u64ImageFrameNum);
    }
    return NULL;
}

void* getIMUProcess(void* phCameraHandle) {
    int32_t n32Result = 0;
    mo_imu_data* pstIMUData = NULL;

    while (m_bIsRunnig) {
        ms_sleep(FETCH_IMU_TIME_LENGTH);

        /**< get current IMU data */
        n32Result = moGetIMUData(*((MO_CAMERA_HANDLE*)phCameraHandle), &pstIMUData);
        if (0 != n32Result) {
            printf("Error: moGetIMUData return %d\n", n32Result);
            continue;
        }

        printf(
            "\nIMU Data>>>\n"
            "FrameNum: %lu, Timestamp: %lu, Temperature: %lf\n"
            "Accel_X : %lf, Accel_Y  : %lf, Accel_Z    : %lf\n"
            "Gyro_X  : %lf, Gyro_Y   : %lf, Gyro_Z     : %lf\n",
            pstIMUData->u64ImageFrameNum, pstIMUData->u64Timestamp, pstIMUData->dTemperature, pstIMUData->dAccelX,
            pstIMUData->dAccelY, pstIMUData->dAccelZ, pstIMUData->dGyroX, pstIMUData->dGyroY, pstIMUData->dGyroZ);
    }
    return NULL;
}

void usage() {
    printf("Usage:\nOpen camera by videoIdx: ./get_imu_sample [videoIdx]\n");
    printf("Locate camera automatically: ./get_imu_sample\n\n");
}

int32_t main(int32_t argc, char** argv) {
    int32_t n32Result = 0;
    MO_CAMERA_HANDLE hCameraHandle = MO_INVALID_HANDLE;
    usage();
    printf("The version of SDK is %s\n", moGetSdkVersion());

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

    pthread_t img_thread = -1;
    pthread_t imu_thread = -1;
    pthread_create(&img_thread, NULL, getIMGProcess, &hCameraHandle);
    pthread_create(&imu_thread, NULL, getIMUProcess, &hCameraHandle);

    /**< Get IMU data */
    printf("Pressing enter key quits the running process\n");

    getchar();

    m_bIsRunnig = false;

    pthread_join(img_thread, NULL);
    pthread_join(imu_thread, NULL);

    /**< Close specific camera */
    moCloseCamera(&hCameraHandle);

    return n32Result;
}
