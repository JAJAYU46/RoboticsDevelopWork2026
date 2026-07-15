#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <deque>
#include <vector>

#include "common_function.h"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

// ─────────────────────────────────────────────────────────────
// IMU queue
//
// 設計原則：
//   - IMU thread 把每一筆 IMU 都 push 進來，同時直接發布到 ROS
//   - Image thread 用 frameNum 找出對應的第一筆 IMU 時間戳
//     作為影像的 header.stamp，然後把這批資料從 queue 清掉
// ─────────────────────────────────────────────────────────────
#define IMU_QUEUE_MAX 2000  // 200Hz * 10s，足夠 buffer

struct IMUEntry {
    mo_imu_data data;
    ros::Time   rosTime;
};

struct IMUQueue {
    pthread_mutex_t      mutex;
    std::deque<IMUEntry> q;
};
static IMUQueue      g_imuQueue = { PTHREAD_MUTEX_INITIALIZER, {} };
static volatile bool g_bRunning = true;

static void imuQueue_push(const mo_imu_data& d, const ros::Time& t) {
    pthread_mutex_lock(&g_imuQueue.mutex);
    g_imuQueue.q.push_back({d, t});
    if (g_imuQueue.q.size() > IMU_QUEUE_MAX)
        g_imuQueue.q.pop_front();
    pthread_mutex_unlock(&g_imuQueue.mutex);
}

// 取出 targetFrame 對應的所有 IMU entry，並清除更早的過期資料
// 回傳值：該 frame 的所有 IMU entries（已從 queue 移除）
static std::vector<IMUEntry> imuQueue_popByFrame(uint64_t targetFrame) {
    std::vector<IMUEntry> result;
    pthread_mutex_lock(&g_imuQueue.mutex);
    auto it = g_imuQueue.q.begin();
    while (it != g_imuQueue.q.end()) {
        if (it->data.u64ImageFrameNum < targetFrame) {
            // 過期資料，丟棄
            it = g_imuQueue.q.erase(it);
        } else if (it->data.u64ImageFrameNum == targetFrame) {
            result.push_back(*it);
            it = g_imuQueue.q.erase(it);
        } else {
            // frameNum > targetFrame，屬於下一幀，停止
            break;
        }
    }
    pthread_mutex_unlock(&g_imuQueue.mutex);
    return result;
}

// ─────────────────────────────────────────────────────────────
// Display queue (imshow must run in main thread)
// ─────────────────────────────────────────────────────────────
struct DisplayFrame { cv::Mat left; cv::Mat right; };
struct DisplayQueue {
    pthread_mutex_t mutex;
    DisplayFrame    latest;
    bool            hasFrame;
};
static DisplayQueue g_dispQueue = { PTHREAD_MUTEX_INITIALIZER, {}, false };

static void dispQueue_push(const cv::Mat& left, const cv::Mat& right) {
    pthread_mutex_lock(&g_dispQueue.mutex);
    g_dispQueue.latest.left  = left.clone();
    g_dispQueue.latest.right = right.clone();
    g_dispQueue.hasFrame = true;
    pthread_mutex_unlock(&g_dispQueue.mutex);
}

static bool dispQueue_pop(cv::Mat& left, cv::Mat& right) {
    pthread_mutex_lock(&g_dispQueue.mutex);
    bool had = g_dispQueue.hasFrame;
    if (had) {
        left  = g_dispQueue.latest.left;
        right = g_dispQueue.latest.right;
        g_dispQueue.hasFrame = false;
    }
    pthread_mutex_unlock(&g_dispQueue.mutex);
    return had;
}

// ─────────────────────────────────────────────────────────────
// ROS publishers
// ─────────────────────────────────────────────────────────────
static image_transport::Publisher* g_pubLeft  = nullptr;
static image_transport::Publisher* g_pubRight = nullptr;
static ros::Publisher*             g_pubImu   = nullptr;

// ─────────────────────────────────────────────────────────────
// Helper: convert 16bit Bayer → 8bit BGR + 8bit Gray
//
// The camera outputs 16-bit Bayer. After debayer the result is
// still 16-bit (CV_16UC3). We must divide by 256 to get 8-bit.
// ─────────────────────────────────────────────────────────────
static void bayerToGrayAndBGR(uint8_t* bayerData,
                               uint16_t w, uint16_t h,
                               cv::Mat& outGray,   // CV_8UC1 for publishing
                               cv::Mat& outBGR8)   // CV_8UC3 for display
{
    Mat bayer16(h, w, CV_16UC1, bayerData);
    Mat bgr16;
    cvtColor(bayer16, bgr16, COLOR_BayerGB2BGR_EA);  // still 16-bit
    bgr16.convertTo(outBGR8, CV_8UC3, 1.0 / 256.0); // → 8-bit
    cvtColor(outBGR8, outGray, COLOR_BGR2GRAY);      // → mono8
}

// ─────────────────────────────────────────────────────────────
// IMU thread
//
// 每一筆 IMU 資料做兩件事：
//   1. 直接發布到 /mo_camera/imu（維持 200 Hz 完整輸出）
//   2. push 進 queue 供 image thread 查詢對應時間戳
// ─────────────────────────────────────────────────────────────
void* imuThreadFunc(void* phCameraHandle) {
    MO_CAMERA_HANDLE hCam = *((MO_CAMERA_HANDLE*)phCameraHandle);
    mo_imu_data* pData = NULL;

    while (g_bRunning && ros::ok()) {
        ms_sleep(FETCH_IMU_TIME_LENGTH);
        if (0 != moGetIMUData(hCam, &pData)) continue;

        // 用硬體時間戳（微秒）換算成 ros::Time，避免系統時間抖動
        ros::Time t;
        t.fromNSec(pData->u64Timestamp * 1000ULL);  // us → ns

        // 1. 直接發布（完整 200 Hz，硬體時間戳）
        sensor_msgs::Imu imuMsg;
        imuMsg.header.stamp    = t;
        imuMsg.header.frame_id = "imu";
        imuMsg.linear_acceleration.x = pData->dAccelX;
        imuMsg.linear_acceleration.y = pData->dAccelY;
        imuMsg.linear_acceleration.z = pData->dAccelZ;
        imuMsg.angular_velocity.x    = pData->dGyroX;
        imuMsg.angular_velocity.y    = pData->dGyroY;
        imuMsg.angular_velocity.z    = pData->dGyroZ;
        imuMsg.orientation_covariance[0] = -1;
        imuMsg.linear_acceleration_covariance.fill(0);
        imuMsg.angular_velocity_covariance.fill(0);
        g_pubImu->publish(imuMsg);

        // 2. 同時存入 queue，供 image thread 查對應的硬體時間戳
        imuQueue_push(*pData, t);
    }
    return NULL;
}

// ─────────────────────────────────────────────────────────────
// Image thread
// ─────────────────────────────────────────────────────────────
struct FrameStash {
    uint64_t  frameNum;
    uint8_t*  left;
    uint8_t*  right;
    ros::Time rosTime;
    bool      valid;
};

void* imageThreadFunc(void* phCameraHandle) {
    MO_CAMERA_HANDLE hCam = *((MO_CAMERA_HANDLE*)phCameraHandle);

    uint16_t w = 0, h = 0;
    if (0 != moGetVideoResolution(hCam, &w, &h)) {
        ROS_ERROR("moGetVideoResolution failed");
        g_bRunning = false;
        return NULL;
    }
    ROS_INFO("Resolution: %u x %u", w, h);

    FrameStash prev = {0, NULL, NULL, ros::Time(0), false};

    while (g_bRunning && ros::ok()) {
        ms_sleep(FETCH_AND_DISPLAY_TIME_LENGTH);

        uint64_t curFrameNum = 0;
        uint8_t* pu8FrameBuf = NULL;
        uint8_t* pu8Left     = NULL;
        uint8_t* pu8Right    = NULL;

        // 1. Grab frame（影像時間戳將從對應 IMU 硬體時間戳取得，不用 now()）
        if (0 != moGetCurrentFrame(hCam, &curFrameNum, &pu8FrameBuf)) {
            ROS_WARN("moGetCurrentFrame failed");
            continue;
        }
        ros::Time imgTime;  // 將由 IMU 硬體時間戳填入

        // 2. Split Bayer
        if (0 != moGetRawImage(hCam, pu8FrameBuf, &pu8Left, &pu8Right)) {
            ROS_WARN("moGetRawImage failed");
            continue;
        }

        // 3. New frame → prev frame 的 IMU 已全部到齊，可以處理
        if (prev.valid && curFrameNum != prev.frameNum) {

            std::vector<IMUEntry> imuEntries = imuQueue_popByFrame(prev.frameNum);

            if (imuEntries.empty()) {
                ROS_WARN("Frame %lu: no IMU samples, skipping publish", prev.frameNum);
            } else {
                // ── 用第一筆 IMU 的時間戳作為影像時間戳 ────────────
                // 理由：影像本身沒有硬體時間戳，IMU 的第一筆是
                // 最接近曝光開始的時刻，作為影像時間戳最合理。
                // kalibr / VINS-Fusion 會透過 timeshift 參數補償剩餘偏差。
                ros::Time imgStamp = imuEntries.front().rosTime;

                // ── Convert images ──────────────────────────────────
                Mat leftGray, leftBGR8;
                Mat rightGray, rightBGR8;
                bayerToGrayAndBGR(prev.left,  w, h, leftGray,  leftBGR8);
                bayerToGrayAndBGR(prev.right, w, h, rightGray, rightBGR8);

                // ── Publish images（兩張用同一時間戳，kalibr 要求）──
                std_msgs::Header hdrL;
                hdrL.stamp    = imgStamp;
                hdrL.frame_id = "camera_left";
                g_pubLeft->publish(
                    cv_bridge::CvImage(hdrL, "mono8", leftGray).toImageMsg());

                std_msgs::Header hdrR;
                hdrR.stamp    = imgStamp;
                hdrR.frame_id = "camera_right";
                g_pubRight->publish(
                    cv_bridge::CvImage(hdrR, "mono8", rightGray).toImageMsg());

                // ── IMU 已在 imuThreadFunc 全部發布，這裡不再重複 ──

                // ── Display ─────────────────────────────────────────
                double fps = 0.0;
                char caFPS[32] = "FPS: ...";
                if (0 == moGetRealTimeFPS(hCam, &fps))
                    snprintf(caFPS, sizeof(caFPS), "FPS: %.1f", fps);
                putText(leftBGR8,  string(caFPS), Point(50, 60),
                        FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0));
                putText(rightBGR8, string(caFPS), Point(50, 60),
                        FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0));
                dispQueue_push(leftBGR8, rightBGR8);

                ROS_INFO_THROTTLE(1.0,
                    "Frame %lu | img_stamp=%.6f | IMU samples=%zu (first=%.6f last=%.6f)",
                    prev.frameNum,
                    imgStamp.toSec(),
                    imuEntries.size(),
                    imuEntries.front().rosTime.toSec(),
                    imuEntries.back().rosTime.toSec());
            }
        }

        // 4. Stash current frame
        // imgTime 未使用（影像時間戳完全來自 IMU 硬體時間戳）
        prev.frameNum = curFrameNum;
        prev.left     = pu8Left;
        prev.right    = pu8Right;
        prev.rosTime  = imgTime;  // unused，保留供 debug
        prev.valid    = true;
    }

    return NULL;
}

// ─────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────
void usage() {
    printf("Usage:\n"
           "  ./mo_camera_node [videoIdx]\n"
           "  ./mo_camera_node   (auto-detect)\n\n");
}

int main(int argc, char** argv) {
    usage();

    ros::init(argc, argv, "mo_camera_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_transport::Publisher pubLeft  = it.advertise("/mo_camera/left/image_raw",  1);
    image_transport::Publisher pubRight = it.advertise("/mo_camera/right/image_raw", 1);
    ros::Publisher             pubImu  = nh.advertise<sensor_msgs::Imu>("/mo_camera/imu", 200);

    g_pubLeft  = &pubLeft;
    g_pubRight = &pubRight;
    g_pubImu   = &pubImu;

    printf("SDK version: %s\n", moGetSdkVersion());

    MO_CAMERA_HANDLE hCam = MO_INVALID_HANDLE;
    char caCameraPath[64];

    if (0 != cameraSampleInit(argc, argv, caCameraPath)) {
        ROS_ERROR("cameraSampleInit failed"); return -1;
    }
    if (0 != moOpenUVCCameraByPath(caCameraPath, &hCam)) {
        ROS_ERROR("Failed to open camera"); return -1;
    }
    if (0 != moSetVideoMode(hCam, MVM_RAW)) {
        ROS_ERROR("moSetVideoMode(MVM_RAW) failed");
        moCloseCamera(&hCam); return -2;
    }

    ROS_INFO("mo_camera_node started. Topics:");
    ROS_INFO("  /mo_camera/left/image_raw");
    ROS_INFO("  /mo_camera/right/image_raw");
    ROS_INFO("  /mo_camera/imu");

    pthread_t imuThread, imgThread;
    pthread_create(&imuThread, NULL, imuThreadFunc,   &hCam);
    pthread_create(&imgThread, NULL, imageThreadFunc, &hCam);

    cv::namedWindow("Left",  cv::WINDOW_NORMAL);
    cv::namedWindow("Right", cv::WINDOW_NORMAL);

    ros::Rate rate(60);
    while (ros::ok()) {
        ros::spinOnce();

        cv::Mat left, right;
        if (dispQueue_pop(left, right)) {
            cv::imshow("Left",  left);
            cv::imshow("Right", right);
        }

        if (cv::waitKey(1) == 27) {
            ros::shutdown();
            break;
        }
        rate.sleep();
    }

    cv::destroyAllWindows();
    g_bRunning = false;
    pthread_join(imgThread, NULL);
    pthread_join(imuThread, NULL);
    pthread_mutex_destroy(&g_imuQueue.mutex);
    pthread_mutex_destroy(&g_dispQueue.mutex);

    moCloseCamera(&hCam);
    return 0;
}
