#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <sched.h>
#include <unistd.h>
#include <deque>
#include <vector>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <map>

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
// Globals
// ─────────────────────────────────────────────────────────────
static atomic<bool> g_bRunning(true);

static image_transport::Publisher* g_pubLeft  = nullptr;
static image_transport::Publisher* g_pubRight = nullptr;
static ros::Publisher* g_pubImu   = nullptr;

// ─────────────────────────────────────────────────────────────
// IMU producer-consumer queue
// Capture thread fills it; publish thread drains it.
// Keeps ROS serialization cost away from capture timing.
// ─────────────────────────────────────────────────────────────
static queue<sensor_msgs::Imu> g_imuMsgQueue;
static mutex                   g_imuMutex;
static condition_variable      g_imuCV;

// ─────────────────────────────────────────────────────────────
// 全域時間同步對齊（Image FrameNum -> Hardware Timestamp）
// ─────────────────────────────────────────────────────────────
static map<uint64_t, ros::Time> g_frameTimeMap;
static mutex                    g_timeMapMutex;
// 最新一筆 IMU 硬體時間戳，供 image thread fallback 使用
static ros::Time g_latestImuStamp;
static mutex     g_latestImuMutex;

// 輔助函式：由 IMU 執行緒寫入影像時間
static void setFrameTimestamp(uint64_t frameNum, const ros::Time& t) {
    lock_guard<mutex> lock(g_timeMapMutex);
    g_frameTimeMap[frameNum] = t;
    // 避免 map 無限制增長，保持最新 100 幀即可
    if (g_frameTimeMap.size() > 100) {
        g_frameTimeMap.erase(g_frameTimeMap.begin());
    }
}

// 輔助函式：由影像執行緒讀取時間，若還沒拿到則回傳 false
static bool getFrameTimestamp(uint64_t frameNum, ros::Time& t) {
    lock_guard<mutex> lock(g_timeMapMutex);
    auto it = g_frameTimeMap.find(frameNum);
    if (it != g_frameTimeMap.end()) {
        t = it->second;
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────
// Encode worker queue
// imageThreadFunc 只負責拿幀＋記時間戳，
// Bayer 轉換 / ROS publish / display 全部丟給 encodeWorkerThread
// 讓 CPU 密集操作遠離 IMU capture thread
// ─────────────────────────────────────────────────────────────
struct RawFrame {
    uint64_t  frameNum;
    ros::Time stamp;
    cv::Mat   leftRaw16;   // CV_16UC1 Bayer
    cv::Mat   rightRaw16;
    uint16_t  w, h;
    char      caFPS[32];
};
static queue<RawFrame>     g_encodeQueue;
static mutex               g_encodeMutex;
static condition_variable  g_encodeCV;

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

static void dispQueue_push(const cv::Mat& l, const cv::Mat& r) {
    pthread_mutex_lock(&g_dispQueue.mutex);
    g_dispQueue.latest.left  = l.clone();
    g_dispQueue.latest.right = r.clone();
    g_dispQueue.hasFrame = true;
    pthread_mutex_unlock(&g_dispQueue.mutex);
}
static bool dispQueue_pop(cv::Mat& l, cv::Mat& r) {
    pthread_mutex_lock(&g_dispQueue.mutex);
    bool had = g_dispQueue.hasFrame;
    if (had) {
        l = g_dispQueue.latest.left;
        r = g_dispQueue.latest.right;
        g_dispQueue.hasFrame = false;
    }
    pthread_mutex_unlock(&g_dispQueue.mutex);
    return had;
}

// ─────────────────────────────────────────────────────────────
// Helper: convert 16-bit Bayer → 8-bit BGR + 8-bit Gray
// ─────────────────────────────────────────────────────────────
static void bayerToGrayAndBGR(uint8_t* bayerData, uint16_t w, uint16_t h,
                               cv::Mat& outGray, cv::Mat& outBGR8)
{
    Mat bayer16(h, w, CV_16UC1, bayerData);
    Mat bgr16;
    cvtColor(bayer16, bgr16, COLOR_BayerGB2BGR_EA);
    bgr16.convertTo(outBGR8, CV_8UC3, 1.0 / 256.0);
    cvtColor(outBGR8, outGray, COLOR_BGR2GRAY);
}

// ─────────────────────────────────────────────────────────────
// ENCODE WORKER thread
// 從 g_encodeQueue 取出 raw frame，做 Bayer 轉換 + ROS publish + display
// ─────────────────────────────────────────────────────────────
void* encodeWorkerThread(void* /*arg*/) {
    while (g_bRunning.load()) {
        RawFrame frame;
        {
            unique_lock<mutex> lock(g_encodeMutex);
            g_encodeCV.wait(lock, [] {
                return !g_encodeQueue.empty() || !g_bRunning.load();
            });
            if (g_encodeQueue.empty()) continue;
            frame = move(g_encodeQueue.front());
            g_encodeQueue.pop();
        }

        // Bayer → Gray + BGR
        cv::Mat leftGray, leftBGR8, rightGray, rightBGR8;
        {
            cv::Mat bgr16;
            cv::cvtColor(frame.leftRaw16, bgr16, cv::COLOR_BayerGB2BGR_EA);
            bgr16.convertTo(leftBGR8, CV_8UC3, 1.0 / 256.0);
            cv::cvtColor(leftBGR8, leftGray, cv::COLOR_BGR2GRAY);
        }
        {
            cv::Mat bgr16;
            cv::cvtColor(frame.rightRaw16, bgr16, cv::COLOR_BayerGB2BGR_EA);
            bgr16.convertTo(rightBGR8, CV_8UC3, 1.0 / 256.0);
            cv::cvtColor(rightBGR8, rightGray, cv::COLOR_BGR2GRAY);
        }

        // ROS publish
        std_msgs::Header hdrL;
        hdrL.stamp    = frame.stamp;
        hdrL.frame_id = "camera_left";
        if (g_pubLeft)
            g_pubLeft->publish(cv_bridge::CvImage(hdrL, "mono8", leftGray).toImageMsg());

        std_msgs::Header hdrR;
        hdrR.stamp    = frame.stamp;
        hdrR.frame_id = "camera_right";
        if (g_pubRight)
            g_pubRight->publish(cv_bridge::CvImage(hdrR, "mono8", rightGray).toImageMsg());

        // Display
        // imshow 在 headless 板子上不可用且耗 CPU，已移除

        ROS_INFO_THROTTLE(2.0, "Frame %lu published with synced TS. %s",
                          frame.frameNum, frame.caFPS);
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────
// IMU CAPTURE thread — ~200 Hz
//
// 依靠 SDK 阻塞讀取硬體資料，維持硬體原生高精度頻率。
// 使用第一個 IMU 訊號建立 ROS 系統時間與硬體時間的 Offset 基準。
// ─────────────────────────────────────────────────────────────
void* imuCaptureThread(void* phCameraHandle) {
    MO_CAMERA_HANDLE hCam = *((MO_CAMERA_HANDLE*)phCameraHandle);
    mo_imu_data* pData   = nullptr;
    uint64_t     prevTs  = 0;

    uint64_t lastImageFrameNum = 0;

    while (g_bRunning.load() && ros::ok()) {
        // 阻塞等待 SDK 回傳（只有此模式下 u64ImageFrameNum 才有值）
        if (0 != moGetIMUData(hCam, &pData)) {
            ROS_WARN_THROTTLE(1.0, "moGetIMUData failed");
            usleep(1000);
            continue;
        }

        // 直接將硬體微秒時間戳轉為 ros::Time，不依賴 ROS wall clock
        // 硬體單位為微秒 (us)：秒 = us / 1e6，奈秒 = (us % 1e6) * 1000
        ros::Time stamp(
            static_cast<uint32_t>(pData->u64Timestamp / 1000000ULL),
            static_cast<uint32_t>((pData->u64Timestamp % 1000000ULL) * 1000ULL)
        );

        // 每包都更新最新 IMU 時間戳，供 image thread fallback 對齊
        {
            lock_guard<mutex> lk(g_latestImuMutex);
            g_latestImuStamp = stamp;
        }

        // 偵測影像 FrameNum 更新，記錄硬體曝光時間戳記（阻塞模式下才有值）
        if (pData->u64ImageFrameNum != 0 && pData->u64ImageFrameNum != lastImageFrameNum) {
            setFrameTimestamp(pData->u64ImageFrameNum, stamp);
            lastImageFrameNum = pData->u64ImageFrameNum;
        }

        // Gap detection
        if (prevTs != 0) {
            int64_t dtUs = (int64_t)pData->u64Timestamp - (int64_t)prevTs;
            if (dtUs > 11000)
                ROS_WARN("IMU gap: %.1f ms (frame %lu)", dtUs/1000.0, pData->u64ImageFrameNum);
        }
        prevTs = pData->u64Timestamp;

        sensor_msgs::Imu msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = "imu";
        msg.angular_velocity.x    = pData->dGyroX;
        msg.angular_velocity.y    = pData->dGyroY;
        msg.angular_velocity.z    = pData->dGyroZ;

        msg.linear_acceleration.x = pData->dAccelX;
        msg.linear_acceleration.y = pData->dAccelY;
        msg.linear_acceleration.z = pData->dAccelZ;

        
        msg.orientation_covariance[0]         = -1.0;
        msg.angular_velocity_covariance[0]    = -1.0;
        msg.linear_acceleration_covariance[0] = -1.0;

        // Push to publish queue
        {
            lock_guard<mutex> lock(g_imuMutex);
            g_imuMsgQueue.push(move(msg));
        }
        g_imuCV.notify_one();
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────
// IMU PUBLISH thread — drains queue and calls publish()
// ROS serialization is isolated from capture timing
// ─────────────────────────────────────────────────────────────
void* imuPublishThread(void* /*arg*/) {
    while (g_bRunning.load()) {
        unique_lock<mutex> lock(g_imuMutex);
        g_imuCV.wait(lock, [] {
            return !g_imuMsgQueue.empty() || !g_bRunning.load();
        });
        while (!g_imuMsgQueue.empty()) {
            sensor_msgs::Imu msg = move(g_imuMsgQueue.front());
            g_imuMsgQueue.pop();
            lock.unlock();
            if (g_pubImu) g_pubImu->publish(msg);
            lock.lock();
        }
    }
    // Flush remaining
    lock_guard<mutex> lock(g_imuMutex);
    while (!g_imuMsgQueue.empty()) {
        if (g_pubImu) g_pubImu->publish(g_imuMsgQueue.front());
        g_imuMsgQueue.pop();
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────
// IMAGE thread — publishes left/right images independently
// ─────────────────────────────────────────────────────────────
void* imageThreadFunc(void* phCameraHandle) {
    MO_CAMERA_HANDLE hCam = *((MO_CAMERA_HANDLE*)phCameraHandle);

    uint16_t w = 0, h = 0;
    if (0 != moGetVideoResolution(hCam, &w, &h)) {
        ROS_ERROR("moGetVideoResolution failed");
        g_bRunning.store(false);
        return nullptr;
    }
    ROS_INFO("Resolution: %u x %u", w, h);

    uint64_t u64FrameNum    = 0;
    uint8_t* pu8FrameBuf    = nullptr;
    uint8_t* pu8Left        = nullptr;
    uint8_t* pu8Right       = nullptr;
    double   fps            = 0.0;
    char     caFPS[32]      = "FPS: ...";

    while (g_bRunning.load() && ros::ok()) {
        ms_sleep(FETCH_AND_DISPLAY_TIME_LENGTH);

        // 1. 抓取影像影格
        if (0 != moGetCurrentFrame(hCam, &u64FrameNum, &pu8FrameBuf)) {
            ROS_WARN("moGetCurrentFrame failed");
            continue;
        }

        // 2. 核心對齊：依據 FrameNum 去 Map 撈取 IMU 幫我們記錄到的精確曝光時間
        ros::Time imgStamp;
        // IMU thread 是阻塞模式，可能比 image thread 慢幾毫秒才寫入 map
        // 等待最多 200ms（40 次 × 5ms），遠大於一個 IMU 週期（5ms）
        int retry = 0;
        while (!getFrameTimestamp(u64FrameNum, imgStamp) && retry < 40 && g_bRunning.load()) {
            usleep(5000);
            retry++;
        }

        if (retry >= 40) {
            ROS_WARN_THROTTLE(1.0, "Skipping frame %lu: No matching IMU timestamp found (waited 200ms)", u64FrameNum);
            continue;
        }

        // 3. 拆分 raw buffer（只做指標分割，不做轉碼）
        if (0 != moGetRawImage(hCam, pu8FrameBuf, &pu8Left, &pu8Right)) {
            ROS_WARN("moGetRawImage failed");
            continue;
        }

        // 4. 更新 FPS 字串
        if (0 == moGetRealTimeFPS(hCam, &fps))
            snprintf(caFPS, sizeof(caFPS), "FPS: %.1f", fps);

        // 5. 把 raw Bayer 資料 clone 到 queue，由 encodeWorkerThread 負責轉碼與發布
        //    imageThreadFunc 不再做任何 CPU 密集操作，避免擠壓 IMU capture thread
        {
            RawFrame rf;
            rf.frameNum   = u64FrameNum;
            rf.stamp      = imgStamp;
            rf.w          = w;
            rf.h          = h;
            rf.leftRaw16  = cv::Mat(h, w, CV_16UC1, pu8Left).clone();
            rf.rightRaw16 = cv::Mat(h, w, CV_16UC1, pu8Right).clone();
            snprintf(rf.caFPS, sizeof(rf.caFPS), "%s", caFPS);

            lock_guard<mutex> lock(g_encodeMutex);
            // queue 太深代表 encode 跟不上，丟掉最舊的避免 lag 累積
            if (g_encodeQueue.size() >= 3) {
                g_encodeQueue.pop();
                ROS_WARN_THROTTLE(2.0, "Encode queue full, dropping oldest frame");
            }
            g_encodeQueue.push(move(rf));
        }
        g_encodeCV.notify_one();
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────
// Helper: set RT priority
// ─────────────────────────────────────────────────────────────
static void setThreadRealtime(pthread_t thread, int priority) {
    struct sched_param param;
    param.sched_priority = priority;
    int ret = pthread_setschedparam(thread, SCHED_FIFO, &param);
    if (ret != 0)
        ROS_WARN("Could not set RT priority %d (errno=%d). Run with sudo or add rtprio to limits.conf", priority, ret);
    else
        ROS_INFO("IMU capture thread set to SCHED_FIFO priority %d", priority);
}

static int makeRTAttr(pthread_attr_t& attr, int priority) {
    pthread_attr_init(&attr);
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0) {
        pthread_attr_destroy(&attr); return -1;
    }
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO) != 0) {
        pthread_attr_destroy(&attr); return -1;
    }
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_attr_setschedparam(&attr, &param) != 0) {
        pthread_attr_destroy(&attr); return -1;
    }
    return 0;
}

// ─────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────
void usage() {
    printf("Usage:\n"
           "  ./raw_publisher [videoIdx]\n"
           "  ./raw_publisher   (auto-detect)\n\n");
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
    //if (0 != moSetVideoMode(hCam, MVM_RAW)) {
        //ROS_ERROR("moSetVideoMode(MVM_RAW) failed");
        //moCloseCamera(&hCam); return -2;
    //}

    ROS_INFO("mo_camera_node started. Topics:");
    ROS_INFO("  /mo_camera/left/image_raw   (~20 Hz)");
    ROS_INFO("  /mo_camera/right/image_raw  (~20 Hz)");
    ROS_INFO("  /mo_camera/imu              (~200 Hz)");

    ROS_INFO("Waiting for ROS time...");
    ros::Time::waitForValid();
    ros::Duration(0.5).sleep();
    ROS_INFO("ROS time ready: %.3f", ros::Time::now().toSec());

    // Start threads
    pthread_t imuCapThread, imuPubThread, imgThread, encWorkerThread;
    {
        pthread_attr_t rtAttr;
        bool attrOk = (makeRTAttr(rtAttr, 80) == 0);
        int ret = pthread_create(&imuCapThread, attrOk ? &rtAttr : nullptr,
                                 imuCaptureThread, &hCam);
        if (attrOk) pthread_attr_destroy(&rtAttr);

        if (ret != 0) {
            // RT 權限不足導致建立失敗，改用普通優先級重試
            ROS_WARN("pthread_create with RT priority failed (errno=%d), retrying without RT", ret);
            ret = pthread_create(&imuCapThread, nullptr, imuCaptureThread, &hCam);
            if (ret != 0) {
                ROS_FATAL("pthread_create(imuCapThread) failed: %d — aborting", ret);
                return -1;
            }
            ROS_WARN("imuCaptureThread running WITHOUT RT priority. "
                     "Consider: sudo setcap cap_sys_nice+ep <binary> or add rtprio to /etc/security/limits.conf");
        } else {
            ROS_INFO("imuCaptureThread created with SCHED_FIFO priority 80");
        }
    }
    pthread_create(&imuPubThread,    nullptr, imuPublishThread,   nullptr);
    pthread_create(&imgThread,       nullptr, imageThreadFunc,    &hCam);
    pthread_create(&encWorkerThread, nullptr, encodeWorkerThread, nullptr);

    // Display disabled — headless board

    // headless board — 不需要 display loop，直接等 ROS shutdown
    ros::spin();

    g_bRunning.store(false);
    g_imuCV.notify_all();
    g_encodeCV.notify_all();

    pthread_join(imuCapThread,   nullptr);
    pthread_join(imuPubThread,   nullptr);
    pthread_join(imgThread,      nullptr);
    pthread_join(encWorkerThread, nullptr);
    pthread_mutex_destroy(&g_dispQueue.mutex);

    moCloseCamera(&hCam);
    ROS_INFO("Node stopped.");
    return 0;
}
