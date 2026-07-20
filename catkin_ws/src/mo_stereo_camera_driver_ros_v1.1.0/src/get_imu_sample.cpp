#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <map>
#include <set>
#include <queue>
#include <vector>

#include "common_function.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/make_shared.hpp>

// OpenCV only for 16→8 bit conversion (no imencode, very fast)
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>   // cvtColor, putText
#include <opencv2/highgui.hpp>   // imshow, waitKey, namedWindow

// =============================================================================
// Global
// =============================================================================
bool m_bIsRunnig = true;
bool g_bEnableDisplay = false;  // 是否显示左右 rectify 画面（无显示器/SSH 环境请设为 false）

ros::Publisher g_pubLeftImage;
ros::Publisher g_pubRightImage;
ros::Publisher g_pubImu;

uint16_t g_u16Width  = 0;
uint16_t g_u16Height = 0;

// =============================================================================
// IMU record  (u64Timestamp unit: µs)
// =============================================================================
struct IMURecord {
    uint64_t u64Timestamp;
    uint64_t u64FirstTimestamp;
    double   dTemperature;
    double   dAccelX, dAccelY, dAccelZ;
    double   dGyroX,  dGyroY,  dGyroZ;
};

pthread_mutex_t               g_mapMutex = PTHREAD_MUTEX_INITIALIZER;
std::map<uint64_t, IMURecord> g_imuMap;
std::set<uint64_t>            g_pendingFrames;
const size_t                  MAX_IMU_MAP_SIZE = 200;

// td statistics
uint64_t        g_lastFrameFirstIMUTs = 0;
double          g_tdSum               = 0.0;
uint64_t        g_tdCount             = 0;
uint64_t        g_dropCount           = 0;
pthread_mutex_t g_tdMutex             = PTHREAD_MUTEX_INITIALIZER;

// =============================================================================
// Publish queue: image thread enqueues rectified 8-bit Y-plane copy,
// publish thread publishes directly (no bit-shift conversion needed).
// Keeping the queue small (MAX=2) ensures we always publish the latest frame.
// =============================================================================
struct ImageFrame {
    ros::Time            stamp;
    uint64_t             frameNum;
    std::vector<uint8_t> leftData;    // rectified left Y-only, 8-bit, width*height bytes
    std::vector<uint8_t> rightData;   // rectified right Y-plane (extracted from I420), 8-bit, width*height bytes
};

std::queue<ImageFrame> g_pubQueue;
const size_t           MAX_PUB_QUEUE = 2;
pthread_mutex_t        g_pubMutex    = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t         g_pubCond     = PTHREAD_COND_INITIALIZER;

// IMU publish queue: SDK fetch thread enqueues, IMU publish thread dequeues.
// Decouples ms_sleep/SDK latency from ROS publish timing.
struct ImuSample {
    ros::Time stamp;
    double ax, ay, az;
    double gx, gy, gz;
};

std::queue<ImuSample> g_imuPubQueue;
const size_t          MAX_IMU_PUB_QUEUE = 50;   // 50 samples = 250ms buffer
pthread_mutex_t       g_imuPubMutex     = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t        g_imuPubCond      = PTHREAD_COND_INITIALIZER;

// =============================================================================
// Helper: µs hardware timestamp → ros::Time
// =============================================================================
static ros::Time imuTsToRosTime(uint64_t us) {
    static bool      inited  = false;
    static ros::Time wallRef;
    static uint64_t  imuRef  = 0;
    if (!inited) {
        wallRef = ros::Time::now();
        imuRef  = us;
        inited  = true;
    }
    return wallRef + ros::Duration((double)((int64_t)(us - imuRef)) / 1e6);
}

// =============================================================================
// Helper: fill and publish MONO8 Image message (shared_ptr, zero extra copy)
// =============================================================================
static void publishMono8(ros::Publisher& pub,
                         std::vector<uint8_t>& data8,
                         ros::Time stamp,
                         const char* frame_id) {
    auto msg = boost::make_shared<sensor_msgs::Image>();
    msg->header.stamp    = stamp;
    msg->header.frame_id = frame_id;
    msg->height          = g_u16Height;
    msg->width           = g_u16Width;
    msg->encoding        = sensor_msgs::image_encodings::MONO8;
    msg->is_bigendian    = 0;
    msg->step            = g_u16Width;
    msg->data            = std::move(data8);   // zero-copy move into message
    pub.publish(msg);
}

// =============================================================================
// Publish thread: dequeue → publish (data already 8-bit, no conversion needed)
// =============================================================================
void* publishThread(void*) {
    while (m_bIsRunnig) {
        pthread_mutex_lock(&g_pubMutex);
        while (g_pubQueue.empty() && m_bIsRunnig)
            pthread_cond_wait(&g_pubCond, &g_pubMutex);
        if (!m_bIsRunnig) { pthread_mutex_unlock(&g_pubMutex); break; }

        ImageFrame frame = std::move(g_pubQueue.front());
        g_pubQueue.pop();
        pthread_mutex_unlock(&g_pubMutex);

        if (g_u16Width == 0) continue;

        // 影像在 getIMGProcess 阶段已经是 rectify 后的 8-bit Y 数据，直接发布即可
        publishMono8(g_pubLeftImage,  frame.leftData,  frame.stamp, "mo_camera_left");
        publishMono8(g_pubRightImage, frame.rightData, frame.stamp, "mo_camera_right");

        printf("[PUB]    FrameNum: %lu  stamp: %.6f\n",
               frame.frameNum, frame.stamp.toSec());
    }
    return NULL;
}

// =============================================================================
// Helper: td update + drop detection
// =============================================================================
static void updateTd(const IMURecord& rec) {
    pthread_mutex_lock(&g_tdMutex);
    if (g_lastFrameFirstIMUTs != 0) {
        double td_ms = (double)(rec.u64FirstTimestamp - g_lastFrameFirstIMUTs) / 1000.0;
        if (td_ms > 75.0) {
            g_dropCount++;
            ROS_WARN("Dropped frame! interval=%.1f ms (total=%lu)", td_ms, g_dropCount);
        }
        g_tdSum += td_ms;
        g_tdCount++;
    }
    g_lastFrameFirstIMUTs = rec.u64FirstTimestamp;
    pthread_mutex_unlock(&g_tdMutex);
}

// =============================================================================
// Helper: copy rectified 8-bit Y data into queue for publish thread
// pu8Left  : rectified left image, Y-only, width*height bytes
// pu8Right : rectified right image, YUV I420, width*height*1.5 bytes
//            → 只取最前面的 Y plane（width*height bytes）用于 MONO8 发布
// =============================================================================
static void enqueueFrame(uint64_t frameNum, const IMURecord& rec,
                         uint8_t* pu8Left, uint8_t* pu8Right) {
    if (!pu8Left || !pu8Right || g_u16Width == 0) return;

    size_t yBytes = (size_t)g_u16Width * g_u16Height;  // 8-bit Y plane 大小，左右图相同

    ImageFrame frame;
    frame.stamp    = imuTsToRosTime(rec.u64FirstTimestamp);
    frame.frameNum = frameNum;
    frame.leftData.assign(pu8Left,  pu8Left  + yBytes);
    frame.rightData.assign(pu8Right, pu8Right + yBytes);  // 只取 I420 的 Y plane，跳过 U/V

    pthread_mutex_lock(&g_pubMutex);
    // Drop oldest if publish thread is lagging — always keep latest frame
    while (g_pubQueue.size() >= MAX_PUB_QUEUE) {
        g_pubQueue.pop();
        ROS_WARN_THROTTLE(1.0, "Pub queue full, dropping old frame");
    }
    g_pubQueue.push(std::move(frame));
    pthread_cond_signal(&g_pubCond);
    pthread_mutex_unlock(&g_pubMutex);
}

// =============================================================================
// Image thread: poll → split → enqueue → display
// =============================================================================
void* getIMGProcess(void* phCameraHandle) {
    int32_t  n32Result        = 0;
    uint64_t u64ImageFrameNum = 0;
    uint64_t u64PrevFrameNum  = (uint64_t)-1;
    uint8_t* pu8FrameBuffer   = NULL;
    uint8_t* pu8OnlyY         = NULL;  // 左图：rectify 后仅 Y 分量
    uint8_t* pu8RectifiedRightYUVI420Img = NULL;  // 右图：rectify 后 YUV I420
    double   d8FPS            = 0.0;
    char     caFPS[24]        = {0};

    MO_CAMERA_HANDLE hCameraHandle = *((MO_CAMERA_HANDLE*)phCameraHandle);

    // 切换到 RECTIFIED 模式（对应 get_rectify_sample.cpp 中的做法）
    n32Result = moSetVideoMode(hCameraHandle, MVM_RECTIFIED);
    if (0 != n32Result) {
        printf("Error: moSetVideoMode failed %d\n", n32Result);
        return NULL;
    }

    // Set fill light once before entering the polling loop.
    n32Result = moSetFilllightType(hCameraHandle, MFT_OFF);
    if (0 != n32Result) {
        printf("Warning: moSetFilllightType failed %d (fill light may not be supported)\n", n32Result);
    } else {
        printf("Fill light turned off.\n");
    }

    // 建立显示窗口
    if (g_bEnableDisplay) {
        cv::namedWindow("LeftRectify",  cv::WINDOW_AUTOSIZE);
        cv::namedWindow("RightRectify", cv::WINDOW_AUTOSIZE);
    }

    while (m_bIsRunnig) {
        n32Result = moGetCurrentFrame(hCameraHandle, &u64ImageFrameNum, &pu8FrameBuffer);
        if (0 != n32Result) { ms_sleep(1); continue; }
        if (u64ImageFrameNum == u64PrevFrameNum) { ms_sleep(1); continue; }
        u64PrevFrameNum = u64ImageFrameNum;

        // 取得 rectify 后的左右影像（左：Y-only，右：YUV I420）
        n32Result = moGetRectifiedImage(hCameraHandle, pu8FrameBuffer,
                                         &pu8OnlyY, &pu8RectifiedRightYUVI420Img);
        if (0 != n32Result) {
            printf("Error: moGetRectifiedImage return %d\n", n32Result);
            continue;
        }

        // ---- 显示画面 ----
        if (g_bEnableDisplay) {
            // 取得即时 FPS
            n32Result = moGetRealTimeFPS(hCameraHandle, &d8FPS);
            if (0 != n32Result) {
                sprintf(caFPS, "FPS: Calculating...");
            } else {
                sprintf(caFPS, "FPS: %.2f", d8FPS);
            }

            if (NULL != pu8OnlyY) {
                cv::Mat matLeftBGR;
                cv::Mat matLeftRectify(g_u16Height, g_u16Width, CV_8UC1, pu8OnlyY);
                cv::cvtColor(matLeftRectify, matLeftBGR, cv::COLOR_GRAY2BGR);
                cv::putText(matLeftBGR, std::string(caFPS), cv::Point(50, 60),
                            cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));
                cv::imshow("LeftRectify", matLeftBGR);
            }

            if (NULL != pu8RectifiedRightYUVI420Img) {
                cv::Mat matRightBGR;  // 1.5 = 12bits / 8bits (I420)
                cv::Mat matRightRectify(g_u16Height * 1.5, g_u16Width, CV_8UC1,
                                         pu8RectifiedRightYUVI420Img);
                cv::cvtColor(matRightRectify, matRightBGR, cv::COLOR_YUV2BGR_I420);
                cv::putText(matRightBGR, std::string(caFPS), cv::Point(50, 60),
                            cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));
                cv::imshow("RightRectify", matRightBGR);
            }
            cv::waitKey(1);  // 让窗口刷新，1ms 非阻塞
        }
        // ---- 显示画面结束 ----

        pthread_mutex_lock(&g_mapMutex);
        auto it = g_imuMap.find(u64ImageFrameNum);
        if (it != g_imuMap.end()) {
            updateTd(it->second);
            enqueueFrame(u64ImageFrameNum, it->second,
                         pu8OnlyY, pu8RectifiedRightYUVI420Img);
        } else {
            g_pendingFrames.insert(u64ImageFrameNum);
            printf("\n[IMAGE]  FrameNum: %lu  (IMU queued)\n", u64ImageFrameNum);
        }
        pthread_mutex_unlock(&g_mapMutex);
    }

    if (g_bEnableDisplay) {
        cv::destroyAllWindows();  // 结束前关闭窗口
    }
    return NULL;
}


// =============================================================================
// IMU publish thread: dequeue ImuSample → publish to ROS
// Runs independently from SDK fetch, so ms_sleep never delays publish.
// =============================================================================
void* imuPublishThread(void*) {
    while (m_bIsRunnig) {
        pthread_mutex_lock(&g_imuPubMutex);
        while (g_imuPubQueue.empty() && m_bIsRunnig)
            pthread_cond_wait(&g_imuPubCond, &g_imuPubMutex);
        if (!m_bIsRunnig) { pthread_mutex_unlock(&g_imuPubMutex); break; }

        ImuSample s = g_imuPubQueue.front();
        g_imuPubQueue.pop();
        pthread_mutex_unlock(&g_imuPubMutex);

        auto imuMsg = boost::make_shared<sensor_msgs::Imu>();
        imuMsg->header.stamp    = s.stamp;
        imuMsg->header.frame_id = "mo_imu";
        imuMsg->linear_acceleration.x = s.ax;
        imuMsg->linear_acceleration.y = s.ay;
        imuMsg->linear_acceleration.z = s.az;
        imuMsg->angular_velocity.x    = s.gx;
        imuMsg->angular_velocity.y    = s.gy;
        imuMsg->angular_velocity.z    = s.gz;
        imuMsg->orientation_covariance[0]         = -1.0;
        imuMsg->linear_acceleration_covariance[0] = -1.0;
        imuMsg->angular_velocity_covariance[0]    = -1.0;
        g_pubImu.publish(imuMsg);
    }
    return NULL;
}

// =============================================================================
// IMU thread
// =============================================================================
void* getIMUProcess(void* phCameraHandle) {
    int32_t      n32Result  = 0;
    mo_imu_data* pstIMUData = NULL;
    uint64_t     lastImuTs  = 0;

    while (m_bIsRunnig) {
        ms_sleep(FETCH_IMU_TIME_LENGTH);

        n32Result = moGetIMUData(*((MO_CAMERA_HANDLE*)phCameraHandle), &pstIMUData);
        if (0 != n32Result) { printf("Error: moGetIMUData %d\n", n32Result); continue; }

        double imu_dt_ms = 0.0;
        if (lastImuTs != 0)
            imu_dt_ms = (double)(pstIMUData->u64Timestamp - lastImuTs) / 1000.0;
        lastImuTs = pstIMUData->u64Timestamp;

        uint64_t frameNum = pstIMUData->u64ImageFrameNum;

        pthread_mutex_lock(&g_mapMutex);
        auto it = g_imuMap.find(frameNum);
        if (it == g_imuMap.end()) {
            IMURecord rec;
            rec.u64Timestamp = rec.u64FirstTimestamp = pstIMUData->u64Timestamp;
            rec.dTemperature = pstIMUData->dTemperature;
            rec.dAccelX = pstIMUData->dAccelX;
            rec.dAccelY = pstIMUData->dAccelY;
            rec.dAccelZ = pstIMUData->dAccelZ;
            rec.dGyroX  = pstIMUData->dGyroX;
            rec.dGyroY  = pstIMUData->dGyroY;
            rec.dGyroZ  = pstIMUData->dGyroZ;
            g_imuMap[frameNum] = rec;
        } else {
            IMURecord& rec   = it->second;
            rec.u64Timestamp = pstIMUData->u64Timestamp;
            rec.dTemperature = pstIMUData->dTemperature;
            rec.dAccelX = pstIMUData->dAccelX;
            rec.dAccelY = pstIMUData->dAccelY;
            rec.dAccelZ = pstIMUData->dAccelZ;
            rec.dGyroX  = pstIMUData->dGyroX;
            rec.dGyroY  = pstIMUData->dGyroY;
            rec.dGyroZ  = pstIMUData->dGyroZ;
        }

        if (g_pendingFrames.count(frameNum)) {
            updateTd(g_imuMap[frameNum]);
            g_pendingFrames.erase(frameNum);
        }

        while (g_imuMap.size() > MAX_IMU_MAP_SIZE)
            g_imuMap.erase(g_imuMap.begin());
        pthread_mutex_unlock(&g_mapMutex);

        // Enqueue IMU sample for publish thread (decoupled from SDK fetch timing)
        ImuSample s;
        s.stamp = imuTsToRosTime(pstIMUData->u64Timestamp);
        s.ax = pstIMUData->dAccelX;
        s.ay = pstIMUData->dAccelY;
        s.az = pstIMUData->dAccelZ;
        s.gx = pstIMUData->dGyroX;
        s.gy = pstIMUData->dGyroY;
        s.gz = pstIMUData->dGyroZ;

        pthread_mutex_lock(&g_imuPubMutex);
        if (g_imuPubQueue.size() >= MAX_IMU_PUB_QUEUE)
            g_imuPubQueue.pop();  // drop oldest if buffer full
        g_imuPubQueue.push(s);
        pthread_cond_signal(&g_imuPubCond);
        pthread_mutex_unlock(&g_imuPubMutex);

        //printf("[IMU]    FrameNum: %lu  Timestamp: %lu us  dt: %.3f ms\n",
               //frameNum, pstIMUData->u64Timestamp, imu_dt_ms);
        ROS_INFO_THROTTLE(1.0, "[IMU] FrameNum: %lu dt: %.3f ms", frameNum, imu_dt_ms);
    }
    return NULL;
}

// =============================================================================
// Usage
// =============================================================================
void usage() {
    printf("Usage:\n");
    printf("  ./mo_camera_ros [videoIdx] [--no-display]\n\n");
    printf("ROS topics:\n");
    printf("  /mo_camera/left/image_raw   MONO8 (800 KB/frame)\n");
    printf("  /mo_camera/right/image_raw  MONO8 (800 KB/frame)\n");
    printf("  /mo_camera/imu              200 Hz\n\n");
    printf("Note: images are rectified 8-bit (left: Y-only, right: I420 Y-plane).\n");
    printf("      Published directly as MONO8, no bit-shift conversion.\n");
    printf("      Add --no-display if running headless (no X11/monitor).\n\n");
}

// =============================================================================
// Main
// =============================================================================
int32_t main(int32_t argc, char** argv) {
    ros::init(argc, argv, "mo_camera_ros");
    ros::NodeHandle nh;

    // 解析 --no-display（无显示器/SSH 环境用），并从 argv 中移除，避免影响 cameraSampleInit 解析 videoIdx
    std::vector<char*> filteredArgv;
    filteredArgv.push_back(argv[0]);
    for (int32_t i = 1; i < argc; ++i) {
        if (0 == strcmp(argv[i], "--no-display")) {
            g_bEnableDisplay = false;
        } else {
            filteredArgv.push_back(argv[i]);
        }
    }
    int32_t filteredArgc = (int32_t)filteredArgv.size();

    g_pubLeftImage  = nh.advertise<sensor_msgs::Image>("/mo_camera/left/image_raw",  5);
    g_pubRightImage = nh.advertise<sensor_msgs::Image>("/mo_camera/right/image_raw", 5);
    g_pubImu        = nh.advertise<sensor_msgs::Imu>  ("/mo_camera/imu",             200);

    usage();
    printf("SDK version: %s\n", moGetSdkVersion());
    printf("Display: %s\n", g_bEnableDisplay ? "ON" : "OFF (--no-display)");

    MO_CAMERA_HANDLE hCameraHandle = MO_INVALID_HANDLE;
    int32_t          n32Result     = 0;
    char             caCameraPath[64];

    if (0 != cameraSampleInit(filteredArgc, filteredArgv.data(), caCameraPath)) return -1;

    n32Result = moOpenUVCCameraByPath(caCameraPath, &hCameraHandle);
    if (0 != n32Result) { printf("Open Camera Failed %d\n", n32Result); return -1; }

    n32Result = moGetVideoResolution(hCameraHandle, &g_u16Width, &g_u16Height);
    if (0 != n32Result) {
        printf("Get video resolution failed %d\n", n32Result);
        moCloseCamera(&hCameraHandle); return -1;
    }
    printf("Resolution: %u x %u  →  MONO8: %u KB/frame\n",
           g_u16Width, g_u16Height, g_u16Width * g_u16Height / 1024);

    n32Result = moSetVideoMode(hCameraHandle, MVM_RECTIFIED);
    if (0 != n32Result) {
        printf("moSetVideoMode failed %d\n", n32Result);
        moCloseCamera(&hCameraHandle); return -1;
    }

    pthread_t img_thread = -1, imu_thread = -1, pub_thread = -1, imu_pub_thread = -1;
    pthread_create(&pub_thread,     NULL, publishThread,    NULL);
    pthread_create(&imu_pub_thread, NULL, imuPublishThread, NULL);  // IMU publish thread (decoupled)
    pthread_create(&img_thread,     NULL, getIMGProcess,   &hCameraHandle);
    pthread_create(&imu_thread,     NULL, getIMUProcess,   &hCameraHandle);

    printf("Publishing... Press Ctrl+C to quit.\n");
    ros::spin();

    m_bIsRunnig = false;
    pthread_cond_broadcast(&g_pubCond);
    pthread_cond_broadcast(&g_imuPubCond);
    pthread_join(img_thread,     NULL);
    pthread_join(imu_thread,     NULL);
    pthread_join(pub_thread,     NULL);
    pthread_join(imu_pub_thread, NULL);

    pthread_mutex_lock(&g_tdMutex);
    if (g_tdCount > 0) {
        printf("\n========================================\n");
        printf("Session summary:\n");
        printf("  Frames  : %lu\n", g_tdCount);
        printf("  td_avg  : %+.3f ms (~%.2f fps)\n",
               g_tdSum / (double)g_tdCount,
               1000.0 / (g_tdSum / (double)g_tdCount));
        printf("  Dropped : %lu\n", g_dropCount);
        printf("========================================\n");
    }
    pthread_mutex_unlock(&g_tdMutex);

    moCloseCamera(&hCameraHandle);
    return 0;
}