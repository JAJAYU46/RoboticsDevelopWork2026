#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <atomic>

#include "mo_stereo_camera_driver_c.h"

#define MAX_DSP      8192
#define IMAGE_WIDTH  1280
#define IMAGE_HEIGHT 720

// =============================================================
// 時間錨點：建立硬體時間戳與 ROS 時間的一對一對應
// 硬體時間戳單位目前未知，先以毫秒估計（啟動後印出差值再確認）
// =============================================================
struct TimeAnchor {
    uint64_t  hw_base;       // 第一筆硬體時間戳
    ros::Time ros_base;      // 對應的 ROS 時間
    bool      initialized;
    pthread_mutex_t mutex;

    TimeAnchor() : hw_base(0), initialized(false) {
        pthread_mutex_init(&mutex, NULL);
    }
    ~TimeAnchor() {
        pthread_mutex_destroy(&mutex);
    }

    // 用硬體時間戳換算成 ROS 時間
    // hw_unit_ms：硬體時間戳單位（毫秒=1.0，微秒=0.001，奈秒=0.000001）
    ros::Time toRosTime(uint64_t hw_ts, double hw_unit_ms = 1.0) {
        pthread_mutex_lock(&mutex);
        if (!initialized) {
            hw_base      = hw_ts;
            ros_base     = ros::Time::now();
            initialized  = true;
            ROS_INFO("[TimeAnchor] Initialized: hw_base=%lu, ros_base=%.6f",
                     hw_base, ros_base.toSec());
        }
        double dt_sec = (double)(hw_ts - hw_base) * hw_unit_ms / 1000.0;
        ros::Time result = ros_base + ros::Duration(dt_sec);
        pthread_mutex_unlock(&mutex);
        return result;
    }
};

// 全域時間錨點，IMU 執行緒與主迴圈共用
static TimeAnchor g_anchor;

// =============================================================
// HW_UNIT_MS：請依實際情況修改
//   1.0      → 硬體時間戳單位為毫秒
//   0.001    → 單位為微秒
//   0.000001 → 單位為奈秒
//
// 確認方式：啟動後觀察 log 中的 "HW ts diff"，
// 191Hz 的 IMU 兩幀差應為 ~5.24ms：
//   diff ≈ 5     → 毫秒  (HW_UNIT_MS = 1.0)
//   diff ≈ 5243  → 微秒  (HW_UNIT_MS = 0.001)
//   diff ≈ 5243000 → 奈秒 (HW_UNIT_MS = 0.000001)
// =============================================================
#define HW_UNIT_MS 0.001

// ===== IMU 執行緒參數 =====
struct IMUPublisherArgs {
    MO_CAMERA_HANDLE* hCameraHandle;
    ros::Publisher*   imu_pub;
    bool*             running;
};

void* imuPublishThread(void* args) {
    IMUPublisherArgs* a = (IMUPublisherArgs*)args;
    mo_imu_data* pstIMUData = NULL;
    int32_t s32Result = 0;

    uint64_t prev_hw_ts = 0;
    int      log_count  = 0;

    while (*(a->running)) {
        s32Result = moGetIMUData(*(a->hCameraHandle), &pstIMUData);
        if (0 != s32Result || pstIMUData == NULL) {
            usleep(1000);
            continue;
        }

        // --- 前幾筆印出差值，協助確認 HW_UNIT_MS ---
        if (log_count < 10) {
            if (prev_hw_ts != 0) {
                int64_t diff = (int64_t)pstIMUData->u64Timestamp - (int64_t)prev_hw_ts;
                ROS_INFO("[IMU] hw_ts=%lu  diff=%ld  (191Hz 預期 ~5.24ms 對應的原始差值)",
                         pstIMUData->u64Timestamp, diff);
            }
            prev_hw_ts = pstIMUData->u64Timestamp;
            log_count++;
        }

        // 用硬體時間戳換算 ROS 時間
        ros::Time stamp = g_anchor.toRosTime(pstIMUData->u64Timestamp, HW_UNIT_MS);

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp    = stamp;
        imu_msg.header.frame_id = "imu_frame";

        imu_msg.angular_velocity.x = pstIMUData->dGyroX;
        imu_msg.angular_velocity.y = pstIMUData->dGyroY;
        imu_msg.angular_velocity.z = pstIMUData->dGyroZ;

        imu_msg.linear_acceleration.x = pstIMUData->dAccelX;
        imu_msg.linear_acceleration.y = pstIMUData->dAccelY;
        imu_msg.linear_acceleration.z = pstIMUData->dAccelZ;

        imu_msg.orientation_covariance[0]         = -1;
        imu_msg.angular_velocity_covariance[0]    = -1;
        imu_msg.linear_acceleration_covariance[0] = -1;

        a->imu_pub->publish(imu_msg);

        // 不再用 usleep 控制頻率，讓 SDK 的 blocking 行為決定取樣率
        // 若 moGetIMUData 是非阻塞的，保留短暫 sleep 避免 busy-loop
        usleep(100);
    }
    return NULL;
}
// ========================

unsigned char * GetColorTable(){
  int i;
  unsigned char *colorTable;
  colorTable = (unsigned char *)malloc(3 * 8192 * sizeof(unsigned char));
  colorTable[0] = 0; colorTable[1] = 0; colorTable[2] = 0;
  for (i = 1; i <= 24; i++) { colorTable[i*3]=255; colorTable[i*3+1]=255; colorTable[i*3+2]=255; }
  for (i = 25; i <= 40; i++) {
    colorTable[i*3]   = (int)(255-((255.0-128.0)/(40.0-24.0))*(i-24));
    colorTable[i*3+1] = (int)(255-((255.0-128.0)/(40.0-24.0))*(i-24));
    colorTable[i*3+2] = (int)(255-((255.0-128.0)/(40.0-24.0))*(i-24));
  }
  for (i = 41; i <= 64; i++) {
    colorTable[i*3]   = (int)(128+(int)(5.291668*(i-41)));
    colorTable[i*3+1] = (int)(128-(int)(5.291668*(i-41)));
    colorTable[i*3+2] = (int)(128+(int)(5.291668*(i-41)));
  }
  for (i = 65;  i <= 120;  i++) { colorTable[i*3]=(int)(255-(int)(4.553571*(i-64)));  colorTable[i*3+1]=0;   colorTable[i*3+2]=255; }
  for (i = 121; i <= 176;  i++) { colorTable[i*3]=0; colorTable[i*3+1]=(int)(0+(int)(4.553571*(i-120)));    colorTable[i*3+2]=255; }
  for (i = 177; i <= 320;  i++) { colorTable[i*3]=0; colorTable[i*3+1]=255; colorTable[i*3+2]=(int)(255-(int)(1.770833*(i-176))); }
  for (i = 321; i <= 800;  i++) { colorTable[i*3]=(int)(0+(int)(0.53125*(i-320)));    colorTable[i*3+1]=255; colorTable[i*3+2]=0; }
  for (i = 801; i <= 2048; i++) { colorTable[i*3]=255; colorTable[i*3+1]=(int)(255-(int)(0.204327*(i-800))); colorTable[i*3+2]=0; }
  for (i = 2049; i < 8192; i++) { colorTable[i*3]=255; colorTable[i*3+1]=0; colorTable[i*3+2]=0; }
  return colorTable;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mo_rgbd");
    ros::NodeHandle nh;

    int image_half_height = IMAGE_HEIGHT >> 1;
    int image_half_width  = IMAGE_WIDTH  >> 1;
    std::string camPath;
    int fps_, PCLOUD_REDUCE_RATE;

    nh.param<std::string>("cam_path", camPath, "/dev/video0");
    nh.param<int>("fps", fps_, 30);
    nh.param<int>("point_cloud_down_sample", PCLOUD_REDUCE_RATE, 1);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_pub     = it.advertise("mo_cam/left/image_raw", 1);
    image_transport::Publisher right_pub    = it.advertise("mo_cam/right/image_raw", 1);
    image_transport::Publisher img_pub      = it.advertise("mo_cam/image_color", 1);
    image_transport::Publisher depth_pub    = it.advertise("mo_cam/image_depth", 1);
    image_transport::Publisher dspColor_pub = it.advertise("mo_cam/image_dsp", 1);

    ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("mo_cam/camera_info", 1);
    ros::Publisher imu_pub      = nh.advertise<sensor_msgs::Imu>("mo_cam/imu", 100);

    camera_info_manager::CameraInfoManager cam_info_manager(
        nh, "mo_cam/camera_info", "package://moak_camera/config/calib.yaml");
    sensor_msgs::CameraInfo cam_info = cam_info_manager.getCameraInfo();

    int32_t          s32Result     = 0;
    MO_CAMERA_HANDLE hCameraHandle = MO_INVALID_HANDLE;

    std::cout << "SDK version : " << moGetSdkVersion() << std::endl;
    std::cout << "Camera path : " << camPath << std::endl;

    s32Result = moOpenUVCCameraByPath("/dev/video0", &hCameraHandle);
    if (0 != s32Result) { printf("Error: moOpenUVCCameraByPath %d\n", s32Result); return -1; }

    s32Result = moSetVideoMode(hCameraHandle, MVM_RECTIFIED);
    if (0 != s32Result) { printf("Error: moSetVideoMode(MVM_RECTIFIED) %d\n", s32Result); return -2; }
    std::cout << "Video mode: MVM_RECTIFIED" << std::endl;

    float fBxf = 0.0f, fBase = 0.0f;
    s32Result = moGetBxfAndBase(hCameraHandle, &fBxf, &fBase);
    if (0 != s32Result) { printf("Error: moGetBxfAndBase %d\n", s32Result); return -3; }
    std::cout << "fBF: " << fBxf << ", fBase: " << fBase << std::endl;

    float* m_mapDspToDisZ    = new float[MAX_DSP];
    float* m_mapDspToDisZ_M  = new float[MAX_DSP];
    float* m_mapDspToDisXY_M = new float[MAX_DSP];
    for(int i = 1; i < MAX_DSP; i++){
        m_mapDspToDisZ[i]    = fBxf  * 32.0 / i;
        m_mapDspToDisZ_M[i]  = fBxf  * 32.0 / i / 1000.0;
        m_mapDspToDisXY_M[i] = fBase * 32.0 / i / 1000.0;
    }
    unsigned char* colorTable = GetColorTable();

    // ===== 啟動 IMU 執行緒 =====
    // IMU 執行緒會取得第一筆硬體時間戳並初始化 g_anchor
    bool imuRunning = true;
    IMUPublisherArgs imuArgs = { &hCameraHandle, &imu_pub, &imuRunning };
    pthread_t imu_thread;
    pthread_create(&imu_thread, NULL, imuPublishThread, &imuArgs);
    std::cout << "IMU thread started, publishing to /mo_cam/imu" << std::endl;

    // 等待 IMU 執行緒完成錨點初始化，最多等 500ms
    int wait_ms = 0;
    while (!g_anchor.initialized && wait_ms < 500) {
        usleep(10000);  // 10ms
        wait_ms += 10;
    }
    if (!g_anchor.initialized) {
        ROS_WARN("[main] IMU anchor not initialized after 500ms, camera timestamps may be off");
    }
    // ===========================

    uint64_t u64ImageFrameNum   = 0;
    uint8_t* pu8FrameBuffer     = NULL;
    uint8_t* pu8LeftGrayImg     = NULL;
    uint8_t* pu8RightYUVI420Img = NULL;

    // 相機幀計數器，用來估算相機硬體時間戳（相機無獨立硬體時間戳時的備用方案）
    uint64_t cam_frame_count = 0;
    ros::Time cam_first_ros;
    bool cam_anchor_initialized = false;

    ros::Rate loop_rate(fps_);
    while (ros::ok())
    {
        s32Result = moGetCurrentFrame(hCameraHandle, &u64ImageFrameNum, &pu8FrameBuffer);
        if (0 != s32Result) {
            printf("Warning: moGetCurrentFrame %d\n", s32Result);
            ros::spinOnce(); loop_rate.sleep(); continue;
        }

        // 相機時間戳：用 IMU 錨點的 ros_base 加上幀號推算
        // 避免每幀都呼叫 ros::Time::now() 引入系統排程抖動
        ros::Time cam_stamp;
        if (g_anchor.initialized) {
            if (!cam_anchor_initialized) {
                // 第一幀：記錄此時的 ROS 時間作為相機錨點
                cam_first_ros            = ros::Time::now();
                cam_frame_count          = 0;
                cam_anchor_initialized   = true;
            }
            // 用幀號 × 幀週期推算時間，比每幀 now() 更穩定
            double frame_period = 1.0 / (double)fps_;
            cam_stamp = cam_first_ros + ros::Duration(cam_frame_count * frame_period);
        } else {
            cam_stamp = ros::Time::now();
        }
        cam_frame_count++;

        std::cout << "Frame: " << u64ImageFrameNum << std::endl;

        s32Result = moGetRectifiedImage(hCameraHandle, pu8FrameBuffer,
                                        &pu8LeftGrayImg, &pu8RightYUVI420Img);
        if (0 != s32Result || pu8LeftGrayImg == NULL || pu8RightYUVI420Img == NULL) {
            printf("Warning: moGetRectifiedImage %d\n", s32Result);
            ros::spinOnce(); loop_rate.sleep(); continue;
        }

        std_msgs::Header header;
        header.stamp    = cam_stamp;
        header.frame_id = "camera_frame";

        cv::Mat leftGray(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, pu8LeftGrayImg);
        cv_bridge::CvImage cv_left(header, "mono8", leftGray);
        left_pub.publish(cv_left.toImageMsg());

        cv::Mat rightYUV(IMAGE_HEIGHT * 1.5, IMAGE_WIDTH, CV_8UC1, pu8RightYUVI420Img);
        cv::Mat rightGray(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
        cv::cvtColor(rightYUV, rightGray, cv::COLOR_YUV2GRAY_I420);
        cv_bridge::CvImage cv_right(header, "mono8", rightGray);
        right_pub.publish(cv_right.toImageMsg());

        cv::Mat rightBGR(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
        cv::cvtColor(rightYUV, rightBGR, cv::COLOR_YUV2BGR_I420);
        cv_bridge::CvImage cv_color(header, "bgr8", rightBGR);
        img_pub.publish(cv_color.toImageMsg());

        cam_info.header = header;
        cam_info_pub.publish(cam_info);

        ros::spinOnce();
        loop_rate.sleep();
    }

    imuRunning = false;
    pthread_join(imu_thread, NULL);

    delete[] m_mapDspToDisZ;
    delete[] m_mapDspToDisZ_M;
    delete[] m_mapDspToDisXY_M;
    free(colorTable);
    moCloseCamera(&hCameraHandle);
    return 0;
}
