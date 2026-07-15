#include "common_function.h"
#include "mo_stereo_camera_driver_c_utilities.h"

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <memory>

#ifdef USE_OPENCV
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
#endif

// -------------------------------------------------------
// 設定參數
// -------------------------------------------------------
#define OBSTACLE_DISTANCE_THRESHOLD_MM  1500.0f  // 警告距離門檻 (mm)
#define DISPARITY_MIN_VALID             1         // 最小有效視差值
#define CONTOUR_MIN_AREA                300       // 最小輪廓面積（過濾雜訊）

void usage() {
    printf("Usage: ./obstacle_detection [videoIdx]\n\n");
    printf("操作說明:\n");
    printf("  滑鼠左鍵拖曳 → 手動框選要測距的物體\n");
    printf("  's' → 固定目前框選區域\n");
    printf("  'c' → 清除框選\n");
    printf("  'q' → 離開\n\n");
}

#ifdef USE_OPENCV

// -------------------------------------------------------
// 全域變數：滑鼠框選用
// -------------------------------------------------------
static Rect  g_roiRect;        // 使用者框選的區域
static bool  g_drawing = false; // 正在拖曳中
static bool  g_roiFixed = false;// 是否固定框
static Point g_startPt;

// -------------------------------------------------------
// PCL Viewer（在獨立執行緒中運行）
// -------------------------------------------------------
static pcl::visualization::PCLVisualizer::Ptr g_pclViewer;
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_cloud;
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_roiCloud;
static float fDepthMMGlobal = -1.0f;
static float g_fBxf  = 0.0f;
static float g_fBase = 0.0f;
static std::mutex g_cloudMutex;
static bool g_pclRunning = false;

void pclViewerThread() {
    
    //g_pclViewer = std::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");
    //<Debug>
    g_pclViewer.reset(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    g_pclViewer->setBackgroundColor(0, 0, 0);
    g_pclViewer->addCoordinateSystem(100.0);
    g_pclViewer->initCameraParameters();

    // 相機座標系：X右、Y下、Z前（深度方向）
    // 視角從後方往前看，Y軸向下對齊影像座標
    g_pclViewer->setCameraPosition(
        0,    0,    -1500,   // 相機位置：Z=-1500（在場景後方）
        0,    0,    0,       // 看向原點
        0,    -1,   0        // 上方向：Y軸負方向（對應影像 Y 向下）
    );
    g_pclViewer->setCameraFieldOfView(0.8);

    g_pclRunning = true;
    while (!g_pclViewer->wasStopped()) {
        g_pclViewer->spinOnce(30);
        {
            std::lock_guard<std::mutex> lock(g_cloudMutex);
            // 更新全域點雲
            if (g_cloud && !g_cloud->empty()) {
                if (!g_pclViewer->updatePointCloud(g_cloud, "cloud"))
                    g_pclViewer->addPointCloud(g_cloud, "cloud");
                g_pclViewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
            }
            // 更新 ROI 點雲（黃色高亮）
            if (g_roiCloud && !g_roiCloud->empty()) {
                if (!g_pclViewer->updatePointCloud(g_roiCloud, "roi_cloud"))
                    g_pclViewer->addPointCloud(g_roiCloud, "roi_cloud");
                g_pclViewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "roi_cloud");
            } else {
                g_pclViewer->removePointCloud("roi_cloud");
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    g_pclRunning = false;
}

void onMouse(int event, int x, int y, int, void*)
{
    if (event == EVENT_LBUTTONDOWN) {
        g_drawing  = true;
        g_roiFixed = false;
        g_startPt  = Point(x, y);
        g_roiRect  = Rect(x, y, 0, 0);
    }
    else if (event == EVENT_MOUSEMOVE && g_drawing) {
        g_roiRect = Rect(
            min(g_startPt.x, x), min(g_startPt.y, y),
            abs(x - g_startPt.x), abs(y - g_startPt.y));
    }
    else if (event == EVENT_LBUTTONUP) {
        g_drawing  = false;
        g_roiFixed = true;
        g_roiRect = Rect(
            min(g_startPt.x, x), min(g_startPt.y, y),
            abs(x - g_startPt.x), abs(y - g_startPt.y));
    }
}



#endif  // USE_OPENCV

// -------------------------------------------------------
// main
// -------------------------------------------------------
int32_t main(int32_t argc, char** argv)
{
    usage();
    printf("SDK version: %s\n", moGetSdkVersion());

    MO_CAMERA_HANDLE hCameraHandle = MO_INVALID_HANDLE;
    int32_t n32Result = 0;
    char caCameraPath[64];
    if (0 != cameraSampleInit(argc, argv, caCameraPath)) return -1;

    n32Result = moOpenUVCCameraByPath(caCameraPath, &hCameraHandle);
    if (0 != n32Result) { printf("Open Camera Failed: %d\n", n32Result); return -1; }

    uint16_t u16W = 0, u16H = 0;
    moGetVideoResolution(hCameraHandle, &u16W, &u16H);
    printf("Resolution: %d x %d\n", u16W, u16H);

    // moConvertDisparity2PointCloud 只支援 MVM_RGBD 模式
    n32Result = moSetVideoMode(hCameraHandle, MVM_RGBD);
    if (0 != n32Result) {
        printf("Set video mode failed: %d\n", n32Result);
        return -1;
    }

    // 主迴圈變數
    uint64_t  u64FrameNum           = 0;
    uint8_t*  pu8FrameBuffer        = NULL;
    uint16_t* pu16RGBDDisparityData = NULL;
    uint8_t*  pu8RGBDYUVI420Img     = NULL;
    double    d8FPS = 0.0;
    char      caFPS[32] = {0};




#ifdef USE_OPENCV
    // 建立視窗並綁定滑鼠事件
    const char* WIN_NAME = "Distance Measurement";
    namedWindow(WIN_NAME, WINDOW_NORMAL);
    setMouseCallback(WIN_NAME, onMouse);
#endif

    printf("Press ENTER to quit  |  'q' in window to quit  |  'p' 開啟/更新點雲\n");

    // 啟動 PCL Viewer 執行緒
    //g_cloud    = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    //g_roiCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    //<Debug><20260714>版本語法不同因為是在Ubuntu20.04, Noetic用的是舊版PCL
    g_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    g_roiCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::thread pclThread(pclViewerThread);
    pclThread.detach();

    while (0 == getchar_nb(FETCH_AND_DISPLAY_TIME_LENGTH))
    {
        // 1. 取得影像幀
        n32Result = moGetCurrentFrame(hCameraHandle, &u64FrameNum, &pu8FrameBuffer);
        if (0 != n32Result) { printf("Get frame failed, retry...\n"); continue; }

        // 2. FPS
        if (0 != moGetRealTimeFPS(hCameraHandle, &d8FPS))
            sprintf(caFPS, "FPS: Calculating...");
        else
            sprintf(caFPS, "FPS: %.1f", d8FPS);

        // 3. 取得 RGBD 影像
        n32Result = moGetRGBDImage(hCameraHandle, pu8FrameBuffer,
                                   &pu16RGBDDisparityData, &pu8RGBDYUVI420Img);
        if (0 != n32Result) { printf("moGetRGBDImage failed: %d\n", n32Result); continue; }



        // 5. 更新點雲（按 'p' 時觸發）
        // 每幀都更新點雲資料到全域變數
        {
            float*   pfX = NULL, *pfY = NULL, *pfZ = NULL;
            uint32_t u32Size = 0;
            if (0 == moConvertDisparity2PointCloud(hCameraHandle, pu16RGBDDisparityData,
                                                    &u32Size, &pfX, &pfY, &pfZ)) {
                std::lock_guard<std::mutex> lock(g_cloudMutex);
                g_cloud->clear();
                g_roiCloud->clear();

                for (uint32_t i = 0; i < u32Size; ++i) {
                    if (pfZ[i] <= 0.0f || pfZ[i] > 10000.0f) continue;
                    pcl::PointXYZRGB pt;
                    pt.x = pfX[i];
                    pt.y = pfY[i];
                    pt.z = pfZ[i];
                    // 依深度著色（近=紅，遠=藍）
                    float t = std::min(pfZ[i] / 5000.0f, 1.0f);
                    pt.r = (uint8_t)((1.0f - t) * 255);
                    pt.g = 50;
                    pt.b = (uint8_t)(t * 255);
                    g_cloud->push_back(pt);
                }

                // 把框選區域內的點單獨存到 g_roiCloud（標黃色）
                // 用焦距將像素座標轉成世界座標範圍過濾
                if (g_roiRect.area() > 10) {
                    g_roiCloud->clear();

                    // 焦距估算：focal = fBxf / fBase
                    // 影像中心 cx, cy
                    float focal = g_fBxf / g_fBase;
                    float cx = u16W / 2.0f;
                    float cy = u16H / 2.0f;

                    // 框選區域的像素範圍
                    float px_min = g_roiRect.x;
                    float px_max = g_roiRect.x + g_roiRect.width;
                    float py_min = g_roiRect.y;
                    float py_max = g_roiRect.y + g_roiRect.height;

                    for (uint32_t i = 0; i < u32Size; ++i) {
                        if (pfZ[i] <= 0.0f || pfZ[i] > 10000.0f) continue;

                        // 用世界座標反推像素座標
                        // pixel_x = X * focal / Z + cx
                        float px = pfX[i] * focal / pfZ[i] + cx;
                        float py = pfY[i] * focal / pfZ[i] + cy;

                        // 判斷是否在框選範圍內
                        if (px >= px_min && px <= px_max &&
                            py >= py_min && py <= py_max) {
                            pcl::PointXYZRGB pt;
                            pt.x = pfX[i];
                            pt.y = pfY[i];
                            pt.z = pfZ[i];
                            pt.r = 255; pt.g = 255; pt.b = 0;  // 黃色
                            g_roiCloud->push_back(pt);
                        }
                    }
                }
            }
        }

#ifdef USE_OPENCV
        // 6. 取得灰階影像（Y channel）
        Mat matYUVI420(u16H * 1.5, u16W, CV_8UC1, pu8RGBDYUVI420Img);
        Mat matGray(matYUVI420, Rect(0, 0, u16W, u16H));
        Mat matDisplay;
        cvtColor(matGray, matDisplay, COLOR_GRAY2BGR);  // 灰階轉 3ch 方便畫框

        // 6. 視差圖
        Mat matDisparity(u16H, u16W, CV_8UC3);
        GetDisparityImage(matDisparity.rows, matDisparity.cols,
                          matDisparity.step, pu16RGBDDisparityData, matDisparity.data);

        // ---------------------------------------------------
        // 7. 計算距離
        // ---------------------------------------------------
        float fDepthMM = -1.0f;

        if (g_roiRect.area() > 10)
        {
            // 使用 SDK 官方 API 直接計算矩形區域深度
            mo_rect_distance_info stRectInfo;
            memset(&stRectInfo, 0, sizeof(stRectInfo));
            stRectInfo.stInParam.u16LeftTopCornerX    = (uint16_t)max(g_roiRect.x, 0);
            stRectInfo.stInParam.u16LeftTopCornerY    = (uint16_t)max(g_roiRect.y, 0);
            stRectInfo.stInParam.u16RectWidth         = (uint16_t)g_roiRect.width;
            stRectInfo.stInParam.u16RectHeight        = (uint16_t)g_roiRect.height;
            stRectInfo.stInParam.pu16RGBDDisparityData = pu16RGBDDisparityData;

            n32Result = moCalculateRectDistance(hCameraHandle, &stRectInfo);
            if (0 == n32Result) {
                fDepthMM = stRectInfo.stOutParam.fMedianDepth;
                fDepthMMGlobal = fDepthMM;  // 同步給 PCL 用
                printf("  Density:%.2f  Min:%.0f  Max:%.0f  Median:%.0f mm\n",
                       stRectInfo.stOutParam.fDisparityDensity,
                       stRectInfo.stOutParam.fMinimumDepth,
                       stRectInfo.stOutParam.fMaximumDepth,
                       stRectInfo.stOutParam.fMedianDepth);
            }

            // 畫框
            Scalar color = (fDepthMM > 0 && fDepthMM < OBSTACLE_DISTANCE_THRESHOLD_MM)
                           ? Scalar(0, 0, 255)   // 紅 = 近
                           : Scalar(0, 255, 0);  // 綠 = 遠或無效

            rectangle(matDisplay,   g_roiRect, color, 2);
            rectangle(matDisparity, g_roiRect, color, 2);

            // 顯示距離標籤
            if (fDepthMM > 0) {
                char caLabel[64];
                sprintf(caLabel, "%.0f mm  (%.2f m)",
                        fDepthMM, fDepthMM / 1000.0f);

                // 標籤背景
                int baseline = 0;
                Size ts = getTextSize(caLabel, FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
                int ly = max(g_roiRect.y - 8, ts.height + 8);
                rectangle(matDisplay,
                          Point(g_roiRect.x, ly - ts.height - 4),
                          Point(g_roiRect.x + ts.width + 4, ly + 4),
                          color, FILLED);
                putText(matDisplay, caLabel,
                        Point(g_roiRect.x + 2, ly),
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 2);

                // 同步到視差圖
                putText(matDisparity, caLabel,
                        Point(g_roiRect.x + 2, ly),
                        FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

                // Terminal 也印出
                printf("  距離: %.1f mm  (%.3f m)\n", fDepthMM, fDepthMM / 1000.0f);

                // 警告
                if (fDepthMM < OBSTACLE_DISTANCE_THRESHOLD_MM) {
                    putText(matDisplay, "!! TOO CLOSE !!",
                            Point(u16W / 2 - 120, 50),
                            FONT_HERSHEY_DUPLEX, 1.0, Scalar(0, 0, 255), 2);
                }
            } else {
                // 視差資料不足
                putText(matDisplay, "No depth data",
                        Point(g_roiRect.x, max(g_roiRect.y - 8, 20)),
                        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 165, 255), 2);
            }
        }

        // ---------------------------------------------------
        // 8. 提示文字
        // ---------------------------------------------------
        // 拖曳中提示
        if (g_drawing) {
            putText(matDisplay, "Selecting...",
                    Point(10, u16H - 40),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);
        }
        // 固定框提示
        if (g_roiFixed && g_roiRect.area() > 10) {
            putText(matDisplay, "Press 'c' to clear",
                    Point(10, u16H - 15),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(180, 180, 180), 1);
        }
        // 無框時提示
        if (!g_drawing && g_roiRect.area() <= 10) {
            putText(matDisplay, "Drag to select object",
                    Point(10, u16H - 15),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
        }

        // FPS
        putText(matDisplay,   string(caFPS), Point(10, 30),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 200, 0), 2);
        putText(matDisparity, string(caFPS), Point(10, 30),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 200, 0), 2);

        // ---------------------------------------------------
        // 9. 顯示
        // ---------------------------------------------------
        imshow(WIN_NAME,    matDisplay);
        imshow("Disparity", matDisparity);

        char key = (char)waitKey(1);
        if (key == 'q') { printf("Quit.\n"); break; }
        if (key == 'c') {
            g_roiRect  = Rect();
            g_roiFixed = false;
            printf("框選已清除\n");
        }
        if (key == 'p') {
            printf("點雲已更新，請查看 PCL Viewer 視窗\n");
        }

#endif  // USE_OPENCV
    }

#ifdef USE_OPENCV
    destroyAllWindows();
#endif
    moCloseCamera(&hCameraHandle);
    printf("Camera closed.\n");
    return 0;
}
