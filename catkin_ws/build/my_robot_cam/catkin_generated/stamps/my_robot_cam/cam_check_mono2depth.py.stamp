#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2

import numpy as np
# from spatialmath import SO3, SE3
# from spatialmath import UnitQuaternion
from scipy.spatial.transform import Rotation as Rot
# 用來同步左右眼的timestamp去確保stereo時左右眼視同個時刻timestamp的去做callback 
import message_filters

FOR_DEBUG = True
DO_EVALUATION = True#False # check instrinct (camera matrix and distortion factor)
DO_POINTCLOUD = True # check extrinct (camera matrix and Baseline)

if(DO_POINTCLOUD): 
    # <For Visualization> 
    import open3d as o3d
    # python3 -m pip install open3d


CHECKER_BOARD_PATTERN_SIZE = (9,6)
CHECKER_BOARD_PATTERN_DISTANCE = 0.022 #單位: [m], 22mm 一格
# ==================== [Set Camera Parameters]  ====================
# <Note>
# 1. Calibration Result from Kalibr. 
# 2. Variable R, t are the Rotation Matrix and Translation Matrix
#    from left camera frame to right camera frame. (R_1r, t_lr)
# 3. T_12: The Transformation Matrix from left cam to right cam

# ==================================================================
# Camera Matrix
fx1, fy1, cx1, cy1 = [376.72365254, 376.73224047, 310.56993772, 188.22580516] # left
fx2, fy2, cx2, cy2 = [376.90291086, 376.90015313, 310.10303915, 188.09722285] # right

# left 
K1 = np.array([
    [fx1,   0, cx1],
    [   0,fy1, cy1],
    [   0,  0,   1]
], dtype=np.float64)
# right 
K2 = np.array([
    [fx2,   0, cx2],
    [   0,fy2, cy2],
    [   0,  0,   1]
], dtype=np.float64)


# Distortion Factor #OpenCV distortion 順序 [k1,k2,p1,p2,k3]
D1 = np.array([-0.00092227, -0.00535565, 0.00192026, -0.00056864], dtype=np.float64)
D2 = np.array([-0.00297347, -0.00090164,  0.0019205,  -0.00082069], dtype=np.float64)

# Rotation # Kalibr Result q
qx, qy, qz, qw = [-0.00004118, -0.00046356,  0.00036192,  0.99999983]
tx, ty, tz = [ 0.06151697, -0.00000337, -0.00004902]
quat = np.array([qx, qy, qz,qw])
r = Rot.from_quat(quat)

R = r.as_matrix() #3x3 numpy array, Rotation Matrix
t = np.array([ tx, ty, tz]).reshape(3,1) # Translation

# Homogeneous transformation matrix
T_left_right = np.eye(4) # 4x4 Transformation Matrix
T_left_right[:3,:3] = R
T_left_right[:3,3:] = t

# ========================= [Start ROS Node] ======================== 
# <Discription>
# Start ROS Node. This Node will get the image from /mo_camera/left/image_raw and /mo_camera/right/image_raw topic, 
# and turn the image_l, image_r to disparity map, and then to depth map and point cloud 
# <Note>
# 1. <Para> Tag 可尋找能夠調整的參數

# ===================================================================
class MyNode:

    def __init__(self):
        
        # Initialize for libraries
        self.bridge = CvBridge() #for opencv
        
        # subscriber undistord image, left, right
        # self.sub_left = rospy.Subscriber("/mo_camera/left/image_raw",Image,self.image_callback_left,queue_size=1)
        # self.sub_right = rospy.Subscriber("/mo_camera/right/image_raw",Image,self.image_callback_right,queue_size=1)
        
        # subscriber for left and right cam with message filter to ensure same timestamp 
        self.sub_left = message_filters.Subscriber("/mo_camera/left/image_raw",Image)
        self.sub_right = message_filters.Subscriber("/mo_camera/right/image_raw",Image)
        # 同步 timestamp # 利用message_filters就不須要分開寫上面兩個callback了, 現在就是同步後一起丟到stereo callback
        ts = message_filters.TimeSynchronizer([self.sub_left, self.sub_right],queue_size=10)
        ts.registerCallback(self.stereo_callback)

        # publisher
        self.pub = rospy.Publisher("/my_topic",String,queue_size=10)


        # [For Get Evaluation]
        self.current_img_left = None
        self.current_img_right = None

        # self.evaluate_flag = False
        self.error_vector_left = []
        self.error_vector_right = []

        # [For Get PointCloud]
        self.image_size = (640,360)
        self.Rl, self.Rr, self.Pl, self.Pr, self.Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, self.image_size , R, t, alpha=0)
        self.xmap_l, self.ymap_l = cv2.initUndistortRectifyMap(K1, D1, self.Rl, self.Pl, self.image_size , cv2.CV_32FC1)
        self.xmap_r, self.ymap_r = cv2.initUndistortRectifyMap(K2, D2, self.Rr, self.Pr, self.image_size , cv2.CV_32FC1)
                    

        rospy.loginfo("Start cam_check_calibration node")
    

    def stereo_callback(self, msg_left, msg_right):
        
        # # 取得 timestamp
        # time_left = msg_left.header.stamp
        # time_right = msg_right.header.stamp
        # rospy.loginfo(
        #     "left: %f right: %f diff: %f",
        #     time_left.to_sec(),
        #     time_right.to_sec(),
        #     abs(time_left.to_sec()-time_right.to_sec())
        # )

        #rospy.loginfo("Receive left image")
        #rospy.loginfo_throttle(1, "Receive left image")
        
        # ==============================



        # ==============================

        # <Step> 1. Get Original distored image
        img_left = self.bridge.imgmsg_to_cv2(msg_left, desired_encoding = "mono8")
        img_right = self.bridge.imgmsg_to_cv2(msg_right, desired_encoding = "mono8")
        
        # 保存目前影像
        self.current_img_left = img_left.copy()
        self.current_img_right = img_right.copy()

        # <Step> 2. Undistorted by camera instrincts
        img_undistorted_left = cv2.undistort(img_left,K1,D1)
        img_undistorted_right = cv2.undistort(img_right,K2,D2)
    
        # <For Visualization> Visialization in one window
        img_left_titled = self.add_title(img_left, "Raw Image Left")
        img_right_titled = self.add_title(img_right, "Raw Image Right")
        img_undistorted_left_titled = self.add_title(img_undistorted_left, "Undistorted Image Left")
        img_undistorted_right_titled = self.add_title(img_undistorted_right, "Undistorted Image Right")
        window_combined = np.vstack((
            np.hstack((img_left_titled, img_undistorted_left_titled)), #top
            np.hstack((img_right_titled, img_undistorted_right_titled)) #down
        ))
        cv2.imshow("Checking Camera Instrinct (Camera Matrix/Distortion Factor)", window_combined)
        key = cv2.waitKey(1)

        if (key == 32): #按下空白鍵才做evaluation and pointcloud 不然會有latency
            print("Capture image for evaluation and PCD")
            # <Step> 3. Do Evaluation
            if (DO_EVALUATION): 
                # window_combined = np.vstack((
                #     np.hstack((img_left_titled, img_undistorted_left_titled)), #top
                #     np.hstack((img_right_titled, img_undistorted_right_titled)) #down
                # ))
                # cv2.imshow("Checking Camera Instrinct (Camera Matrix/Distortion Factor)", window_combined)
                error_left = self.evaluate_cam_instrinct(img_left, K1, D1)
                error_right = self.evaluate_cam_instrinct(img_right, K2, D2)
                
                # ------------- <For Visualization> -------------
                if error_left is not None:
                    title_left = f"Undistorted Image Left, Reprojection Error {error_left:.3f}px"
                else:
                    title_left = "Undistorted Image Left, Chessboard Not Found"
                if error_right is not None:
                    title_right = f"Undistorted Image Right, Reprojection Error {error_right:.3f}px"
                else:
                    title_right = "Undistorted Image Right, Chessboard Not Found"
                
                titled_ul = self.add_title(img_undistorted_left, title_left)
                titled_ur  = self.add_title(img_undistorted_right, title_right)
                
                window_combined = np.vstack((
                    np.hstack((img_left_titled, titled_ul)), #top
                    np.hstack((img_right_titled, titled_ur)) #down
                ))
                cv2.imshow("Checking Camera Instrinct (Calculate Reprojection Error)", window_combined)
                # -----------------------------------------------
                
                
                if(error_left is not None): 
                    print("Reprojection error (left):",error_left,"pixel")
                    self.error_vector_left.append(error_left)
                if(error_right is not None): 
                    print("Reprojection error (right):",error_right,"pixel")
                    self.error_vector_right.append(error_right)
                
            # <Step> 4. Do Point Cloud
            if (DO_POINTCLOUD): 
                # 4.1 Do Rectify - get rectify image
                # <Note> xmap_l本身就是有undistored+rectify了 
                img_left_rectified = cv2.remap(img_left, self.xmap_l, self.ymap_l, cv2.INTER_LINEAR)
                img_right_rectified = cv2.remap(img_right, self.xmap_r, self.ymap_r, cv2.INTER_LINEAR)
                
                # img_left_rectified = cv2.remap(img_undistorted_left, self.xmap_l, self.ymap_l, cv2.INTER_LINEAR)
                # img_right_rectified = cv2.remap(img_undistorted_right, self.xmap_r, self.ymap_r, cv2.INTER_LINEAR)
                
                # <Debug> 
                if(FOR_DEBUG): 
                    debug_left_rect_org, debug_right_rect_org = self.Check_Rectify(img_left, img_right)
                    
                    debug_left_rect, debug_right_rect = self.Check_Rectify(img_left_rectified, img_right_rectified)
                    if (debug_left_rect is not None and debug_right_rect is not None):
                        titled_ul = self.add_title(debug_left_rect, "img_left_rectified checkerboard")
                        titled_ur  = self.add_title(debug_right_rect, "img_right_rectified checkerboard")
                        
                        titled_ul_org = self.add_title(debug_left_rect_org, "img_left_origin checkerboard")
                        titled_ur_org  = self.add_title(debug_right_rect_org, "img_right_origin checkerboard")
                        
                        window_combined = np.vstack((
                            # np.hstack((titled_ul, titled_ur)), #top
                            # np.hstack((img_left_titled, img_right_titled)) #down
                            np.hstack((titled_ul_org, titled_ul)), #top
                            np.hstack((titled_ur_org, titled_ur)) #down
                        ))
                        cv2.imshow("img rectified with checkerboard", window_combined)

                    
                # <For Debug> 
                if (FOR_DEBUG): 
                    cv2.imshow("img_left_rectified", img_left_rectified)
                    cv2.imshow("img_right_rectified", img_right_rectified)
                # 4.2 Do Stereo Matching
                # <Para>
                # 深度太雜亂 → 調 blockSize、speckleWindowSize
                # 遠距離深度不夠 → 增加 numDisparities
                # 邊緣細節不好 → 減小 blockSize
                # ## Set some of the stereo-matching parameters
                # minDisparity = 120      # Minimum disparity 
                # numDisparities = 16*5  # Number of disparities considered. Needs to be a multiple of 16
                # blockSize = 31         # Size of the neighborhood/block. Needs to be an odd number
                # uniquenessRatio = 5    # Threshold on the uniqueness ratio

                # # Create the stereo block matcher
                # stereo = cv2.StereoBM_create()
                # stereo.setMinDisparity(minDisparity)
                # stereo.setNumDisparities(numDisparities)
                # stereo.setBlockSize(blockSize)
                # stereo.setUniquenessRatio(uniquenessRatio)


                # blockSize = 31
                # stereo = cv2.StereoBM_create(
                #     minDisparity = 120,  # Minimum disparity 
                #     numDisparities = 16*5, # Number of disparities considered. Needs to be a multiple of 16
                #     blockSize=blockSize, # Size of the neighborhood/block. Needs to be an odd number
                #     uniquenessRatio = 5, 
                #     # <Note> P1, P2 平滑懲罰(smoothness penalty), 因為希望相鄰像素的 disparity 不要突然變很多
                #     # <Note> P2 一定要比 P1 大很多，否則演算法就不會偏好平滑的深度
                #     # P1=8*blockSize**2, # P1:當相鄰像素 disparity 差 1 時，要付出的代價
                #     # P2=32*blockSize**2 # P2:當相鄰像素 disparity 差 大於 1 時，要付出的代價
                #     #uniquenessRatio=10,
                #     #speckleWindowSize=100,
                #     #speckleRange=32
                # )
                
                
                # blockSize = 5
                # stereo = cv2.StereoSGBM_create( #numDisparities=16, blockSize=15
                #     minDisparity = 0,  # Minimum disparity 
                #     numDisparities = 16*8, # Number of disparities considered. Needs to be a multiple of 16
                #     blockSize=blockSize, # Size of the neighborhood/block. Needs to be an odd number
                #     # <Note> P1, P2 平滑懲罰(smoothness penalty), 因為希望相鄰像素的 disparity 不要突然變很多
                #     # <Note> P2 一定要比 P1 大很多，否則演算法就不會偏好平滑的深度
                #     P1=8*blockSize**2, # P1:當相鄰像素 disparity 差 1 時，要付出的代價
                #     P2=32*blockSize**2, # P2:當相鄰像素 disparity 差 大於 1 時，要付出的代價
                #     uniquenessRatio=10,
                #     speckleWindowSize=100,
                #     speckleRange=32
                # )
                stereo = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=128,     # 必須是16的倍數
                blockSize=7,            # 奇數，越大越平滑但細節少

                P1=8 * 3 * 7**2,
                P2=32 * 3 * 7**2,

                disp12MaxDiff=1,
                uniquenessRatio=10,

                speckleWindowSize=100,
                speckleRange=32,

                preFilterCap=31,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )


                # 4.3 Do Disparity Map （視差圖）
                dispmap = stereo.compute(img_left_rectified, img_right_rectified)
                disp = dispmap.astype(np.float32)/16 #因為opencv 會把disparity放大16倍
                # Put nan (not a numner) for all unknown pixels (disparity below the minimum disparity 
                # disp[disp<stereo.getMinDisparity()] = np.nan
                if (FOR_DEBUG): #顯示disparity # 正常會看到近亮遠暗
                    #disp_show = cv2.normalize(disp,None,0,255,cv2.NORM_MINMAX)
                    #disp_show = np.uint8(disp_show)
                    # cv2.imshow("Disparity", disp_show)
                    cv2.imshow("Disparity", dispmap)
                # 4.4 Do Point Cloud
                points3d = cv2.reprojectImageTo3D(disp,self.Q) # 每個pixel points3d[v,u]會得到[X,Y,Z], 單位[m]
                if (FOR_DEBUG):
                    print(points3d.shape) #理論上會得到(360,640,3)
                # 去除無效點
                mask = disp > 0
                points = points3d[mask] # points: the point cloud points that is shape (N,3)
                # 上色
                colors = cv2.cvtColor(img_left_rectified,cv2.COLOR_GRAY2BGR)
                colors = colors[mask]

                # <For Visualization>
                # 4.5 Visualization 
                # Visualize Point Cloud 
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float32)/255.0) 
                o3d.visualization.draw_geometries([pcd]) 

                # Visualize Depth Image (深度圖) 
                z_depth = points3d[:,:,2]
                depth_show = z_depth.copy()
                depth_show[~mask] = 0# 去掉無效值
                depth_vis = cv2.normalize(depth_show,None,0,255,cv2.NORM_MINMAX) # normalize 顯示
                depth_vis = np.uint8(depth_vis)
                cv2.imshow("Depth Image", depth_vis)
                    

                

            
        elif (key == ord('q') or key ==ord('Q')):
            if (DO_EVALUATION): 
                if len(self.error_vector_left) > 0:
                    mean_error = np.mean(self.error_vector_left)
                    print("======================")
                    print("Number of images:",len(self.error_vector_left))
                    print("Mean Reprojection Error (Left):",mean_error,"pixel")
                    print("======================")
                else:
                    print("No left evaluation data")
                if len(self.error_vector_right) > 0:
                    mean_error = np.mean(self.error_vector_right)
                    print("======================")
                    print("Number of images:",len(self.error_vector_right))
                    print("Mean Reprojection Error (Right):",mean_error,"pixel")
                    print("======================")
                else:
                    print("No right evaluation data")

    # def image_callback_right(self, msg):
    #     #rospy.loginfo("Receive right image")
    #     #rospy.loginfo_throttle(1, "Receive right image")
    #     pass
    
    # <For Visualization>
    def add_title(self, img, title):

        h, w = img.shape[:2]
        # scale
        scale = 0.5 #50%

        # 如果是 grayscale，轉成 BGR
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        canvas = np.zeros((h+int(50*scale), w, 3), dtype=np.uint8)

        canvas[int(50*scale):, :] = img

        cv2.putText(
            canvas,
            title,
            (10,int(35*scale)),
            cv2.FONT_HERSHEY_SIMPLEX,
            1*scale, #字大小
            (255,255,255),
            2
        )

        return canvas

    def detectChessboardCorners(self, img, pattern_size, criteria):
        # Rough detection of the corners
        ret, corners = cv2.findChessboardCorners(img, pattern_size)    
        if(ret):
            # Refine the corners to subpixel precision
            corners = cv2.cornerSubPix(img, corners, (11,11), (-1,-1), criteria)        
            return corners, True
        else:
            return None, False
    def Check_Rectify(self, img_left_rectified, img_right_rectified): 

        ### Detect the left-most corners in the rectified left image and display the corresponding horizontal (epipolar) lines in the right image
        pattern_size = CHECKER_BOARD_PATTERN_SIZE
        criteria = (cv2.TERM_CRITERIA_EPS +cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
        
        corners_rectified_left, ret_left = self.detectChessboardCorners(img_left_rectified, pattern_size, criteria)
        corners_rectified_right, ret_right = self.detectChessboardCorners(img_right_rectified, pattern_size, criteria)
        debug_left_rect = cv2.cvtColor(img_left_rectified, cv2.COLOR_GRAY2BGR)
        debug_right_rect = cv2.cvtColor(img_right_rectified, cv2.COLOR_GRAY2BGR)
        # ================================
        # Draw rectified checkerboard result
        # ================================
        if ret_left and ret_right:
            left_edge_corners = corners_rectified_left[0::pattern_size[0]]
            right_edge_corners = corners_rectified_right[0::pattern_size[0]]
            Xl = []
            # Left rectified
            for corner in left_edge_corners:
                # x, y = corner[0,0], corner[0,1]
                x, y = corner[0]
                x = int(x)
                y = int(y)
                Xl.append(x)
                # draw point
                cv2.circle(debug_left_rect,(x,y),5,(0,255,0),2)

                # epipolar line
                cv2.line(debug_left_rect,(0,y),(self.image_size[0],y),(0,255,0),1)


            # Right rectified
            for i, corner in enumerate(right_edge_corners):

                x, y = corner[0]
                # x,y = corner[0,0], corner[0,1]
                x=int(x)
                y=int(y)
                cv2.circle(debug_right_rect,(x,y),5,(0,255,0),2)

                # epipolar line
                cv2.line(debug_right_rect,(0,y),(self.image_size[0],y),(0,255,0),1)

                # draw corresponding x position from left
                cv2.line(debug_right_rect,(Xl[i], y-10),(Xl[i], y+10),(0,0,255),2)

                # disparity text
                disparity = Xl[i]-x
                cv2.putText(debug_right_rect,f"{disparity:.1f}",(int((Xl[i]+x)/2),y-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)
            dy = abs(y - int(left_edge_corners[i][0][1]))

            print(
                f"corner {i}: left_y={left_edge_corners[i][0][1]:.2f}, "
                f"right_y={y}, dy={dy:.2f}"
            )
            return debug_left_rect, debug_right_rect
        else: 
            return None, None
            
    def evaluate_cam_instrinct(self, img, K, D, pattern_size=CHECKER_BOARD_PATTERN_SIZE, pattern_distance = CHECKER_BOARD_PATTERN_DISTANCE): 
         
        # pattern_size = (9,6) # 棋盤樣子
        # square_distance = 0.022 # 22mm一格
        n = pattern_size[0] #9
        m = pattern_size[1] #6

        # 建立真實3D棋盤座標, 得到各期盤點的(x,y,0)
        objp = np.zeros((n*m,3),np.float32)
        objp[:,:2] = np.mgrid[
            0:n,
            0:m
        ].T.reshape(-1,2)
        objp *= pattern_distance #22mm一格
        # 偵測已經畫面中的棋盤, 若找到了所有點, 那ret==true
        ret, corners = cv2.findChessboardCorners(
            img,
            pattern_size
        )
        # 讓找到的pixel到小數點第二位更細
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS +cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
            corners = cv2.cornerSubPix(img,corners,(11,11),(-1,-1),criteria)
            
            # 求棋盤相對相機位置rvec,tvec
            success,rvec,tvec = cv2.solvePnP(
                objp,
                corners,
                K,
                D
            )
            if not success: 
                return None
            # 投影回pixel
            projected,_ = cv2.projectPoints(
                objp,
                rvec,
                tvec,
                K,
                D
            )
            # 算error用方均根
            error = np.sqrt(
                np.mean(
                    np.sum(
                        (corners - projected)**2,
                        axis=2
                    )
                )
            )

            if(FOR_DEBUG==True): 
                draw = img.copy()
                cv2.drawChessboardCorners(draw,pattern_size,corners,ret)
                cv2.putText(draw,f"Reprojection Error: {error:.3f} px",(20,40),cv2.FONT_HERSHEY_SIMPLEX,1,255,2)
                cv2.imshow("Chessboard detection",draw)

            return error
        else: 
            print("fail board detection")
            return None

    def run(self):

        msg = String()
        msg.data = "Hello ROS1"
        self.pub.publish(msg)
        rospy.loginfo("Publish: %s",msg.data)

        

if __name__ == "__main__":

    rospy.init_node(
        "camera_node"
    )

    node = MyNode()

    rospy.spin()