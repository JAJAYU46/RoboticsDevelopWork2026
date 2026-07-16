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
DO_EVALUATION = True # check instrinct (camera matrix and distortion factor)
DO_POINTCLOUD = True # check extrinct (camera matrix and Baseline)

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

R = r.as_dcm() # <Note> For newer scipy version is .as_matrix() #3x3 numpy array, Rotation Matrix
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


        # For Evaluation
        self.current_img_left = None
        self.current_img_right = None

        # self.evaluate_flag = False
        self.error_vector_left = []
        self.error_vector_right = []

        rospy.loginfo("Start cam_check_calibration node")
    

    def stereo_callback(self, msg_left, msg_right):
        
        # 取得 timestamp
        time_left = msg_left.header.stamp
        time_right = msg_right.header.stamp
        rospy.loginfo(
            "left: %f right: %f diff: %f",
            time_left.to_sec(),
            time_right.to_sec(),
            abs(time_left.to_sec()-time_right.to_sec())
        )
        
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
                pass# 4.1 Do Rectify 


            
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