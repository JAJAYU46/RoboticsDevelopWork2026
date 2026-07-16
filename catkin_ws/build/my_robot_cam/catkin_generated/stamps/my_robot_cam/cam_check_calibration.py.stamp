#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2

import numpy as np

FOR_DEBUG = True
# [Camera Instrincts]
# cal_final_campara 
#cal_kalibr
# Camera Matrix
fx_l, fy_l, cx_l, cy_l = [376.72365254, 376.73224047, 310.56993772, 188.22580516]
K_left = np.array([
    [fx_l,   0, cx_l],
    [   0,fy_l, cy_l],
    [   0,   0,    1]
], dtype=np.float64)

# Distortion Factor #OpenCV distortion 順序 [k1,k2,p1,p2,k3]
D_left = np.array([-0.00092227, -0.00535565, 0.00192026, -0.00056864], dtype=np.float64)




# #cal_test3
# # Camera Matrix
# K_left = np.array([
#     [662.975989,0,538.069450],
#     [0,656.146131,357.357752],
#     [0,0,1]
# ], dtype=np.float64)
# # Distortion Factor #OpenCV distortion 順序 [k1,k2,p1,p2,k3]
# D_left = np.array([0.005977,-0.028316,-0.001539,-0.040968,0], dtype=np.float64)


# cal_test2
# # Camera Matrix
# K_left = np.array([
# [732.826191,0,453.859918],
# [0,705.503816,323.094646],
# [0,0,1]
# ], dtype=np.float64)
# # Distortion Factor #OpenCV distortion 順序 [k1,k2,p1,p2,k3]
# D_left = np.array([0.198570,-0.456837,-0.007790,-0.098853,0], dtype=np.float64)

# cam_test1
# # Camera Matrix
# K_left = np.array([
#     [673.929818, 0.0, 507.397864],
#     [0.0, 669.409742, 342.695492],
#     [0.0, 0.0, 1.0]
# ], dtype=np.float64)
# # Distortion Factor #OpenCV distortion 順序 [k1,k2,p1,p2,k3]
# D_left = np.array([-0.007869, 0.131069, -0.004328, -0.062251, 0.0], dtype=np.float64)

class MyNode:

    def __init__(self):
        
        # Initialize for libraries
        self.bridge = CvBridge() #for opencv
        
        # subscriber undistord image, left, right
        self.sub_left = rospy.Subscriber("/mo_camera/left/image_raw",Image,self.image_callback_left,queue_size=1)
        self.sub_right = rospy.Subscriber("/mo_camera/right/image_raw",Image,self.image_callback_right,queue_size=1)

        # publisher
        self.pub = rospy.Publisher("/my_topic",String,queue_size=10)


        # For Evaluation
        self.current_img = None
        self.evaluate_flag = False
        self.error_vector = []

        rospy.loginfo("Start cam_check_calibration node")

    def image_callback_left(self, msg):
        #rospy.loginfo("Receive left image")
        #rospy.loginfo_throttle(1, "Receive left image")
        
        # ==============================



        # ==============================

        # 1. Original distored image
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "mono8")
        # 保存目前影像
        self.current_img = img.copy()

        # 2. Undistored by camera instrincts
        img_undistorted = cv2.undistort(img,K_left,D_left)

        # # 3. Evaluation 
        # error = self.evaluate_cam_instrinct(img_undistorted)
        # if(error != None): 
        #     print("Reprojection error:",error,"pixel")
        
        
        cv2.imshow("Left Raw Image", img)
        cv2.imshow("Left Undistored Image", img_undistorted)
        key = cv2.waitKey(1)

        if (key == 32): #按下空白鍵才做evaluation
            print("Capture image for evaluation")
            cv2.imshow("Captured Left Raw Image", img)
            cv2.imshow("Captured Left Undistored Image", img_undistorted)
            error = self.evaluate_cam_instrinct(img)
            if(error is not None): 
                print("Reprojection error:",error,"pixel")
                self.error_vector.append(error)
        elif (key == ord('q') or key ==ord('Q')):
            if len(self.error_vector) > 0:
                mean_error = np.mean(self.error_vector)
                print("======================")
                print("Number of images:",len(self.error_vector))
                print("Mean Reprojection Error:",mean_error,"pixel")
                print("======================")
            else:
                print("No evaluation data")

    def image_callback_right(self, msg):
        #rospy.loginfo("Receive right image")
        #rospy.loginfo_throttle(1, "Receive right image")
        pass
    def evaluate_cam_instrinct(self, img): 
        
        # 棋盤樣子
        pattern_size=(9,6)
        # 建立真實3D棋盤座標, 得到各期盤點的(x,y,0)
        objp = np.zeros((9*6,3),np.float32)
        objp[:,:2] = np.mgrid[
            0:9,
            0:6
        ].T.reshape(-1,2)
        objp *= 0.022 #22mm一格
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
                K_left,
                D_left
            )
            if not success: 
                return None
            # 投影回pixel
            projected,_ = cv2.projectPoints(
                objp,
                rvec,
                tvec,
                K_left,
                D_left
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