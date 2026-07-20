#!/usr/bin/env python3

import subprocess
import time
import os
import re
import signal

processes = [] # To remember what is started

# 此file在cd 到catkin_ws執行
# Set Files Location 
#Location_Kalibr_workspace = "../kalibr_workspace" #預設放在和catkin_ws 同層的資料夾
Location_Kalibr_workspace = "../../kalibr_workspace" #目前在他的上上層
Location_Calibration_folder = "src/my_robot_cam/src/dataset/cam_calibration" #預設放在此package的dataset中
#Location_Kalibr_to_Catkin = "../catkin_ws" #從kalibr_workspace到catkin_ws的相對路徑 #預設kalibr_workspace放在和catkin_ws 同層的資料夾
Location_Kalibr_to_Catkin = "../RoboticsDevelopWork2026/catkin_ws"

def start_calibration_system():
    global processes

    print("Starting camera calibration system...") 

    print("Creating Neccessary Files...") 
    # 先創好資料夾
    if not os.path.exists(Location_Calibration_folder):
        os.makedirs( Location_Calibration_folder,exist_ok=True) #沒有就創建, 有就不做
        print("Created:", Location_Calibration_folder)
    Location_rosbag = Location_Calibration_folder+"/data/recorded_rosbag"
    os.makedirs( Location_rosbag,exist_ok=True)
    Location_calibration_results = Location_Calibration_folder+"/data/calibration_results" #最後一次要的calibration結果會存在cal_final
    os.makedirs( Location_calibration_results,exist_ok=True)
    Location_calibration_results_cal_final = Location_Calibration_folder+"/data/calibration_results/cal_final" #最後一次要的calibration結果會存在cal_final
    os.makedirs( Location_calibration_results_cal_final,exist_ok=True)
    

    log_file = open(Location_Calibration_folder+"/camera.log", "w") #用來看運行時的隱藏log跟error

    # 建立checkerboard的target file for calibration
    Location_calibration_targets = Location_Calibration_folder+"/data/targets"
    os.makedirs( Location_calibration_targets,exist_ok=True)
    # 預設是9x6 22mm checkerboard
    target_file = os.path.join(Location_calibration_targets, "checkerboard_9x6.yaml")
    if not os.path.exists(target_file):
        print("Creating checkerboard target file...")
        content = (
            "target_type: 'checkerboard' #gridtype\n"
            "targetCols: 6               #number of internal chessboard corners\n"
            "targetRows: 9               #number of internal chessboard corners\n"
            "rowSpacingMeters: 0.022      #size of one chessboard square [m]\n"
            "colSpacingMeters: 0.022      #size of one chessboard square [m]\n")

        with open(target_file, "w") as f:
            f.write(content)
        print("Created:", target_file)

    while True: 
        print("""
        =====================================
                Camera Calibration Menu
        =====================================
        1. Start Camera 
        2. Record ROS Bag (Need to start Camera First)
        3. Camera Calibration 
        4. Stop Camera 
        """)
        
        
        cmd = input("Select: ") 
        
        
            

        if cmd == "1": 
            subprocess.Popen(
                """
                rqt_image_view /mo_camera/left/image_raw
                """, 
                shell=True,
                executable="/bin/bash"
            )
            print("Starting Camera, publishing image to /mo_camera/left/image_raw /mo_camera/right/image_raw")
            
            p = subprocess.Popen( # Open a Thread
                """
                source /opt/ros/noetic/setup.bash &&
                source devel/setup.bash && 
                rosrun moak_camera get_imu_sample 
                """, 
                shell = True, 
                executable = "/bin/bash", 
                #stdout=subprocess.DEVNULL, #隱藏輸出, 但仍會顯示error
                stdout=log_file,
                stderr=log_file
            )
            processes.append(p)

        elif(cmd == "2"): # Record ROS Bag
            print("Recording ROS bag from /mo_camera/left/image_raw /mo_camera/right/image_raw")
            p = subprocess.Popen(
                f"""
                source /opt/ros/noetic/setup.bash && 
                cd {Location_rosbag} && 
                rosbag record \
                /mo_camera/left/image_raw \
                /mo_camera/right/image_raw
                """, 
                shell=True,
                executable="/bin/bash", 
                preexec_fn=os.setsid
            )
            print("________________________________")
            print("Press 'q' to stop recording...")
            print("________________________________")
            input()
            os.killpg(os.getpgid(p.pid),signal.SIGINT) #即送ctrl+C停止
            print("ROS bag record stopped")

        elif(cmd == "3"): 
            print("Please Choose the rosbag to calibrate")
            selected_bag = None

            # 找所有 .bag 並顯示提供選擇
            bag_files = [
                f for f in os.listdir(Location_rosbag)
                if f.endswith(".bag")
            ]
            if len(bag_files) == 0:
                print("No rosbag found!")
                continue
            else: 
                print("\nAvailable rosbag:")
                print("====================")
                for i, bag in enumerate(bag_files):
                    print(f"{i+1}. {bag}")
                print("====================")
                choice = int(input("Select rosbag: "))
                selected_bag = bag_files[choice-1]

                print("Selected:")
                print(selected_bag)
            
            # 清空原來的final資料夾
            subprocess.run(
                f"""
                rm -rf "{Location_calibration_results_cal_final}"/*
                """,
                shell=True,
                executable="/bin/bash"
            )
            # 複製一份rosbag到calibration_results, 因為kalibr會把資料建在和rosbag同一層的資料夾
            subprocess.run(
                f"""
                cp {Location_rosbag}/{selected_bag} {Location_calibration_results_cal_final}/{selected_bag} 
                """, 
                shell=True,
                executable="/bin/bash"
            )
            
            
            print("Start Calibration") 
            location_bag_file_fromkalibrws = f"{Location_Kalibr_to_Catkin}/{Location_calibration_results_cal_final}/{selected_bag}"
            target_file_fromkalibrws = f"{Location_Kalibr_to_Catkin}/{target_file}"
            subprocess.run( #就不再背景執行 因為會卡住
                f"""
                cd {Location_Kalibr_workspace} && 
                export DISPLAY=:0 &&
                source /opt/ros/noetic/setup.bash &&
                source devel/setup.bash &&
                rosrun kalibr kalibr_calibrate_cameras     --bag {location_bag_file_fromkalibrws} --target {target_file_fromkalibrws}     --models pinhole-radtan pinhole-radtan     --topics /mo_camera/left/image_raw /mo_camera/right/image_raw
                """,
                # rosrun kalibr kalibr_calibrate_cameras     --bag /home/user/Documents/JaJaDocuments/RoboticsDevelopWork2026/catkin_ws/src/my_robot_cam/src/dataset/cam_calibration/data/calibration_results/cal_final/2026-07-17-14-50-17.bag --target /home/user/Documents/JaJaDocuments/RoboticsDevelopWork2026/catkin_ws/src/my_robot_cam/src/dataset/cam_calibration/data/targets/checkerboard_9x6.yaml     --models pinhole-radtan pinhole-radtan     --topics /mo_camera/left/image_raw /mo_camera/right/image_raw 
                shell=True,
                executable="/bin/bash"
            )

            # 備份資料結果到cal_test0. cal_test1...這樣下去
            
            # 依序編號cal_test_id
            max_id = -1
            for folder in os.listdir(Location_calibration_results):
                match = re.match(r"cal_test_(\d+)", folder)
                if match:
                    num = int(match.group(1))
                    max_id = max(max_id, num)
            new_test_id = max_id + 1
            Location_test_file = Location_calibration_results+f"/cal_test_{new_test_id}"
            os.makedirs( Location_test_file,exist_ok=True)
            # 把結果複製備份到cal_test_id 中
            subprocess.run(
                # 不複製bag
                f"""
                find "{Location_calibration_results_cal_final}" -maxdepth 1 -type f ! -name "*.bag" -exec cp {{}} "{Location_test_file}" \\;
                """, 
                shell=True,
                executable="/bin/bash"
            )
            record_file = os.path.join(Location_test_file, "used_bag.txt")

            with open(record_file, "w") as f:
                f.write(f"Used bag: {Location_calibration_results_cal_final}/{selected_bag}\n")

if __name__ == "__main__":
    start_calibration_system()


        

        # # 開新的terminal
        # subprocess.Popen(
        #     [
        #         "gnome-terminal",
        #         "--",
        #         "bash",
        #         "-c",
        #         "rosrun my_robot_cam cam_node"
        #     ]
        # )

