import cv2
import apriltag

# ====== input image ======
img_path = "src/my_robot_cam/src/dataset/test_images/test_img2.png"

img = cv2.imread(img_path)

if img is None:
    raise Exception("Image not found")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


# ====== AprilTag detector ======
options = apriltag.DetectorOptions(
    families="tag36h11"
)

detector = apriltag.Detector(options)


# ====== detect ======
detections = detector.detect(gray)

print("Detected tags:", len(detections))


for det in detections:
    print("===================")
    print("ID:", det.tag_id)
    print("Center:", det.center)
    print("Corners:")
    print(det.corners)

    # draw result
    corners = det.corners.astype(int)

    for i in range(4):
        cv2.line(
            img,
            tuple(corners[i]),
            tuple(corners[(i+1)%4]),
            (0,255,0),
            2
        )

    cv2.circle(
        img,
        tuple(det.center.astype(int)),
        5,
        (0,0,255),
        -1
    )

    cv2.putText(
        img,
        str(det.tag_id),
        tuple(det.center.astype(int)),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255,0,0),
        2
    )


cv2.imshow("AprilTag detection", img)
cv2.waitKey(0)
cv2.destroyAllWindows()