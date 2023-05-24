import cv2
import numpy as np
import serial
from MyRobot import MyRobot
import time
# def get_mm(workspace):
#     img_draw = workspace.copy()
#     img_draw_hsv = cv2.cvtColor(img_draw, cv2.COLOR_BGR2HSV)

#     orange_lower = np.array([0, 150, 150])
#     orange_upper = np.array([30, 255, 255])
#     x,y = position_color(img_draw_hsv, img_draw, "orange", orange_lower, orange_upper)

#     green_lower = np.array([40, 90, 90])
#     green_upper = np.array([70, 255, 255])
#     x,y = position_color(img_draw_hsv, img_draw, "green", green_lower, green_upper)

#     blue_lower = np.array([90, 125, 125])
#     blue_upper = np.array([100, 255, 255])
#     x,y = position_color(img_draw_hsv, img_draw, "blue", blue_lower, blue_upper)

#     pink_lower = np.array([145, 90, 90])
#     pink_upper = np.array([170, 255, 255])
#     x,y = position_color(img_draw_hsv, img_draw, "pink", pink_lower, pink_upper)

def get_mm_tranform(box):
    x = box[0]
    y = box[1]
    #670 370
    # x_max = 670
    # y_max = 370
    x_mm = (670-x)*145/670
    y_mm = (y)*100/370
    return [x_mm-70, y_mm+121, box[2]]


def perspective(contour):
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.02*peri, True)
    x, y, w, h = cv2.boundingRect(approx)
    pts = np.array([[0, 0], [750, 0], [0, 500], [750, 500]], dtype="float32")
    img_pts = np.array([[x,y], [x+w,y], [x,y+h], [x+w, y+h]], dtype="float32")

    M = cv2.getPerspectiveTransform(img_pts, pts)
    return M

def position_color(img,img_draw, color, lower, upper):
    box_list = []
    mask = cv2.inRange(img, lower, upper)
    ksize=3
    ker=np.ones((ksize,ksize),np.uint8)
    mask = cv2.dilate(mask,ker,iterations = 8)
    #show mask
    cv2.imshow(color, mask)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(img_draw, contours, -1, (0, 255, 0), 3)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
            x, y, w, h = cv2.boundingRect(approx)
            box_list.append([x,y,color])
            center = (x + w // 2, y + h // 2)
            cv2.putText(img_draw, color, (center[0]-20, center[1]-40), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
            cv2.circle(img_draw, (center[0], center[1]), 5, (0, 255, 0), cv2.FILLED)
            cv2.putText(img_draw, "x: "+str(center[0]), (center[0]-20, center[1]-20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(img_draw, "y: "+str(center[1]), (center[0]-20, center[1]+20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
    return box_list

# Webcam setup
cap = cv2.VideoCapture(1)  # Set the index to 0 for the default webcam

# Color detection parameters (adjust as needed)
lower_area = np.array([110, 50, 50])
upper_area = np.array([125, 255, 255])

# Initialize Zone    
J1=[0, 79, 0, 90]
J2=[0, 0, 100, 0]
J3=[0, 0, 140, 90]

# J1=[180, 79, 0, 90]
# J2=[0, 0, -100, 180]
# J3=[0, 0, -150, 0]

dh_params = [J1, J2, J3]
workspace = None
robot = MyRobot(dh_params)


while True:
    # Read the webcam feed
    ret, frame = cap.read()
    if not ret:
        continue
    
    boxes = []
    img_hsv= cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_hsv, lower_area, upper_area)
    # ksize=3

    # ker=np.ones((ksize,ksize),np.uint8)
    # mask = cv2.dilate(mask,ker,iterations = 8)
    #show mask
    cv2.imshow("area", mask)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 5000:
            m = perspective(cnt)
            workspace = cv2.warpPerspective(frame, m, (750, 500))

    if workspace is not None:
        img_draw = workspace.copy()
        img_draw_hsv = cv2.cvtColor(img_draw, cv2.COLOR_BGR2HSV)

        orange_lower = np.array([0, 150, 150])
        orange_upper = np.array([30, 255, 255])
        for i in position_color(img_draw_hsv, img_draw, "orange", orange_lower, orange_upper):
            boxes.append(i)

        green_lower = np.array([40, 90, 90])
        green_upper = np.array([70, 255, 255])
        for i in position_color(img_draw_hsv, img_draw, "green", green_lower, green_upper):
            boxes.append(i)

        blue_lower = np.array([90, 125, 125])
        blue_upper = np.array([100, 255, 255])
        for i in position_color(img_draw_hsv, img_draw, "blue", blue_lower, blue_upper):
            boxes.append(i)

        pink_lower = np.array([145, 90, 90])
        pink_upper = np.array([170, 255, 255])
        for i in position_color(img_draw_hsv, img_draw, "pink", pink_lower, pink_upper):
            boxes.append(i)

    if len(boxes) > 0:
        print(boxes)

    if robot.is_ready:
        if len(boxes) > 0:
            mission = get_mm_tranform(boxes[0])
            print("mission",mission)

            robot.start(mission)
            boxes.pop(0)

        else:
            pass



#670 370

    # Display the resulting frame
    cv2.imshow('frame', frame)
    # cv2.imshow("Image", img_draw)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and close the windows
cap.release()
cv2.destroyAllWindows()
