import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import copy
from std_msgs.msg import String

model = YOLO('yolov8n.pt')
count = 100

traffic_msg = 'none'



def compressed_image_callback(msg):

    global traffic_msg

    np_arr = np.frombuffer(msg.data, np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # cv2.imwrite('image.jpg', img_bgr)

    results = model.predict(img_bgr)
    print(results[0])
    print()
    print(results[0].boxes)
    num = results[0].boxes.cls.size(0)

    is_exist_traffic_light = False
    x, y, w, h = 0, 0, 0, 0

    for i in range(num):
        if results[0].boxes.cls[i] == 9.:
            now_w = results[0].boxes.xywh[i][2]
            now_h = results[0].boxes.xywh[i][3]
            if now_w > now_h and w*h <= now_w*now_h:
                is_exist_traffic_light = True
                x = int(results[0].boxes.xywh[i][0])
                y = int(results[0].boxes.xywh[i][1])
                w = int(now_w)
                h = int(now_h)

    
    res_plotted = results[0].plot()
    print(type(res_plotted))

    cv2.imwrite('result.jpg', res_plotted)
    

    lower_red1 = np.array([0, 0, 0])
    upper_red1 = np.array([10, 255 ,255])
    lower_red2 = np.array([165, 0, 0])
    upper_red2 = np.array([179, 255 ,255])

    lower_green = np.array([36, 50, 50])
    upper_green = np.array([86, 255, 255])

    if is_exist_traffic_light:
        crop_img = res_plotted[y-h//2:y+h//2, x-w//2:x+w//2]
        cv2.imwrite('traffic_light.jpg', crop_img)
        crop_img_hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        crop_green_img = cv2.inRange(crop_img_hsv, lower_green, upper_green)
        cv2.imwrite('traffic_light_green.jpg', crop_green_img)
        crop_red_img1 = cv2.inRange(crop_img_hsv, lower_red1, upper_red1)
        crop_red_img2 = cv2.inRange(crop_img_hsv, lower_red2, upper_red2)
        crop_red_img = cv2.bitwise_or(crop_red_img1, crop_red_img2)
        cv2.imwrite('traffic_light_red.jpg', crop_red_img)
    cv2.imshow("Pedes detection Cam", res_plotted)
    cv2.waitKey(1) 
    print(crop_img.shape)
    print(crop_green_img.shape)
    print(crop_red_img.shape)

    area_green = 0
    area_red = 0
    area = crop_img.shape[0]*crop_img.shape[1]

    for i in range(crop_img.shape[0]):
        for j in range(crop_img.shape[1]):
            tmp_red = crop_red_img[i][j]
            tmp_green = crop_green_img[i][j]
            if tmp_red != 0:
                area_red += 1
            if tmp_green != 0:
                area_green += 1
    
    if area_green*15 < area:
        area_green = 0
    if area_red*30 < area:
        area_red = 0
    
    if area_red > area_green:
        traffic_msg = "red"
    elif area_green > area_red:
        traffic_msg = "green"
    else:
        traffic_msg = "none"

    traffic_pub.publish(traffic_msg)


def main():
    rospy.init_node('compressed_to_image_converter', anonymous=True)
    
    # Subscribe to the compressed image topic
    rospy.Subscriber('/image_jpeg/compress', CompressedImage, compressed_image_callback)
    
    # Create a publisher for the Image message
    # global image_pub
    global traffic_pub
    traffic_pub = rospy.Publisher('/traffic_signal', String, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    
    main()