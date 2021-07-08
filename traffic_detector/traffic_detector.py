import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import serial

# Communications Setup
ser = serial.Serial("/dev/ttyACM0", 9600)
ser.baudrate = 9600

# Parameters
BIN_THRESH = 50
CAR_THRESH = 10000
LOW_RED_1 = np.array([170,50,50])
HIGH_RED_1 = np.array([180,255,255])
LOW_RED_2 = np.array([0,50,50])
HIGH_RED_2 = np.array([10,255,255])
tot_area = 0
car_count = 0
t=0
cooldown = False
sent = False


if __name__ == "__main__":
    ref = None
    camera = PiCamera()
    camera.resolution = (640,480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size = (640,480))
    time.sleep(0.1)
                       
    cv2.namedWindow("frame1")
    cv2.namedWindow("frame2")

    firstTime = True
    for frameCap in camera.capture_continuous(rawCapture, format = "bgr", use_video_port = True):
        
        # Grab Webcam image and values from trackbar
        frame = frameCap.array
        if firstTime:
            ref = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            firstTime = False
            rawCapture.truncate(0)
            rawCapture.seek(0)
            continue
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, LOW_RED_1, HIGH_RED_1)
        mask2 = cv2.inRange(hsv, LOW_RED_2, HIGH_RED_2)
        mask = mask1+mask2
        masked = cv2.bitwise_and(frame,frame,mask=mask)
        
        greyed = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY) # Convert to Greyscale
        thresh = cv2.threshold(greyed, BIN_THRESH, 255, cv2.THRESH_BINARY)[1]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
        prev_area = tot_area
        tot_area = 0
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            tot_area += area
        
        if tot_area - prev_area > CAR_THRESH and not cooldown:
            cooldown = True
            sent = False
            t=0
            car_count+=1
            send_char = b'%d' %car_count
            ser.write(send_char)
        
        if tot_area < CAR_THRESH:
            car_count = 0
            if not sent:
                send_char = b'%d' %car_count
                ser.write(send_char)
                sent = True
        
        if cooldown:
            t+=1
            if t==10:
                cooldown = False

        cv2.putText(frame, "Cars Waiting: {}".format(str(car_count)), (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (250, 0, 1), 2)
            
        
        
        cv2.imshow("frame1", frame)
        cv2.imshow("frame2", opened)
        
        rawCapture.truncate(0)
        rawCapture.seek(0)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        
        
    #cam.release()
    cv2.destroyAllWindows()

