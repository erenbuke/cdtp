import cv2
import threading
import numpy as np
import time
import serial


class camThread(threading.Thread):
    def __init__(self, previewName, camID, ser, ser2):
        threading.Thread.__init__(self)
        self.previewName = previewName
        self.camID = camID
        self.ser = ser
        self.ser2 = ser2
    def run(self):
        print ("Starting " + self.previewName)
        camPreview(self.previewName, self.camID, self.ser, self.ser2)

def empty(a):
    pass

def camPreview(previewName, camID, ser, ser2):

    cv2.namedWindow(previewName)

    frameWidth = 640
    frameHeight = 480

    cam = cv2.VideoCapture(camID)

    '''cv2.namedWindow("HSV")
    cv2.resizeWindow("HSV", 640, 240)
    cv2.createTrackbar("R Min", "HSV", 0, 179, empty)
    cv2.createTrackbar("R Max", "HSV", 179, 179, empty)
    cv2.createTrackbar("G Min", "HSV", 0, 255, empty)
    cv2.createTrackbar("G Max", "HSV", 255, 255, empty)
    cv2.createTrackbar("B Min", "HSV", 0, 255, empty)
    cv2.createTrackbar("B Max", "HSV", 255, 255, empty)'''

    cam.set(3, frameWidth)
    cam.set(4, frameHeight)

    if cam.isOpened():  # try to get the first frame
        rval, frame = cam.read()
    else:
        rval = False

    count = 0

    while rval:
        cv2.imshow(previewName, frame)
        rval, frame = cam.read()

        frameHsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        '''h_min = cv2.getTrackbarPos("R Min", "HSV")
        h_max = cv2.getTrackbarPos("R Max", "HSV")
        s_min = cv2.getTrackbarPos("G Min", "HSV")
        s_max = cv2.getTrackbarPos("G Max", "HSV")
        v_min = cv2.getTrackbarPos("B Min", "HSV")
        v_max = cv2.getTrackbarPos("B Max", "HSV")'''

        redLower = np.array([169, 120, 46])
        redUpper = np.array([179, 191, 255])
        redMask = cv2.inRange(frameHsv, redLower, redUpper)
        redResult = cv2.bitwise_and(frame, frame, mask = redMask)

        greenLower = np.array([100, 80, 0])
        greenUpper = np.array([106, 175, 255])
        greenMask = cv2.inRange(frameHsv, greenLower, greenUpper)
        greenResult = cv2.bitwise_and(frame, frame, mask = greenMask)

        yellowLower = np.array([20, 100, 100])
        yellowUpper = np.array([30, 255, 255])
        yellowMask = cv2.inRange(frameHsv, yellowLower, yellowUpper)
        yellowResult = cv2.bitwise_and(frame, frame, mask = yellowMask)

        #cv2.imshow("asd", greenMask)
        #cv2.imshow("asd", yellowMask)

        if cv2.countNonZero(redMask) > 10:
            print("Rock on road " + str(camID))
            if (camID == 0):
                ser.write([1])
                ser2.write([1])
            else:
                ser2.write([4])
                ser.write([4])

        elif cv2.countNonZero(greenMask) > 50:
            print("Animal on road " + str(camID))
            if(camID == 0):
                ser.write([2])
                ser2.write([2])
            else:
                ser2.write([5])
                ser.write([5])

        elif cv2.countNonZero(yellowMask) > 10:
            if count == 2:
                print("Car on road " + str(camID))
                if (camID == 0):
                    ser.write([3])
                    ser2.write([3])
                else:
                    ser2.write([6])
                    ser.write([6])
            else:
                count = count + 1

        else:
            count = 0

        #cv2.imshow("Red Result", redMask)
        #cv2.imshow("Green Result", greenMask)
        #cv2.imshow("Yellow Result", yellowMask)

        key = cv2.waitKey(20)
        if key == 27:  # exit on ESC
            break

        #time.sleep(1)
    cam.release()
    cv2.destroyWindow(previewName)

# Create two threads as follows
ser = serial.Serial("com4", 9600)
ser2 = serial.Serial("com3", 9600)

thread1 = camThread("Camera 1", 0, ser, ser2)
thread2 = camThread("Camera 2", 1, ser, ser2)
thread1.start()
thread2.start()

'''
import serial  # Serial imported for Serial communication
import time  # Required to use delay functions

ArduinoSerial = serial.Serial('com3', 9600)
while(True):

    a = int(input("enter: "))

    ArduinoSerial.write([a])
'''