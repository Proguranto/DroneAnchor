import cv2
import numpy as np
from djitellopy import tello
import time
import simple_pid import PID

me = tello.Tello()
me.connect()
print(me.get_battery())

# start drone stream
me.streamon()

# get drone in the air
me.takeoff()
# send command to make
me.send_rc_control(0, 0, 25, 0)
time.sleep(3.0)

w, h = 360, 240
#w, h = 720, 480
# range to keep from person where [lower bound area, upper bound area]
# area because the bounding boxes are in rectangles
goalRange = [8000, 9800]

# pid = [proportional, integral, derivative]
pid = [0.4, 0, 0.4]
x_pid = PID(0.7, 0.0001, 0.1, setpoint=1, output_limits=(-40, 40))
y_pid = PID(0.7, 0.0001, 0.1, setpoint=1, output_limits=(-40, 40))



# previous error
prev_xError = 0
prev_yError = 0

def findFace(img):
    faceCascade = cv2.CascadeClassifier("Resources/haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.03, 4)

    myFaceListC = []
    myFaceListArea = []

    for (x, y, w, h) in faces:
        # draw bounding box
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # compute center point
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        # draw green dot for center of box
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
        # add current face to our lists
        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)

    # make sure we found at least one face
    if len(myFaceListArea) != 0:
        # return face with largest area
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        # no face detected
        return img, [[0, 0], 0]


def trackFace(info):
    area = info[1]
    x, y = info[0]
    # forward-backward movement
    forwardBackward = 0
    upDown = 0

    # Proportional Integral Derivative (PID) calculations
    xError = x - (w // 2)
    yError = y - (h // 2)
    # Also keeps image centered.
    # yaw velocity - rotation around the y-axis
    # speed = pid[0] * xError + pid[2] * (xError - prev_xError)
    speed = x_pid(xError)
    # clip to ensure speed isn't too high or low (avoid extremes)
    # speed = int(np.clip(speed, -100, 100))

    # upDown = pid[0] * yError + pid[2] * (yError - prev_yError)
    # upDown = int(np.clip(upDown, -100, 100))
    upDown = y_pid(yError)

    if area > goalRange[0] and area < goalRange[1]:
        forwardBackward = 0
    elif area > goalRange[1]:
        forwardBackward = -30
    elif area < goalRange[0] and area != 0:
        forwardBackward = 30

    # no face detected
    if x == 0 and y == 0:
        speed = 0
        upDown = 0
        xError = 0
        yError = 0


    print(xError, forwardBackward, upDown, speed)

    me.send_rc_control(0, forwardBackward, upDown // 3, speed // 2)

    return xError, yError


while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img, (w, h))
    img, info = findFace(img)
    prev_xError, prev_yError = trackFace(info)
    print("Center", info[0], "Area", info[1])
    cv2.imshow("Output", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        break

