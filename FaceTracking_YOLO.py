import cv2
from djitellopy import tello
import time
from simple_pid import PID
import torch
import numpy as np

model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

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
# w, h = 720, 480
# range to keep from person where [lower bound area, upper bound area]
# area because the bounding boxes are in rectangles
# goalRange = [40000, 60000]
goalRange = [15000, 25000]

# pid = [proportional, integral, derivative]
pid = [0.4, 0, 0.4]
y_pid = PID(0.7, 0.0001, 0.1, setpoint=1, output_limits=(-40, 40))

# previous error
prev_xError = 0
prev_yError = 0


def findFace(img):
    results = model(img)

    myPersonListC = []
    myPersonListArea = []

    dataframe = results.pandas().xyxy[0]
    for i in range(dataframe.shape[0]):
        if dataframe.loc[i, "name"] == "person":
            # Get confidence
            confidence = dataframe.loc[i, "confidence"]
            print("confidence is ", confidence)
            if confidence > 0.6:
                # Gather coordinates.
                x_min, y_min, x_max, y_max = dataframe.loc[i, "xmin"], dataframe.loc[i, "ymin"], dataframe.loc[i, "xmax"], dataframe.loc[i, "ymax"]
                x_min = int(x_min)
                x_max = int(x_max)
                y_min = int(y_min)
                y_max = int(y_max)
                center_x = (x_max + x_min) // 2
                center_y = (y_max + y_min) // 2

                # draw bounding box
                cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)
                area = (x_max - x_min) * (y_max - y_min)
                cv2.circle(img, (center_x, center_y), 5, (0, 255, 0), cv2.FILLED)

                myPersonListC.append([center_x, center_y])
                myPersonListArea.append(area)


    # make sure we found at least one face
    if len(myPersonListArea) != 0:
        # return face with largest area
        i = myPersonListArea.index(max(myPersonListArea))
        return img, [myPersonListC[i], myPersonListArea[i]]
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
    speed = pid[0] * xError + pid[2] * (xError - prev_xError)
    # clip to ensure speed isn't too high or low (avoid extremes)
    speed = int(np.clip(speed, -100, 100))
    upDown = int(y_pid(yError))

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

    me.send_rc_control(0, forwardBackward, upDown // 2, speed // 2)

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

