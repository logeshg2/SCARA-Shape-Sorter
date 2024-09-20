import time
import cv2
import numpy as np
from ardu_control import arm_control

# arduino object:
ardu = arm_control(serialport="COM6")
vid = cv2.VideoCapture(2)
time.sleep(4)

# start camera:
for i in range(25):
    x = np.random.randint(100,300)
    y = np.random.randint(-200,200)

    ardu.moveTo(x,y)
    time.sleep(5)

    _,frame = vid.read()
    cv2.imwrite(r"D:\Scara Code\hand_eye_calib_data\images\img{}.png".format(i),frame)

    TB_G = ardu.Base2Grip()
    np.savez(r"D:\Scara Code\hand_eye_calib_data\pose\pose{}.npz".format(i),TB_G=TB_G)

ardu.arduStop()