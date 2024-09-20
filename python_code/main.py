from ardu_control import arm_control
from detection import camera
from multiprocessing import Process,Queue
import time
import cv2

# main function:
def main():
    ardu.moveTo(250,-250) # Robot moves to safe zone
    time.sleep(5)
    while True:
        # Step1:
        # Capture image and show it: (detection takes place inside view_frame())
        cam.capture_image()
        cam.img_preprocess()
        cam.view_frame()
        # cam.detect_frame()
        
        # Step1:
        # Detection of packages
        lconf = cam.listConf
        lcls = cam.listCls
        lcxcy = cam.listcxcy
        if (lconf == []):
            continue

        # Selection of target package
        destcls,destcxcy = cam.findDestcxcy(lconf,lcls,lcxcy)
        destcls = cam.model.names[int(destcls)]
        cam.listConf = []
        cam.listCls = []
        cam.listDepth = []
        cam.listcxcy = []

        # TC_O: getting transformation from camera to object:
        cx,cy = destcxcy
        cz = cam.getdepth(cx,cy)  # cx,cy,cz are in image frame
        TC_O = cam.Imgframe2Camframe(cx,cy,cz)  # TC_O -> homogeneous transformation

        # Object position from Camera frame to Robot Frame ()
        dx,dy,dz = cam.Camframe2Robotframe(TC_O)  # we get end-effector point in robot frame 
        if (not dx) or (not dy) or (not dz):
            continue

        # # Step3:
        ardu.moveTo(dx,dy)  # moving to target coordinate(x,y)
        if ardu.skip:
            continue
        ardu.zMoveTo(110) # for now z distance is fixed

        # # Step6:
        ardu.writeServo(140) # grasp

        # # Step7:
        ardu.zMoveTo(200)
        ardu.moveTo(250,-250)

        ardu.writeServo(0) # drop
        
        cv2.destroyAllWindows()
        # # Repeat the steps from 1 to 7 until end
        # # End of loop
    

# objects:
ardu = arm_control(serialport="COM6")
cam = camera()
time.sleep(5)    # time taken for the yolo model to load

# Flags:


if __name__ == '__main__':
    main()
    ardu.arduStop()


# Reference:
# 1. When arm_control() is executed: Connection is established, homing is done
# 2. camera() class contains the function related to capturing image, annotation, etc...