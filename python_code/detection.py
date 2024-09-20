import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
from ultralytics import YOLO
from realsense_depth import *
from ultralytics.utils.plotting import Annotator 

class camera():
    def __init__(self):
        self.c_frame = self.d_frame = None
        self.new_frame = False
        # self.vid = cv2.VideoCapture(0)  # Get video frame  # webcam
        self.vid = DepthCamera()  # Depth camera instance
        # self.vid.set(cv2.CAP_PROP_FRAME_WIDTH,848)  # resize the frame
        # self.vid.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        self.model = YOLO("./yolo_weights/best_shape.pt")

        file = np.load("intrinsic_params.npz")
        self.cameraIntrinsics = (file['arr_0'])
        # Lists:  --> **** These are detection done in one instance / frame (So - when needed in main loop) *****
        # Note: --> The index's are respestive to each other
        self.listConf = []
        self.listCls = []
        self.listDepth = []
        self.listcxcy = []

    
    def capture_image(self):
        check_frame,depth_frame,self.c_frame,self.d_frame = self.vid.get_frame()
        if (not check_frame):
            print("No input image")

    def detect_frame(self):  # YOLO detection
        annotator = Annotator(self.c_frame)
        for result in self.model.predict(source=self.c_frame,stream=True):#,show=True):
            for box in result.boxes:
                b = box.xyxy[0]
                conf = round(float(box.conf),2)
                cls = box.cls
                if conf > 0.7:
                    x1,y1,x2,y2=b
                    (cx,cy) = (int((x1+x2)/2),int((y1+y2)/2))
                    self.listConf.append(conf)
                    self.listCls.append(cls)
                    self.listcxcy.append((cx,cy))   # stored as -> [(c1,c2),(c3,c4),...]
                    self.listDepth.append((self.getdepth(cx,cy)))
                    annotator.box_label(b, self.model.names[int(cls)],(0,0,255),(0,255,0))
                    self.c_frame = annotator.result()

    def view_frame(self):
        self.detect_frame()
        # cv2.circle(self.c_frame,(320,240),1,(0,0,255),2)   # Centre point of the screen
        cv2.imshow("OUTPUT",self.c_frame)
        # plt.imshow(self.c_frame,cmap="gray")
        key = cv2.waitKey(1)
    
    # Preprocess input image:
    def img_preprocess(self):
        self.c_frame = cv2.cvtColor(self.c_frame,cv2.COLOR_BGR2GRAY)  # current model requires grayscale image for detection
        
        frame = np.zeros_like(self.c_frame)
        frame[120:400,170:480] = self.c_frame[120:400,170:480]  # croping the ROI from c_frame

        self.c_frame = np.stack((frame,)*3, axis=-1) # yolo accepts only this format of input with 3 channels

        # Undistortion of image:
        # use opencv undistortion in future, the current camera (intel realsense D435i) do not need it


    def getdepth(self,x,y):  # depth of point (x,y)
        depth = self.d_frame.get_distance(x,y)  # in meters(m)
        return (depth * 1000) # output is converted into mm
    

    def findDestcxcy(self,lco,lcs,lxy): # this function is used to find the destination objects cx,cy
        if (len(lco) == 0):
            return
        max_of_conf = max(lco)
        index_of_maxconf = lco.index(max_of_conf)
        return lcs[index_of_maxconf],lxy[index_of_maxconf]


    def Imgframe2Camframe(self,u,v,z):   # this function converts the point from image coord frame (2D) to camera coord frame (3D) 
        # u and v are pixel points 
        # z is the depth from intel on the praticular (u,v) point in image
        
        fx = self.cameraIntrinsics[0,0]
        fy = self.cameraIntrinsics[1,1]
        cx = self.cameraIntrinsics[0,2]
        cy = self.cameraIntrinsics[1,2]

        # dist_coeffs = np.array([-0.005879508354853155, -1.3178281516285262, 0.20336267909523562, 0.035747941135932895, 2.1934808430935084])
        # undistorted = cv2.undistortPoints(np.array([[u, v]], dtype=np.float32), cameraIntrinsics, dist_coeffs)
        # u, v = undistorted[0, 0, :]

        # 2D to 3D: (Perspective Projection)
        X_c = (u - cx) * z / fx
        Y_c = (v - cy) * z / fy
        Z_c = z

        TC_O = np.eye(4,4)
        TC_O[0:3,3] = (X_c,Y_c,Z_c)

        return TC_O  # transformation of point in camera frame

    def Camframe2Robotframe(self,TC_O): # dummy function (original is in ardu_control)
        # TC_O is a 4x4 matrix
        # file = np.load("TC_B.npz")
        # TC_B = np.round(file['arr_0'])    # TC_B -> transformation from camera to base (Hand-Eye calibration)

        # for now i consider TC_B == TB_C
        # TB_C = TC_B #linalg.inv(TC_B)

        # For now I have just manually measured the TB_C matrix: ( *** IMP ***)
        TB_C = np.eye(4,4)
        TB_C[0:3,3] = [290,-20,530] # in mm (not calibrated)

        rot = [[-0,1,0],[1,0,-0],[-0,0,-1]]
        TB_C[0:3,0:3] = rot

        # homogeneous transformation from base to object(point) in image
        TB_O = np.dot(TB_C,TC_O)
        # print(TB_O)
        (x,y,z) = TB_O[0:3,3]
        return (x,y,z)

    # ***IMP***
    # cls-> ["circle","pentagon","rectangle","square","triangle"]
    # conf-> [0.7 to 1]