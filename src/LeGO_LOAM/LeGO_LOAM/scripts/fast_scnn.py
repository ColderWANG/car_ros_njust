#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import threading
import numpy as np
import cv2
from openvino.inference_engine import IECore

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread, Lock

# (.xml and .bin files) or (.onnx file)
model_xml = "/home/w/car_ros/src/LeGO-LOAM/LeGO-LOAM/scripts/model.xml"
model_bin = "/home/w/car_ros/src/LeGO-LOAM/LeGO-LOAM/scripts/model.bin"
device = 'CPU' # 设备
# ---------------------------Step 1. Initialize inference engine core--------------------------------------------------
ie = IECore()

# ---------------------------Step 2. Read a model in OpenVINO Intermediate Representation or ONNX format---------------

# net = ie.read_network(model=args.model)
net = ie.read_network(model=model_xml)

# ---------------------------Step 3. Configure input & output----------------------------------------------------------
# Get names of input and output blobs
input_blob = next(iter(net.input_info))
out_blob = next(iter(net.outputs))

# Set input and output precision manually
net.input_info[input_blob].precision = 'FP32'
net.outputs[out_blob].precision = 'FP16'

# ---------------------------Step 4. Loading model to the device-------------------------------------------------------
exec_net = ie.load_network(network=net, device_name=device)

mean = (0.485, 0.456, 0.406)  # city, rgb
std = (0.229, 0.224, 0.225)

def fast_scnn(image):
    # timestart = cv2.getTickCount()

    # image = cv2.imread(image)
    image = image.copy()[:, :, ::-1]
    # image = img.copy()[:, :, ::-1]  # cvtColor

    image = image.transpose(2, 0, 1)
    image = np.ascontiguousarray(image, dtype=np.float32)
    for i in range(image.shape[0]):
        image[i, :, :] = (image[i, :, :] / 255 - mean[i]) / std[i]
    res = exec_net.infer(inputs={input_blob: image})
    res = res[out_blob]
    res = np.argmax(res, 1).squeeze(0)
    palette = np.random.randint(0, 256, (256, 3), dtype=np.uint8)
    np.random.seed(123)
    res = palette[res]
    return res
    # cv2.imwrite('./openvino_res.jpg', res)
    # timeend = cv2.getTickCount()
    # timeelapse = (timeend - timestart) / cv2.getTickFrequency()
    # fps = 1. / timeelapse
    # print("fps = %s; infer time = %s ms" % (fps, timeelapse * 1000))

class Image_Processer:
    def __init__(self):
        self.mutex = Lock()
        self.ImgSub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.ImgPub = rospy.Publisher("/image_aftSeg", Image, queue_size = 1)
        self.newimg = False
    def callback(self,data):
        try:
            if self.mutex.acquire(False):
                self.cv_image =self.bridge.imgmsg_to_cv2(data,"bgr8")
                self.newimg = True
                self.mutex.release()
        except CvBridgeError as e:
            print(e)
    def run(self):
        if self.newimg:
            self.mutex.acquire()
            Seg_img = fast_scnn(self.cv_image)
            self.newimg = False
            self.mutex.release()
            self.ImgPub.publish(self.bridge.cv2_to_imgmsg(Seg_img,"bgr8"))
    def Sub_thread(self):
        rospy.spin()

if __name__ == '__main__':
    #fast_scnn("./0-172.jpg")
    rospy.init_node('fast_scnn')
    rospy.loginfo('\033[1;32m---->\033[0m visualizatoin Started.')
    IP = Image_Processer()
    thread = Thread(target = IP.Sub_thread , args = "Sub_Img_thread")
    while not rospy.is_shutdown():
        IP.run()
        

    
    
    
