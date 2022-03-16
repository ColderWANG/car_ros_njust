import cv2.cv2 as cv2
import numpy as np
from copy import deepcopy
import math


class c_red_flag_t():
    def __init__(self, source):
        self.cap = cv2.VideoCapture(source)
        self.debug = False
        self.mat_debug = None
        self.flag_pos = None

    def MouseEvent(self, event, x, y, flags, param):
        if self.mat_debug is not None:
            print(self.mat_debug[y][x])

    def debug_window(self):
        self.debug = True
        cv2.namedWindow("debug")
        cv2.setMouseCallback('debug', self.MouseEvent)

    def HSV_range(self, H_range, S_range, V_range, mat_hsv):
        mat_hsv = deepcopy(mat_hsv)
        H, S, V = cv2.split(mat_hsv)
        AOI = deepcopy(H)
        AOI[AOI != 255] = 255

        self.H = H
        self.S = S
        self.V = V

        AOI[H < H_range[0]] = 0
        AOI[H > H_range[1]] = 0

        AOI[S < S_range[0]] = 0
        AOI[S > S_range[1]] = 0

        AOI[V < V_range[0]] = 0
        AOI[V > V_range[1]] = 0
        return AOI

    def next_frame(self):
        speed = 0.0
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            AOI1 = self.HSV_range([160, 200], [160, 255], [150, 255], frame_hsv)
            AOI2 = self.HSV_range([0, 4], [100, 255], [150, 255], frame_hsv)
            AOI3 = self.HSV_range([0, 7], [100, 255], [150, 255], frame_hsv)
            AOI1[AOI2 > 0] = 255
            AOI1[AOI3 > 0] = 255
            AOI = AOI1

            AOI = cv2.erode(AOI, np.ones((3, 3)))
            AOI = cv2.dilate(AOI, np.ones((10, 10)))

            retval, labels, stats, centroids = cv2.connectedComponentsWithStats(AOI, connectivity=8)
            find = False
            new_pos = None
            for i in range(retval):
                if stats[i][4] < 100000 and stats[i][4] > 100:
                    find = True
                    new_pos = centroids[i]
                if find:
                    break
            if find:
                if new_pos is not None and self.flag_pos is not None:
                    xerr = new_pos[1] - self.flag_pos[1]
                    yerr = new_pos[0] - self.flag_pos[0]
                    speed = math.sqrt(xerr * xerr + yerr * yerr)
            self.flag_pos = new_pos

            if self.debug is True:
                self.mat_debug = frame_hsv
                if self.flag_pos is not None:
                    frame = cv2.circle(frame, (int(self.flag_pos[0]), int(self.flag_pos[1])), 20, (0, 255, 0), -1)
                cv2.imshow("debug", frame)
                cv2.imshow("debugAOI", AOI)
                cv2.waitKey(33)
        return speed

    def release(self):
        if self.cap.isOpened():
            self.cap.release()
