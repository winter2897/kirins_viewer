## Form implementation generated from reading ui file 'dev.ui'
## pyuic5 dev.ui -o test.py
## pyrcc5 resource.qrc -o resource_rc.py

## PyQT5 
import resource_rc
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import  QPixmap, QPainter
from PyQt5.QtWidgets import  QWidget, QApplication
from PyQt5.QtCore import QThread, Qt, pyqtSignal, QPoint, QRect, pyqtSlot

import sys
import cv2
import time
from math import sqrt
import numpy as np

## Socket
import msg
import socket
import pickle
import ViscaPacketsEnum
from  threading import Thread

## Address of sever socket
HOST_IP = '192.168.0.215'
PORT = 6969

## Stream from deepstream
RGB_STREAM = 'rtsp://192.168.0.215:8554/stream0'
THERMAL_STREAM = 'rtsp://192.168.0.215:8555/stream1'

## Global variable
curr_camera = 'rgb'
drawing_SOT_outbox = False
drawing_detectbox = False

class RGB_Stream(QThread):  
    # Signal to change frame in pixmap
    changePixmap = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self._run_flag = True

    def run(self):
        cap = cv2.VideoCapture(gst_pipeline(RGB_STREAM), cv2.CAP_GSTREAMER)

        while self._run_flag:
            QtWidgets.QApplication.processEvents
            ret, frame = cap.read()

            if ret:
                # Check if resolution of stream is not equal 1280x720 then resize it
                if frame.shape[0] != 720:
                    frame = cv2.resize(frame,(1280,720))
                self.changePixmap.emit(frame)

        # Shut down capture system
        cap.release()

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()

class Thermal_Stream(QThread):  
    # Signal to change frame in pixmap
    changePixmap = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self._run_flag = True

    def run(self):
        cap = cv2.VideoCapture(gst_pipeline(THERMAL_STREAM), cv2.CAP_GSTREAMER)

        while self._run_flag:
            QtWidgets.QApplication.processEvents
            ret, frame = cap.read()

            if ret:
                # Check if resolution of stream is not equal 1280x720 then resize it
                if frame.shape[0] != 720:
                    frame = cv2.resize(frame,(1280,720))
                self.changePixmap.emit(frame)

        # Shut down capture system
        cap.release()

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()

class MainWindow(QWidget):
    def __init__(self, parent = None):
        super(MainWindow, self).__init__(parent)

        ## Get resolution of screen window 
        # screen = app.primaryScreen()
        # rect = screen.availableGeometry()
        # size = screen.size()
        # print('Size of Screen: %d x %d' % (size.width(), size.height())) 
        # self.win_width = rect.width()
        # self.win_height = rect.height() 
        self.win_width = 1280
        self.win_height = 720

        ## Initial resolution of stream
        self.space = 10 # pixels
        self.video_height = int(self.win_height)
        self.video_width = int(self.video_height*1280/720) # 1280 x 720
        self.scale = 720/self.video_height # raw video's pixel density is 1280 x 720
        print('Size of Stream: %d x %d' % (self.video_width, self.video_height))

        #####------------------------------------------------------#####
        #####                       MAIN WINDOW                    #####
        #####------------------------------------------------------#####
        self.setGeometry(0, 0, self.win_width, self.win_height)
        self.setWindowTitle("Kirins View")
        self.setWindowIcon(QtGui.QIcon("icon/system/logo_kirins.png"))
        self.move(300,150)
        self.setFixedWidth(self.win_width)
        self.setFixedHeight(self.win_height)

        #####------------------------------------------------------#####
        #####                        STREAM                        #####
        #####------------------------------------------------------#####
        self.stream_frame = QtWidgets.QFrame(self)
        self.stream_frame.setGeometry(QtCore.QRect(0, 0, 1280, 720))
        self.stream = QtWidgets.QLabel(self.stream_frame)
        self.stream.setGeometry(QtCore.QRect(0, 0, 1280, 720))
        
        # Dual mode
        self.rgb_stream = QtWidgets.QLabel(self.stream_frame)
        self.rgb_stream.setGeometry(QtCore.QRect(0, 120, 640, 480))
        self.thermal_stream = QtWidgets.QLabel(self.stream_frame)
        self.thermal_stream.setGeometry(QtCore.QRect(640, 120, 640, 480))

        #####------------------------------------------------------#####
        #####                          BUTTON                      #####
        #####------------------------------------------------------#####

        ## Initial mode status
        self.lock_gimbal = False
        self.curr_monitor_mode = '2160p/29.97'
        self.curr_wb_mode = 'Auto'
        self.curr_focus_mode = 'Auto'
        self.curr_eps_mode = 'Full Auto'

        self.drawing_point = False
        self.drawing_SOT_inbox = False
        self.loading_model = False

        self.curr_autozoom = False
        self.curr_stabilize = False
        self.curr_defog = False
        self.curr_record = False
        self.curr_backlight = False
        self.curr_sensitivity = False

        self.curr_page = 0
        self.curr_inspection = 'COCO'

        # Initial bar value
        self.red_gain_val = 0
        self.blue_gain_val = 0
        self.focus_manual_val = 0
        self.gain_val = 1
        self.iris_val = 0
        self.shutter_val = 6

        ## horizontalLayout_7 includes reset, focus, EPS, setting button on top right of window
        self.layoutWidget_5 = QtWidgets.QWidget(self)
        self.layoutWidget_5.setGeometry(QtCore.QRect(1010, 10, 265, 58))
        self.layoutWidget_5.setObjectName("layoutWidget_5")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.layoutWidget_5)
        self.horizontalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_7.setSpacing(5)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.reset_button = QtWidgets.QPushButton(self.layoutWidget_5)
        self.reset_button.setEnabled(True)
        self.reset_button.setAutoFillBackground(False)
        self.reset_button.setStyleSheet("")
        self.reset_button.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/kirins/icon/system/reset.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.reset_button.setIcon(icon)
        self.reset_button.setIconSize(QtCore.QSize(50, 50))
        self.reset_button.setCheckable(False)
        self.reset_button.setChecked(False)
        self.reset_button.setFlat(True)
        self.reset_button.setObjectName("reset_button")
        self.reset_button.setStyleSheet("background-color: transparent;")
        self.horizontalLayout_7.addWidget(self.reset_button)
        self.focus_button = QtWidgets.QPushButton(self.layoutWidget_5)
        self.focus_button.setEnabled(True)
        self.focus_button.setAutoFillBackground(False)
        self.focus_button.setStyleSheet("")
        self.focus_button.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/kirins/icon/system/focus.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon1.addPixmap(QtGui.QPixmap(":/kirins/icon/system/focus_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.focus_button.setIcon(icon1)
        self.focus_button.setIconSize(QtCore.QSize(50, 50))
        self.focus_button.setCheckable(True)
        self.focus_button.setChecked(False)
        self.focus_button.setFlat(True)
        self.focus_button.setObjectName("focus_button")
        self.focus_button.setStyleSheet("background-color: transparent;")
        self.horizontalLayout_7.addWidget(self.focus_button)
        self.EPS_button = QtWidgets.QPushButton(self.layoutWidget_5)
        self.EPS_button.setEnabled(True)
        self.EPS_button.setAutoFillBackground(False)
        self.EPS_button.setStyleSheet("background-color: transparent;")
        self.EPS_button.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(":/kirins/icon/system/EPS.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon2.addPixmap(QtGui.QPixmap(":/kirins/icon/system/eps_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.EPS_button.setIcon(icon2)
        self.EPS_button.setIconSize(QtCore.QSize(50, 50))
        self.EPS_button.setCheckable(True)
        self.EPS_button.setChecked(False)
        self.EPS_button.setFlat(True)
        self.EPS_button.setObjectName("EPS_button")
        self.horizontalLayout_7.addWidget(self.EPS_button)
        self.setting_button = QtWidgets.QPushButton(self.layoutWidget_5)
        self.setting_button.setEnabled(True)
        self.setting_button.setAutoFillBackground(False)
        self.setting_button.setText("")
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(":/kirins/icon/system/setting.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon3.addPixmap(QtGui.QPixmap(":/kirins/icon/system/setting_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.setting_button.setIcon(icon3)
        self.setting_button.setIconSize(QtCore.QSize(50, 50))
        self.setting_button.setCheckable(True)
        self.setting_button.setChecked(False)
        self.setting_button.setFlat(True)
        self.setting_button.setObjectName("setting_button")
        self.setting_button.setStyleSheet("background-color: transparent;")
        self.horizontalLayout_7.addWidget(self.setting_button)

        ## horizontalLayout_8 includes RNG, ASL, LON, LAT on bottom left of window
        self.layoutWidget = QtWidgets.QWidget(self)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 570, 1261, 131))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_8.setSpacing(5)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.gridLayout_6 = QtWidgets.QGridLayout()
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.RNG_value = QtWidgets.QLabel(self.layoutWidget)
        self.RNG_value.setObjectName("RNG_value")
        self.RNG_value.setText("60 m")
        self.gridLayout_6.addWidget(self.RNG_value, 1, 1, 1, 1)
        self.LON_value = QtWidgets.QLabel(self.layoutWidget)
        self.LON_value.setObjectName("LON_value")
        self.LON_value.setText("LON")
        self.gridLayout_6.addWidget(self.LON_value, 2, 2, 1, 2)
        self.ASL = QtWidgets.QLabel(self.layoutWidget)
        self.ASL.setObjectName("ASL")
        self.ASL.setText("ASL") 
        self.gridLayout_6.addWidget(self.ASL, 1, 2, 1, 1)
        self.RNG = QtWidgets.QLabel(self.layoutWidget)
        self.RNG.setObjectName("RNG")
        self.RNG.setText("RNG")
        self.gridLayout_6.addWidget(self.RNG, 1, 0, 1, 1)
        self.ASL_value = QtWidgets.QLabel(self.layoutWidget)
        self.ASL_value.setObjectName("ASL_value")
        self.ASL_value.setText("1 m")
        self.gridLayout_6.addWidget(self.ASL_value, 1, 3, 1, 1)
        self.LAT_value = QtWidgets.QLabel(self.layoutWidget)
        self.LAT_value.setObjectName("LAT_value")
        self.LAT_value.setText("LAT")
        self.gridLayout_6.addWidget(self.LAT_value, 2, 0, 1, 2)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_6.addItem(spacerItem, 0, 0, 1, 1)
        self.horizontalLayout_8.addLayout(self.gridLayout_6)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem1)

        ## self.layoutWidget includes audio, navi, mount button on center bottom of window
        self.audio_button = QtWidgets.QPushButton(self.layoutWidget)
        self.audio_button.setEnabled(True)
        self.audio_button.setAutoFillBackground(False)
        self.audio_button.setStyleSheet("background-color: transparent;")
        self.audio_button.setText("")
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(":/kirins/icon/system/audio.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon4.addPixmap(QtGui.QPixmap(":/kirins/icon/system/audio_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.audio_button.setIcon(icon4)
        self.audio_button.setIconSize(QtCore.QSize(50, 50))
        self.audio_button.setCheckable(True)
        self.audio_button.setChecked(False)
        self.audio_button.setFlat(True)
        self.audio_button.setStyleSheet("background-color: transparent;")
        self.audio_button.setObjectName("audio_button")
        self.horizontalLayout_8.addWidget(self.audio_button, 0, QtCore.Qt.AlignBottom)
        self.mount_button = QtWidgets.QPushButton(self.layoutWidget)
        self.mount_button.setEnabled(True)
        self.mount_button.setAutoFillBackground(False)
        self.mount_button.setStyleSheet("background-color: transparent;")
        self.mount_button.setText("")
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap(":/kirins/icon/system/mount.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon5.addPixmap(QtGui.QPixmap(":/kirins/icon/system/mount_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.mount_button.setIcon(icon5)
        self.mount_button.setIconSize(QtCore.QSize(50, 50))
        self.mount_button.setCheckable(True)
        self.mount_button.setChecked(False)
        self.mount_button.setFlat(True)
        self.mount_button.setObjectName("mount_button")
        self.horizontalLayout_8.addWidget(self.mount_button, 0, QtCore.Qt.AlignBottom)
        self.navi_button = QtWidgets.QPushButton(self.layoutWidget)
        self.navi_button.setEnabled(True)
        self.navi_button.setAutoFillBackground(False)
        self.navi_button.setStyleSheet("background-color: transparent;")
        self.navi_button.setText("")
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap(":/kirins/icon/system/navigation.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon6.addPixmap(QtGui.QPixmap(":/kirins/icon/system/navigation_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.navi_button.setIcon(icon6)
        self.navi_button.setIconSize(QtCore.QSize(50, 50))
        self.navi_button.setCheckable(True)
        self.navi_button.setChecked(False)
        self.navi_button.setFlat(True)
        self.navi_button.setObjectName("navi_button")
        self.horizontalLayout_8.addWidget(self.navi_button, 0, QtCore.Qt.AlignBottom)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem2)
        self.gridLayout_5 = QtWidgets.QGridLayout()
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.down_button = QtWidgets.QPushButton(self.layoutWidget)
        self.down_button.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.down_button.setMouseTracking(True)
        self.down_button.setStyleSheet("background-color: transparent;")
        self.down_button.setText("")
        icon7 = QtGui.QIcon()
        icon7.addPixmap(QtGui.QPixmap(":/kirins/icon/system/down.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.down_button.setIcon(icon7)
        self.down_button.setIconSize(QtCore.QSize(30, 30))
        self.down_button.setFlat(True)
        self.down_button.setObjectName("down_button")
        self.gridLayout_5.addWidget(self.down_button, 2, 1, 1, 1)
        self.left_button = QtWidgets.QPushButton(self.layoutWidget)
        self.left_button.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.left_button.setMouseTracking(True)
        self.left_button.setStyleSheet("background-color: transparent;")
        self.left_button.setText("")
        icon8 = QtGui.QIcon()
        icon8.addPixmap(QtGui.QPixmap(":/kirins/icon/system/left.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.left_button.setIcon(icon8)
        self.left_button.setIconSize(QtCore.QSize(30, 30))
        self.left_button.setFlat(True)
        self.left_button.setObjectName("left_button")
        self.gridLayout_5.addWidget(self.left_button, 1, 0, 1, 1)
        self.up_button = QtWidgets.QPushButton(self.layoutWidget)
        self.up_button.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.up_button.setMouseTracking(True)
        self.up_button.setStyleSheet("background-color: transparent;")
        self.up_button.setText("")
        icon9 = QtGui.QIcon()
        icon9.addPixmap(QtGui.QPixmap(":/kirins/icon/system/up.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.up_button.setIcon(icon9)
        self.up_button.setIconSize(QtCore.QSize(30, 30))
        self.up_button.setFlat(True)
        self.up_button.setObjectName("up_button")
        self.gridLayout_5.addWidget(self.up_button, 0, 1, 1, 1)
        self.home_button = QtWidgets.QPushButton(self.layoutWidget)
        self.home_button.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.home_button.setMouseTracking(True)
        self.home_button.setStyleSheet("background-color: transparent;")
        self.home_button.setText("")
        icon10 = QtGui.QIcon()
        icon10.addPixmap(QtGui.QPixmap(":/kirins/icon/system/home.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.home_button.setIcon(icon10)
        self.home_button.setIconSize(QtCore.QSize(30, 30))
        self.home_button.setFlat(True)
        self.home_button.setObjectName("home_button")
        self.gridLayout_5.addWidget(self.home_button, 1, 1, 1, 1)
        self.right_button = QtWidgets.QPushButton(self.layoutWidget)
        self.right_button.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.right_button.setMouseTracking(True)
        self.right_button.setStyleSheet("background-color: transparent;")
        self.right_button.setText("")
        self.icon11 = QtGui.QIcon()
        self.icon11.addPixmap(QtGui.QPixmap(":/kirins/icon/system/right.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.right_button.setIcon(self.icon11)
        self.right_button.setIconSize(QtCore.QSize(30, 30))
        self.right_button.setFlat(True)
        self.right_button.setObjectName("right_button")
        self.gridLayout_5.addWidget(self.right_button, 1, 2, 1, 1)
        self.horizontalLayout_8.addLayout(self.gridLayout_5)

        ## self.layoutWidget_4 includes infer, MOT, SOT, syncbox, inpsection, laser button on top left of window
        self.layoutWidget_4 = QtWidgets.QWidget(self)
        self.layoutWidget_4.setGeometry(QtCore.QRect(10, 10, 399, 58))
        self.layoutWidget_4.setObjectName("layoutWidget_4")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout(self.layoutWidget_4)
        self.horizontalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_9.setSpacing(5)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.infer_button = QtWidgets.QPushButton(self.layoutWidget_4)
        self.infer_button.setEnabled(True)
        self.infer_button.setAutoFillBackground(False)
        self.infer_button.setText("")
        icon12 = QtGui.QIcon()
        icon12.addPixmap(QtGui.QPixmap(":/kirins/icon/system/infer.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon12.addPixmap(QtGui.QPixmap(":/kirins/icon/system/infer_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.infer_button.setIcon(icon12)
        self.infer_button.setIconSize(QtCore.QSize(50, 50))
        self.infer_button.setCheckable(True)
        self.infer_button.setChecked(False)
        self.infer_button.setFlat(True)
        self.infer_button.setStyleSheet("background-color: transparent;")
        self.infer_button.setObjectName("infer_button")
        self.horizontalLayout_9.addWidget(self.infer_button)
        self.MOT_button = QtWidgets.QPushButton(self.layoutWidget_4)
        self.MOT_button.setEnabled(False)
        self.MOT_button.setAutoFillBackground(False)
        self.MOT_button.setText("")
        icon13 = QtGui.QIcon()
        icon13.addPixmap(QtGui.QPixmap(":/kirins/icon/system/MOT.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon13.addPixmap(QtGui.QPixmap(":/kirins/icon/system/MOT_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.MOT_button.setIcon(icon13)
        self.MOT_button.setIconSize(QtCore.QSize(50, 50))
        self.MOT_button.setCheckable(True)
        self.MOT_button.setChecked(False)
        self.MOT_button.setStyleSheet("background-color: transparent;")
        self.MOT_button.setFlat(True)
        self.MOT_button.setObjectName("MOT_button")
        self.horizontalLayout_9.addWidget(self.MOT_button)
        self.SOT_button = QtWidgets.QPushButton(self.layoutWidget_4)
        self.SOT_button.setEnabled(True)
        self.SOT_button.setAutoFillBackground(False)
        self.SOT_button.setText("")
        icon14 = QtGui.QIcon()
        icon14.addPixmap(QtGui.QPixmap(":/kirins/icon/system/SOT.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon14.addPixmap(QtGui.QPixmap(":/kirins/icon/system/SOT_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.SOT_button.setIcon(icon14)
        self.SOT_button.setIconSize(QtCore.QSize(50, 50))
        self.SOT_button.setCheckable(True)
        self.SOT_button.setChecked(False)
        self.SOT_button.setFlat(True)
        self.SOT_button.setStyleSheet("background-color: transparent;")
        self.SOT_button.setObjectName("SOT_button")
        self.horizontalLayout_9.addWidget(self.SOT_button)
        self.syncbbox_button = QtWidgets.QPushButton(self.layoutWidget_4)
        self.syncbbox_button.setEnabled(False)
        self.syncbbox_button.setAutoFillBackground(False)
        self.syncbbox_button.setText("")
        icon15 = QtGui.QIcon()
        icon15.addPixmap(QtGui.QPixmap(":/kirins/icon/system/sync_bbox.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon15.addPixmap(QtGui.QPixmap(":/kirins/icon/system/sync_bbox_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.syncbbox_button.setIcon(icon15)
        self.syncbbox_button.setIconSize(QtCore.QSize(50, 50))
        self.syncbbox_button.setCheckable(True)
        self.syncbbox_button.setChecked(False)
        self.syncbbox_button.setFlat(True)
        self.syncbbox_button.setStyleSheet("background-color: transparent;")
        self.syncbbox_button.setObjectName("syncbbox_button")
        self.horizontalLayout_9.addWidget(self.syncbbox_button)
        self.inspection_button = QtWidgets.QPushButton(self.layoutWidget_4)
        self.inspection_button.setEnabled(True)
        self.inspection_button.setAutoFillBackground(False)
        self.inspection_button.setStyleSheet("")
        self.inspection_button.setText("")
        icon16 = QtGui.QIcon()
        icon16.addPixmap(QtGui.QPixmap(":/kirins/icon/system/inspection.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon16.addPixmap(QtGui.QPixmap(":/kirins/icon/system/inspection_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.inspection_button.setIcon(icon16)
        self.inspection_button.setIconSize(QtCore.QSize(50, 50))
        self.inspection_button.setCheckable(True)
        self.inspection_button.setChecked(False)
        self.inspection_button.setFlat(True)
        self.inspection_button.setStyleSheet("background-color: transparent;")
        self.inspection_button.setObjectName("inspection_button")
        self.horizontalLayout_9.addWidget(self.inspection_button)
        self.laser_button = QtWidgets.QPushButton(self.layoutWidget_4)
        self.laser_button.setEnabled(True)
        self.laser_button.setAutoFillBackground(False)
        self.laser_button.setStyleSheet("")
        self.laser_button.setText("")
        icon17 = QtGui.QIcon()
        icon17.addPixmap(QtGui.QPixmap(":/kirins/icon/system/laser.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon17.addPixmap(QtGui.QPixmap(":/kirins/icon/system/laser_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.laser_button.setIcon(icon17)
        self.laser_button.setIconSize(QtCore.QSize(50, 50))
        self.laser_button.setCheckable(True)
        self.laser_button.setChecked(False)
        self.laser_button.setFlat(True)
        self.laser_button.setStyleSheet("background-color: transparent;")
        self.laser_button.setObjectName("laser_button")
        self.horizontalLayout_9.addWidget(self.laser_button)

        ## Camera button switch mode between RGB and Flir camera
        self.verticalLayoutWidget = QtWidgets.QWidget(self)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 70, 101, 491))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        spacerItem3 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_5.addItem(spacerItem3)
        self.camera_button = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.camera_button.setEnabled(True)
        self.camera_button.setAutoFillBackground(False)
        self.camera_button.setText("")
        icon18 = QtGui.QIcon()
        icon18.addPixmap(QtGui.QPixmap(":/kirins/icon/system/RGB_.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon18.addPixmap(QtGui.QPixmap(":/kirins/icon/system/thermal_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.camera_button.setIcon(icon18)
        self.camera_button.setIconSize(QtCore.QSize(50, 50))
        self.camera_button.setCheckable(True)
        self.camera_button.setChecked(False)
        self.camera_button.setFlat(True)
        self.camera_button.setStyleSheet("background-color: transparent;")
        self.camera_button.setObjectName("camera_button")
        self.verticalLayout_5.addWidget(self.camera_button, 0, QtCore.Qt.AlignLeft)
        self.dual_button = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.dual_button.setEnabled(True)
        self.dual_button.setAutoFillBackground(False)
        self.dual_button.setText("")
        icon19 = QtGui.QIcon()
        icon19.addPixmap(QtGui.QPixmap(":/kirins/icon/system/dual.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon19.addPixmap(QtGui.QPixmap(":/kirins/icon/system/dual_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.dual_button.setIcon(icon19)
        self.dual_button.setIconSize(QtCore.QSize(50, 50))
        self.dual_button.setCheckable(True)
        self.dual_button.setChecked(False)
        self.dual_button.setFlat(True)
        self.dual_button.setObjectName("dual_button")
        self.dual_button.setStyleSheet("background-color: transparent;")
        self.verticalLayout_5.addWidget(self.dual_button, 0, QtCore.Qt.AlignLeft)
        spacerItem4 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_5.addItem(spacerItem4)

        ## self.verticalLayoutWidget_2 includes lock, syncgimbal, angle and touch button on center right of window
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(1160, 70, 111, 501))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        spacerItem5 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_6.addItem(spacerItem5)
        self.lock_button = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.lock_button.setEnabled(True)
        self.lock_button.setAutoFillBackground(False)
        self.lock_button.setText("")
        icon19 = QtGui.QIcon()
        icon19.addPixmap(QtGui.QPixmap(":/kirins/icon/system/lock.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon19.addPixmap(QtGui.QPixmap(":/kirins/icon/system/lock_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.lock_button.setIcon(icon19)
        self.lock_button.setIconSize(QtCore.QSize(50, 50))
        self.lock_button.setCheckable(True)
        self.lock_button.setChecked(False)
        self.lock_button.setFlat(True)
        self.lock_button.setStyleSheet("background-color: transparent;")
        self.lock_button.setObjectName("lock_button")
        self.verticalLayout_6.addWidget(self.lock_button, 0, QtCore.Qt.AlignRight)
        self.syncgimbal_button = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.syncgimbal_button.setEnabled(True)
        self.syncgimbal_button.setAutoFillBackground(False)
        self.syncgimbal_button.setText("")
        icon20 = QtGui.QIcon()
        icon20.addPixmap(QtGui.QPixmap(":/kirins/icon/system/sync_gimbal.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon20.addPixmap(QtGui.QPixmap(":/kirins/icon/system/syncgimbal_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.syncgimbal_button.setIcon(icon20)
        self.syncgimbal_button.setIconSize(QtCore.QSize(50, 50))
        self.syncgimbal_button.setCheckable(True)
        self.syncgimbal_button.setChecked(False)
        self.syncgimbal_button.setFlat(True)
        self.syncgimbal_button.setStyleSheet("background-color: transparent;")
        self.syncgimbal_button.setObjectName("syncgimbal_button")
        self.verticalLayout_6.addWidget(self.syncgimbal_button, 0, QtCore.Qt.AlignRight)
        self.angle_button = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.angle_button.setEnabled(True)
        self.angle_button.setAutoFillBackground(False)
        self.angle_button.setText("")
        icon21 = QtGui.QIcon()
        icon21.addPixmap(QtGui.QPixmap(":/kirins/icon/system/angle.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon21.addPixmap(QtGui.QPixmap(":/kirins/icon/system/angle_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.angle_button.setIcon(icon21)
        self.angle_button.setIconSize(QtCore.QSize(50, 50))
        self.angle_button.setCheckable(True)
        self.angle_button.setChecked(False)
        self.angle_button.setFlat(True)
        self.angle_button.setObjectName("angle_button")
        self.verticalLayout_6.addWidget(self.angle_button, 0, QtCore.Qt.AlignRight)
        self.touch_button = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.touch_button.setEnabled(True)
        self.touch_button.setAutoFillBackground(False)
        self.touch_button.setText("")
        icon22 = QtGui.QIcon()
        icon22.addPixmap(QtGui.QPixmap(":/kirins/icon/system/touch.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon22.addPixmap(QtGui.QPixmap(":/kirins/icon/system/touch_.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.touch_button.setIcon(icon22)
        self.touch_button.setIconSize(QtCore.QSize(50, 50))
        self.touch_button.setCheckable(True)
        self.touch_button.setChecked(False)
        self.touch_button.setFlat(True)
        self.touch_button.setStyleSheet("background-color: transparent;")
        self.touch_button.setObjectName("touch_button")
        self.verticalLayout_6.addWidget(self.touch_button, 0, QtCore.Qt.AlignRight)
        spacerItem6 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_6.addItem(spacerItem6)

        ## Create zoom bar 
        self.zoom_widget = QtWidgets.QWidget(self)
        self.zoom_widget.setGeometry(QtCore.QRect(1060, 110, 91, 411))
        self.zoom_widget.setObjectName("zoom_widget")
        self.zoom_out = QtWidgets.QLabel(self.zoom_widget)
        self.zoom_out.setGeometry(QtCore.QRect(37, 38, 15, 15))
        self.zoom_out.setText("")
        self.zoom_out.setPixmap(QtGui.QPixmap(":/kirins/icon/system/plus.png"))
        self.zoom_out.setScaledContents(True)
        self.zoom_out.setObjectName("zoom_out")
        self.zoom_bar = QtWidgets.QSlider(self.zoom_widget)
        self.zoom_bar.setGeometry(QtCore.QRect(40, 60, 20, 291))
        self.zoom_bar.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.zoom_bar.setStyleSheet("")
        self.zoom_bar.setMinimum(1)
        self.zoom_bar.setMaximum(31)
        self.zoom_bar.setProperty("value", 1)
        self.zoom_bar.setOrientation(QtCore.Qt.Vertical)
        self.zoom_bar.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.zoom_bar.setTickInterval(1)
        self.zoom_bar.setObjectName("zoom_bar")
        self.zoom_bar.setTracking(False)
        self.zoom_in = QtWidgets.QLabel(self.zoom_widget)
        self.zoom_in.setGeometry(QtCore.QRect(38, 354, 15, 15))
        self.zoom_in.setText("")
        self.zoom_in.setPixmap(QtGui.QPixmap(":/kirins/icon/system/minus.png"))
        self.zoom_in.setScaledContents(True)
        self.zoom_in.setObjectName("zoom_in")

        self.autoZoom = QtWidgets.QCheckBox(self.zoom_widget)
        self.autoZoom.setGeometry(QtCore.QRect(20, 370, 61, 25))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.autoZoom.setFont(font)
        self.autoZoom.setObjectName("checkBox")
        self.autoZoom.setText("Auto")
        self.autoZoom.setEnabled(False)

        self.label_searching = QtWidgets.QLabel(self)
        self.label_searching.setGeometry(QtCore.QRect(600, 20, 111, 21))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_searching.setFont(font)
        self.label_searching.setStyleSheet("color: rgb(204, 0, 0);\n")
        self.label_searching.setAlignment(QtCore.Qt.AlignCenter)
        self.label_searching.setObjectName("label_2")

        ## Link button to function 
        self.infer_button.clicked.connect(self.infer_pushed)
        self.MOT_button.clicked.connect(self.MOT_pushed)
        self.SOT_button.clicked.connect(self.SOT_pushed)
        self.syncbbox_button.clicked.connect(self.syncbbox_pushed)
        self.inspection_button.clicked.connect(self.inspection_pushed)

        self.dual_button.clicked.connect(self.dual_pushed)
        self.camera_button.clicked.connect(self.camera_pushed)
        self.lock_button.clicked.connect(self.lock_pushed)
        self.syncgimbal_button.clicked.connect(self.syncgimbal_pushed)   
        self.angle_button.clicked.connect(self.gimbal_angle_pushed)
        self.touch_button.clicked.connect(self.touch_pushed)
        self.home_button.clicked.connect(self.home_pushed)
        self.up_button.clicked.connect(self.up_pushed)
        self.down_button.clicked.connect(self.down_pushed)
        self.left_button.clicked.connect(self.left_pushed)
        self.right_button.clicked.connect(self.right_pushed)

        self.setting_button.clicked.connect(self.camera_setting_pushed)
        self.EPS_button.clicked.connect(self.exposure_setting_pushed)
        self.focus_button.clicked.connect(self.focus_setting_pushed)
        self.reset_button.clicked.connect(self.reset_pushed)

        self.audio_button.clicked.connect(self.audio_setting_pushed)
        self.navi_button.clicked.connect(self.navi_setting_pushed)

        ## Link slider to function
        self.zoom_bar.valueChanged[int].connect(self.zoom_valchange)
        self.autoZoom.stateChanged.connect(self.autozoom_pushed)

        #####------------------------------------------------------#####
        #####                      DRAWING BBOX                    #####
        #####------------------------------------------------------#####

        ## Create pixmap obtain frames of stream 
        self.pix = QPixmap(self.rect().size())
        self.pix.fill(Qt.white)

        ## Create start point and end point to draw bounding box
        self.begin, self.destination = QPoint(), QPoint()
        self.x1, self.y1, self.w, self.h = -1,-1,-1,-1

        ## For right mouse
        self.start_right_mouse, self.end_right_mouse = QPoint(), QPoint()
        self.right_mouse_move = False

        #####------------------------------------------------------#####
        #####                      PLAY STREAM                     #####
        #####------------------------------------------------------#####
        self.thread = {}

        # Create the video capture thread
        self.thread[0] = RGB_Stream()
        # Connect its signal to the update_image slot
        self.thread[0].changePixmap.connect(self.update_rgb_frame)
        # Start the thread
        self.thread[0].start()

        # Create the video capture thread
        self.thread[1] = Thermal_Stream()
        # Connect its signal to the update_image slot
        self.thread[1].changePixmap.connect(self.update_thermal_frame)
        # Start the thread
        self.thread[1].start()

    def closeEvent(self, event):
        self.thread[0].stop()
        self.thread[1].stop()
        event.accept()

    #####------------------------------------------------------#####
    #####                    BUTTON FUNCTION                   #####
    #####------------------------------------------------------#####
    def exposure_setting_pushed(self):
        if self.EPS_button.isChecked():
            self.exposure_panel = QtWidgets.QFrame(self)
            self.exposure_panel.setGeometry(QtCore.QRect(340, 120, 571, 381))
            self.exposure_panel.setFrameShape(QtWidgets.QFrame.StyledPanel)
            self.exposure_panel.setFrameShadow(QtWidgets.QFrame.Raised)
            self.exposure_panel.setObjectName("exposure_panel")
            self.exposure_panel.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.eps_spot_button = QtWidgets.QPushButton(self.exposure_panel)
            self.eps_spot_button.setGeometry(QtCore.QRect(210, 140, 150, 50))
            self.eps_spot_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.eps_spot_button.setObjectName("eps_spot_button")
            self.eps_spot_button.setText("Spot")
            self.bright_button = QtWidgets.QPushButton(self.exposure_panel)
            self.bright_button.setGeometry(QtCore.QRect(390, 70, 150, 50))
            self.bright_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.bright_button.setObjectName("bright_button")
            self.bright_button.setText("Bright")
            self.iris_button = QtWidgets.QPushButton(self.exposure_panel)
            self.iris_button.setGeometry(QtCore.QRect(30, 70, 150, 50))
            self.iris_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.iris_button.setText("IRIS Priority")
            self.iris_button.setObjectName("iris_button")
            self.eps_fullAuto_button = QtWidgets.QPushButton(self.exposure_panel)
            self.eps_fullAuto_button.setGeometry(QtCore.QRect(30, 10, 150, 50))
            self.eps_fullAuto_button.setLayoutDirection(QtCore.Qt.RightToLeft)
            self.eps_fullAuto_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.eps_fullAuto_button.setText("Full Auto")
            self.eps_fullAuto_button.setObjectName("eps_fullAuto_button")
            self.eps_manual_button = QtWidgets.QPushButton(self.exposure_panel)
            self.eps_manual_button.setGeometry(QtCore.QRect(210, 10, 150, 50))
            self.eps_manual_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.eps_manual_button.setText("Manual")
            self.eps_manual_button.setObjectName("eps_manual_button")
            
            self.shutter_button = QtWidgets.QPushButton(self.exposure_panel)
            self.shutter_button.setGeometry(QtCore.QRect(390, 10, 150, 50))
            self.shutter_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.shutter_button.setText("Shutter Priority")
            self.shutter_button.setObjectName("shutter_button")
            self.gain_button = QtWidgets.QPushButton(self.exposure_panel)
            self.gain_button.setGeometry(QtCore.QRect(210, 70, 150, 50))
            self.gain_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.gain_button.setText("Gain Priority")
            self.gain_button.setObjectName("gain_button")
            self.gain_bar = QtWidgets.QSlider(self.exposure_panel)
            self.gain_bar.setEnabled(False)
            self.gain_bar.setGeometry(QtCore.QRect(30, 210, 501, 21))
            self.gain_bar.setMinimum(1)
            self.gain_bar.setMaximum(17)
            self.gain_bar.setValue(self.gain_val)
            self.gain_bar.setOrientation(QtCore.Qt.Horizontal)
            self.gain_bar.setTickPosition(QtWidgets.QSlider.TicksBelow)
            self.gain_bar.setObjectName("gain_bar")
            self.gain_bar.setStyleSheet("background-color: transparent;")
            self.gain_bar.setTracking(False)
            self.shutter_bar = QtWidgets.QSlider(self.exposure_panel)
            self.shutter_bar.setEnabled(False)
            self.shutter_bar.setGeometry(QtCore.QRect(30, 260, 501, 21))
            self.shutter_bar.setMinimum(6)
            self.shutter_bar.setMaximum(33)
            self.shutter_bar.setValue(self.shutter_val)
            self.shutter_bar.setOrientation(QtCore.Qt.Horizontal)
            self.shutter_bar.setTickPosition(QtWidgets.QSlider.TicksBelow)
            self.shutter_bar.setObjectName("shutter_bar")
            self.shutter_bar.setStyleSheet("background-color: transparent;")
            self.shutter_bar.setTracking(False)
            self.iris_bar = QtWidgets.QSlider(self.exposure_panel)
            self.iris_bar.setEnabled(False)
            self.iris_bar.setGeometry(QtCore.QRect(30, 310, 501, 21))
            self.iris_bar.setMaximum(25)
            self.iris_bar.setValue(self.iris_val)
            self.iris_bar.setOrientation(QtCore.Qt.Horizontal)
            self.iris_bar.setTickPosition(QtWidgets.QSlider.TicksBelow)
            self.iris_bar.setObjectName("iris_bar")
            self.iris_bar.setStyleSheet("background-color: transparent;")
            self.iris_bar.setTracking(False)
            self.label_2 = QtWidgets.QLabel(self.exposure_panel)
            self.label_2.setGeometry(QtCore.QRect(10, 350, 51, 19))
            self.label_2.setObjectName("label_2")
            self.label_2.setText("Mode:")
            self.label_2.setStyleSheet("background-color: transparent;")
            self.exposure_mode = QtWidgets.QLabel(self.exposure_panel)
            self.exposure_mode.setGeometry(QtCore.QRect(60, 350, 141, 19))
            self.exposure_mode.setObjectName("exposure_mode")
            self.exposure_mode.setText(self.curr_eps_mode)
            self.exposure_mode.setStyleSheet("background-color: transparent;")
            self.label_4 = QtWidgets.QLabel(self.exposure_panel)
            self.label_4.setEnabled(True)
            self.label_4.setGeometry(QtCore.QRect(30, 190, 72, 19))
            self.label_4.setObjectName("label_4")
            self.label_4.setText("Gain")
            self.label_4.setStyleSheet("background-color: transparent;")
            self.label_5 = QtWidgets.QLabel(self.exposure_panel)
            self.label_5.setGeometry(QtCore.QRect(30, 240, 111, 19))
            self.label_5.setObjectName("label_5")
            self.label_5.setText("Shutter Speed")
            self.label_5.setStyleSheet("background-color: transparent;")
            self.label_6 = QtWidgets.QLabel(self.exposure_panel)
            self.label_6.setGeometry(QtCore.QRect(30, 290, 72, 19))
            self.label_6.setObjectName("label_6")
            self.label_6.setText("IRIS")
            self.label_6.setStyleSheet("background-color: transparent;")

            ## Check bar status
            if self.curr_eps_mode == 'Manual' or self.curr_eps_mode == 'Bright':
                self.gain_bar.setEnabled(True)
                self.iris_bar.setEnabled(True)
                self.shutter_bar.setEnabled(True)

            ## Link mode function
            self.eps_fullAuto_button.clicked.connect(self.eps_fullAuto_pushed)
            self.eps_manual_button.clicked.connect(self.eps_manual_pushed)
            self.shutter_button.clicked.connect(self.shutter_pushed)
            self.iris_button.clicked.connect(self.iris_pushed)
            self.gain_button.clicked.connect(self.gain_pushed)
            self.bright_button.clicked.connect(self.bright_pushed)
            self.eps_spot_button.clicked.connect(self.eps_spot_pushed)

            ## Link bar callback funtion
            self.gain_bar.valueChanged[int].connect(self.gain_valchange)
            self.iris_bar.valueChanged[int].connect(self.iris_valchange)
            self.shutter_bar.valueChanged[int].connect(self.shutter_valchange)

            ## Show exposure panel on window
            self.exposure_panel.show()
        else:
            ## Close exposure panel on window
            self.exposure_panel.deleteLater()

    def camera_setting_pushed(self):       
        if self.setting_button.isChecked():
            self.camera_setting_panel = QtWidgets.QFrame(self)
            self.camera_setting_panel.setGeometry(QtCore.QRect(370, 160, 571, 381))
            self.camera_setting_panel.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.camera_setting_panel.setFrameShape(QtWidgets.QFrame.StyledPanel)
            self.camera_setting_panel.setFrameShadow(QtWidgets.QFrame.Raised)
            self.camera_setting_panel.setObjectName("camera_setting_panel")
            self.camera_setting_menu = QtWidgets.QTabWidget(self.camera_setting_panel)
            self.camera_setting_menu.setGeometry(QtCore.QRect(0, 0, 571, 381))
            self.camera_setting_menu.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.camera_setting_menu.setTabPosition(QtWidgets.QTabWidget.North)
            self.camera_setting_menu.setTabShape(QtWidgets.QTabWidget.Rounded)
            self.camera_setting_menu.setObjectName("camera_setting_menu")
            self.common_menu = QtWidgets.QWidget()
            self.common_menu.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.common_menu.setObjectName("common_menu")
            self.stabilize_button = QtWidgets.QPushButton(self.common_menu)
            self.stabilize_button.setGeometry(QtCore.QRect(220, 90, 150, 50))
            self.stabilize_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.stabilize_button.setText("Stabilize")
            self.stabilize_button.setCheckable(True)
            self.stabilize_button.setChecked(self.curr_stabilize)
            self.stabilize_button.setObjectName("stabilize_button")
            self.defog_button = QtWidgets.QPushButton(self.common_menu)
            self.defog_button.setGeometry(QtCore.QRect(400, 90, 150, 50))
            self.defog_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.defog_button.setText("Defog")
            self.defog_button.setCheckable(True)
            self.defog_button.setChecked(self.curr_defog)
            self.defog_button.setObjectName("defog_button")
            self.capture_button = QtWidgets.QPushButton(self.common_menu)
            self.capture_button.setGeometry(QtCore.QRect(40, 150, 150, 50))
            self.capture_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.capture_button.setText("Capture")
            self.capture_button.setCheckable(True)
            self.capture_button.setObjectName("capture_button")
            self.record_button = QtWidgets.QPushButton(self.common_menu)
            self.record_button.setGeometry(QtCore.QRect(220, 150, 150, 50))
            self.record_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.record_button.setText("Record")
            self.record_button.setCheckable(True)
            self.record_button.setChecked(self.curr_record)
            self.record_button.setObjectName("record_button")
            self.sensitivity_button = QtWidgets.QPushButton(self.common_menu)
            self.sensitivity_button.setGeometry(QtCore.QRect(40, 90, 150, 50))
            self.sensitivity_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.sensitivity_button.setCheckable(True)
            self.sensitivity_button.setChecked(self.curr_sensitivity)
            self.sensitivity_button.setObjectName("sensitivity_button")
            self.backlight_button = QtWidgets.QPushButton(self.common_menu)
            self.backlight_button.setGeometry(QtCore.QRect(400, 150, 150, 50))
            self.backlight_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.backlight_button.setCheckable(True)
            self.backlight_button.setChecked(self.curr_backlight)
            self.backlight_button.setObjectName("backlight_button")
            self.camera_setting_menu.addTab(self.common_menu, "")
            self.white_balance_menu = QtWidgets.QWidget()
            self.white_balance_menu.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.white_balance_menu.setObjectName("white_balance_menu")
            self.indoor_button = QtWidgets.QPushButton(self.white_balance_menu)
            self.indoor_button.setGeometry(QtCore.QRect(390, 20, 150, 50))
            self.indoor_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.indoor_button.setObjectName("indoor_button")
            self.wb_auto_button = QtWidgets.QPushButton(self.white_balance_menu)
            self.wb_auto_button.setGeometry(QtCore.QRect(30, 20, 150, 50))
            self.wb_auto_button.setLayoutDirection(QtCore.Qt.RightToLeft)
            self.wb_auto_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.wb_auto_button.setObjectName("wb_auto_button")
            self.onePush_button = QtWidgets.QPushButton(self.white_balance_menu)
            self.onePush_button.setGeometry(QtCore.QRect(390, 80, 150, 50))
            self.onePush_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.onePush_button.setObjectName("onePush_button")
            self.outdoorAuto_button = QtWidgets.QPushButton(self.white_balance_menu)
            self.outdoorAuto_button.setGeometry(QtCore.QRect(210, 80, 150, 50))
            self.outdoorAuto_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.outdoorAuto_button.setObjectName("outdoorAuto_button")
            self.wb_manual_button = QtWidgets.QPushButton(self.white_balance_menu)
            self.wb_manual_button.setGeometry(QtCore.QRect(210, 20, 150, 50))
            self.wb_manual_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.wb_manual_button.setObjectName("wb_manual_button")
            self.sodiumLamp_button = QtWidgets.QPushButton(self.white_balance_menu)
            self.sodiumLamp_button.setGeometry(QtCore.QRect(390, 150, 150, 50))
            self.sodiumLamp_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.sodiumLamp_button.setObjectName("sodiumLamp_button")
            self.outdoor_button = QtWidgets.QPushButton(self.white_balance_menu)
            self.outdoor_button.setGeometry(QtCore.QRect(30, 80, 150, 50))
            self.outdoor_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.outdoor_button.setObjectName("outdoor_button")
            self.autoTracing_button = QtWidgets.QPushButton(self.white_balance_menu)
            self.autoTracing_button.setGeometry(QtCore.QRect(30, 150, 150, 50))
            self.autoTracing_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.autoTracing_button.setObjectName("autoTracing_button")
            self.label = QtWidgets.QLabel(self.white_balance_menu)
            self.label.setGeometry(QtCore.QRect(10, 320, 51, 19))
            self.label.setObjectName("label")
            self.wb_mode = QtWidgets.QLabel(self.white_balance_menu)
            self.wb_mode.setGeometry(QtCore.QRect(60, 320, 141, 19))
            self.wb_mode.setObjectName("wb_mode")
            self.red_gain_bar = QtWidgets.QSlider(self.white_balance_menu)
            self.red_gain_bar.setGeometry(QtCore.QRect(30, 240, 501, 21))
            self.red_gain_bar.setOrientation(QtCore.Qt.Horizontal)
            self.red_gain_bar.setObjectName("red_gain_bar")
            self.red_gain_bar.setValue(self.red_gain_val)
            self.red_gain_bar.setMaximum(15)
            self.red_gain_bar.setMinimum(0)
            self.red_gain_bar.setEnabled(False)
            self.red_gain_bar.setStyleSheet("background-color: transparent;")
            self.red_gain_bar.setTracking(False)
            self.blue_gain_bar = QtWidgets.QSlider(self.white_balance_menu)
            self.blue_gain_bar.setGeometry(QtCore.QRect(30, 290, 501, 21))
            self.blue_gain_bar.setOrientation(QtCore.Qt.Horizontal)
            self.blue_gain_bar.setObjectName("blue_gain_bar")
            self.blue_gain_bar.setValue(self.blue_gain_val)
            self.blue_gain_bar.setMaximum(15)
            self.blue_gain_bar.setMinimum(0)
            self.blue_gain_bar.setEnabled(False)
            self.blue_gain_bar.setStyleSheet("background-color: transparent;")
            self.blue_gain_bar.setTracking(False)
            if self.curr_wb_mode == 'Manual':
                self.red_gain_bar.setEnabled(True)
                self.blue_gain_bar.setEnabled(True)
            self.label_2 = QtWidgets.QLabel(self.white_balance_menu)
            self.label_2.setGeometry(QtCore.QRect(30, 220, 51, 19))
            self.label_2.setObjectName("label_2")
            self.label_4 = QtWidgets.QLabel(self.white_balance_menu)
            self.label_4.setGeometry(QtCore.QRect(30, 270, 51, 19))
            self.label_4.setObjectName("blue_gain_bar")
            self.label_2.setText("R Gain")
            self.label_4.setText("B Gain")
            self.camera_setting_menu.addTab(self.white_balance_menu, "")
            self.monitor_menu = QtWidgets.QWidget()
            self.monitor_menu.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.monitor_menu.setObjectName("monitor_menu")
            self.stackedWidget = QtWidgets.QStackedWidget(self.monitor_menu)
            self.stackedWidget.setGeometry(QtCore.QRect(10, 0, 551, 281))
            self.stackedWidget.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.stackedWidget.setObjectName("stackedWidget")
            self.page = QtWidgets.QWidget()
            self.page.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.page.setObjectName("page")
            self.monitor_2160p2997_button = QtWidgets.QPushButton(self.page)
            self.monitor_2160p2997_button.setGeometry(QtCore.QRect(20, 20, 150, 50))
            self.monitor_2160p2997_button.setLayoutDirection(QtCore.Qt.RightToLeft)
            self.monitor_2160p2997_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_2160p2997_button.setCheckable(False)
            self.monitor_2160p2997_button.setObjectName("monitor_2160p2997")
            self.monitor_1080p5994_button = QtWidgets.QPushButton(self.page)
            self.monitor_1080p5994_button.setGeometry(QtCore.QRect(20, 80, 150, 50))
            self.monitor_1080p5994_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_1080p5994_button.setObjectName("monitor_1080p5994")
            self.monitor_1080p2997_button = QtWidgets.QPushButton(self.page)
            self.monitor_1080p2997_button.setGeometry(QtCore.QRect(380, 80, 150, 50))
            self.monitor_1080p2997_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_1080p2997_button.setObjectName("monitor_1080p2997")
            self.monitor_2160p2398_button = QtWidgets.QPushButton(self.page)
            self.monitor_2160p2398_button.setGeometry(QtCore.QRect(380, 20, 150, 50))
            self.monitor_2160p2398_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_2160p2398_button.setObjectName("monitor_2160p2398")
            self.monitor_2160p25_button = QtWidgets.QPushButton(self.page)
            self.monitor_2160p25_button.setGeometry(QtCore.QRect(200, 20, 150, 50))
            self.monitor_2160p25_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_2160p25_button.setObjectName("monitor_2160p25")
            self.monitor_1080p50_button = QtWidgets.QPushButton(self.page)
            self.monitor_1080p50_button.setGeometry(QtCore.QRect(200, 80, 150, 50))
            self.monitor_1080p50_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_1080p50_button.setObjectName("monitor_1080p50")
            self.monitor_1080p25_button = QtWidgets.QPushButton(self.page)
            self.monitor_1080p25_button.setGeometry(QtCore.QRect(20, 150, 150, 50))
            self.monitor_1080p25_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_1080p25_button.setObjectName("monitor_1080p25")
            self.monitor_1080p23_button = QtWidgets.QPushButton(self.page)
            self.monitor_1080p23_button.setGeometry(QtCore.QRect(200, 150, 150, 50))
            self.monitor_1080p23_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_1080p23_button.setObjectName("monitor_1080p23")
            self.monitor_1080i5994_button = QtWidgets.QPushButton(self.page)
            self.monitor_1080i5994_button.setGeometry(QtCore.QRect(380, 150, 150, 50))
            self.monitor_1080i5994_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_1080i5994_button.setObjectName("monitor_1080i5994")
            self.monitor_720p5994_button = QtWidgets.QPushButton(self.page)
            self.monitor_720p5994_button.setGeometry(QtCore.QRect(200, 220, 150, 50))
            self.monitor_720p5994_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_720p5994_button.setObjectName("monitor_720p5994")
            self.monitor_1080i50_button = QtWidgets.QPushButton(self.page)
            self.monitor_1080i50_button.setGeometry(QtCore.QRect(20, 220, 150, 50))
            self.monitor_1080i50_button.setLayoutDirection(QtCore.Qt.RightToLeft)
            self.monitor_1080i50_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_1080i50_button.setCheckable(True)
            self.monitor_1080i50_button.setObjectName("monitor_1080i50")
            self.monitor_720p50_button = QtWidgets.QPushButton(self.page)
            self.monitor_720p50_button.setGeometry(QtCore.QRect(380, 220, 150, 50))
            self.monitor_720p50_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_720p50_button.setObjectName("monitor_720p50")
            self.stackedWidget.addWidget(self.page)
            self.page_2 = QtWidgets.QWidget()
            self.page_2.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.page_2.setObjectName("page_2")
            self.monitor_480p5994_button = QtWidgets.QPushButton(self.page_2)
            self.monitor_480p5994_button.setGeometry(QtCore.QRect(120, 130, 150, 50))
            self.monitor_480p5994_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_480p5994_button.setObjectName("monitor_480p5994")
            self.monitor_576p50_button = QtWidgets.QPushButton(self.page_2)
            self.monitor_576p50_button.setGeometry(QtCore.QRect(300, 130, 150, 50))
            self.monitor_576p50_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.monitor_576p50_button.setObjectName("monitor_576p50")
            self.stackedWidget.addWidget(self.page_2)
            
            self.next_button1 = QtWidgets.QPushButton(self.page)
            self.next_button1.setGeometry(QtCore.QRect(540, 5, 12, 12))
            self.next_button1.setStyleSheet("background-color: transparent;")
            self.next_button1.setText("")
            self.next_button1.setIcon(self.icon11)
            self.next_button1.setIconSize(QtCore.QSize(12, 12))
            self.next_button1.setFlat(True)
            self.next_button1.setObjectName("next_button1")

            self.next_button2 = QtWidgets.QPushButton(self.page_2)
            self.next_button2.setGeometry(QtCore.QRect(540, 5, 12, 12))
            self.next_button2.setStyleSheet("background-color: transparent;")
            self.next_button2.setText("")
            self.next_button2.setIcon(self.icon11)
            self.next_button2.setIconSize(QtCore.QSize(12, 12))
            self.next_button2.setFlat(True)
            self.next_button2.setObjectName("next_button2")
            self.camera_setting_menu.addTab(self.monitor_menu, "")

            self.autoZoom.setText("Auto")
            self.stabilize_button.setText("Stabilize")
            self.defog_button.setText("Defog")
            self.capture_button.setText("Capture")
            self.record_button.setText("Record")
            self.sensitivity_button.setText("High Sensitivity")
            self.backlight_button.setText("Backlight")
            self.camera_setting_menu.setTabText(self.camera_setting_menu.indexOf(self.common_menu), "Common")
            self.indoor_button.setText("Indoor")
            self.wb_auto_button.setText("Auto")
            self.onePush_button.setText("One Push")
            self.outdoorAuto_button.setText("Outdoor Auto")
            self.wb_manual_button.setText("Manual")
            self.sodiumLamp_button.setText("Sodium Lamp")
            self.outdoor_button.setText("Outdoor")
            self.autoTracing_button.setText("Auto Tracing")
            self.camera_setting_menu.setTabText(self.camera_setting_menu.indexOf(self.white_balance_menu), "White Balance")
            self.monitor_2160p2997_button.setText("2160p/29.97")
            self.monitor_1080p5994_button.setText("1080p/59.94")
            self.monitor_1080p2997_button.setText("1080p/29.97")
            self.monitor_2160p2398_button.setText("2160p/23.98")
            self.monitor_2160p25_button.setText("2160p/25")
            self.monitor_1080p50_button.setText("1080p/50")
            self.monitor_1080p25_button.setText("1080p/25")
            self.monitor_1080p23_button.setText("1080p/23")
            self.monitor_1080i5994_button.setText("1080i/59.94")
            self.monitor_1080i50_button.setText("1080i/50")
            self.monitor_480p5994_button.setText("480p/59.94")
            self.monitor_720p50_button.setText("720p/50")
            self.monitor_720p5994_button.setText("720p/59.94")
            self.monitor_576p50_button.setText("576p/50")
            self.camera_setting_menu.setTabText(self.camera_setting_menu.indexOf(self.monitor_menu), "Monitor Modes")

            self.label = QtWidgets.QLabel(self.white_balance_menu)
            self.label.setGeometry(QtCore.QRect(10, 320, 51, 19))
            self.label.setObjectName("label")
            self.wb_mode = QtWidgets.QLabel(self.white_balance_menu)
            self.wb_mode.setGeometry(QtCore.QRect(60, 320, 141, 19))
            self.wb_mode.setObjectName("wb_mode")
            self.wb_mode.setText(self.curr_wb_mode)
            self.label.setText("Mode:")

            self.label_3 = QtWidgets.QLabel(self.monitor_menu)
            self.label_3.setGeometry(QtCore.QRect(10, 320, 51, 19))
            self.label_3.setObjectName("label_3")
            self.monitor_mode = QtWidgets.QLabel(self.monitor_menu)
            self.monitor_mode.setGeometry(QtCore.QRect(60, 320, 151, 19))
            self.monitor_mode.setObjectName("monitor_mode")
            self.label_3.setText("Mode:")
            self.label_3.setStyleSheet("background-color: transparent;")
            self.monitor_mode.setText(self.curr_monitor_mode)

            self.stabilize_button.clicked.connect(self.stabilize_pushed)
            self.defog_button.clicked.connect(self.defog_pushed)
            self.backlight_button.clicked.connect(self.backlight_pushed)
            self.sensitivity_button.clicked.connect(self.sensitivity_pushed)

            ## Button link to white balance mode function
            self.wb_auto_button.clicked.connect(self.wb_auto_pushed)
            self.wb_manual_button.clicked.connect(self.wb_manual_pushed)
            self.indoor_button.clicked.connect(self.indoor_pushed)
            self.outdoor_button.clicked.connect(self.outdoor_pushed)
            self.outdoorAuto_button.clicked.connect(self.outdoorAuto_pushed)
            self.onePush_button.clicked.connect(self.onePush_pushed)
            self.autoTracing_button.clicked.connect(self.autoTracing_pushed)
            self.sodiumLamp_button.clicked.connect(self.sodiumLamp_pushed)

            ## Button link to monitor mode function
            self.next_button1.clicked.connect(self.next1_pushed)
            self.next_button2.clicked.connect(self.next2_pushed)
            self.monitor_2160p2997_button.clicked.connect(self.monitor_2160p2997_pushed)
            self.monitor_2160p25_button.clicked.connect(self.monitor_2160p25_pushed)
            self.monitor_2160p2398_button.clicked.connect(self.monitor_2160p2398_pushed)
            self.monitor_1080p5994_button.clicked.connect(self.monitor_1080p5994_pushed)
            self.monitor_1080p50_button.clicked.connect(self.monitor_1080p50_pushed)
            self.monitor_1080p2997_button.clicked.connect(self.monitor_1080p2997_pushed)
            self.monitor_1080p25_button.clicked.connect(self.monitor_1080p25_pushed)
            self.monitor_1080p23_button.clicked.connect(self.monitor_1080p23_pushed)
            self.monitor_1080i5994_button.clicked.connect(self.monitor_1080i5994_pushed)
            self.monitor_1080i50_button.clicked.connect(self.monitor_1080i50_pushed)
            self.monitor_720p5994_button.clicked.connect(self.monitor_720p5994_pushed)
            self.monitor_720p50_button.clicked.connect(self.monitor_720p50_pushed)
            self.monitor_480p5994_button.clicked.connect(self.monitor_480p5994_pushed)
            self.monitor_576p50_button.clicked.connect(self.monitor_576p50_pushed)

            ## Check white balance manual status
            if self.curr_wb_mode == 'Manual':
                self.red_gain_bar.setEnabled(True)
                self.blue_gain_bar.setEnabled(True)

            ## Slider link to call back to change value
            self.red_gain_bar.valueChanged[int].connect(self.red_gain_valchange)
            self.blue_gain_bar.valueChanged[int].connect(self.blue_gain_valchange)

            self.camera_setting_panel.show()
        else:
            self.camera_setting_panel.deleteLater()

    def next1_pushed(self):
        self.stackedWidget.setCurrentIndex(1)

    def next2_pushed(self):
        self.stackedWidget.setCurrentIndex(0)

    def focus_setting_pushed(self):
        if self.focus_button.isChecked():
            self.focus_setting_panel = QtWidgets.QFrame(self)
            self.focus_setting_panel.setGeometry(QtCore.QRect(410, 250, 551, 181))
            self.focus_setting_panel.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.focus_setting_panel.setFrameShape(QtWidgets.QFrame.StyledPanel)
            self.focus_setting_panel.setFrameShadow(QtWidgets.QFrame.Raised)
            self.focus_setting_panel.setObjectName("focus_setting_panel")
            self.focus_auto_button = QtWidgets.QPushButton(self.focus_setting_panel)
            self.focus_auto_button.setGeometry(QtCore.QRect(200, 20, 150, 50))
            self.focus_auto_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.focus_auto_button.setText("Auto")
            self.focus_auto_button.setCheckable(False)
            self.focus_auto_button.setObjectName("focus_auto_button")
            self.focus_spot_button = QtWidgets.QPushButton(self.focus_setting_panel)
            self.focus_spot_button.setGeometry(QtCore.QRect(380, 20, 150, 50))
            self.focus_spot_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.focus_spot_button.setCheckable(False)
            self.focus_spot_button.setObjectName("focus_spot_button")
            self.focus_spot_button.setText("Spot")
            self.focus_manual_button = QtWidgets.QPushButton(self.focus_setting_panel)
            self.focus_manual_button.setGeometry(QtCore.QRect(20, 20, 150, 50))
            self.focus_manual_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.focus_manual_button.setText("Manual")
            self.focus_manual_button.setCheckable(False)
            self.focus_manual_button.setObjectName("focus_manual_button")
            self.label_7 = QtWidgets.QLabel(self.focus_setting_panel)
            self.label_7.setGeometry(QtCore.QRect(20, 150, 51, 19))
            self.label_7.setStyleSheet("background-color: transparent;")
            self.label_7.setObjectName("label_7")
            self.label_7.setText("Mode:")
            self.focus_mode = QtWidgets.QLabel(self.focus_setting_panel)
            self.focus_mode.setGeometry(QtCore.QRect(70, 150, 141, 19))
            self.focus_mode.setStyleSheet("background-color: transparent;")
            self.focus_mode.setObjectName("focus_mode")
            self.focus_mode.setText(self.curr_focus_mode)
            self.focus_manual_bar = QtWidgets.QSlider(self.focus_setting_panel)
            self.focus_manual_bar.setGeometry(QtCore.QRect(20, 120, 511, 20))
            self.focus_manual_bar.setStyleSheet("")
            self.focus_manual_bar.setOrientation(QtCore.Qt.Horizontal)
            self.focus_manual_bar.setObjectName("focus_manual_bar")
            self.focus_manual_bar.setEnabled(False)
            self.focus_manual_bar.setMaximum(61440)
            self.focus_manual_bar.setMinimum(0)
            self.focus_manual_bar.setValue(self.focus_manual_val)
            self.focus_manual_bar.setSingleStep(100)
            self.focus_manual_bar.setStyleSheet("background-color: transparent;")
            self.focus_manual_bar.setTracking(False)
            self.label_8 = QtWidgets.QLabel(self.focus_setting_panel)
            self.label_8.setGeometry(QtCore.QRect(20, 90, 61, 19))
            self.label_8.setStyleSheet("background-color: transparent;")
            self.label_8.setObjectName("label_8")
            self.label_8.setText("Manual")

            ## Check bar status
            if self.curr_focus_mode == 'Manual':
                self.focus_manual_bar.setEnabled(True)

            ## Button link to focus mode function
            self.focus_auto_button.clicked.connect(self.focus_auto_pushed)
            self.focus_manual_button.clicked.connect(self.focus_manual_pushed)
            self.focus_spot_button.clicked.connect(self.focus_spot_pushed)

            ## Slider link to call back to change value
            self.focus_manual_bar.valueChanged[int].connect(self.focus_manual_valchange)

            self.focus_setting_panel.show()
        else:
            self.focus_setting_panel.deleteLater()

    def inspection_pushed(self):
        if self.inspection_button.isChecked():
            self.inspection_panel = QtWidgets.QFrame(self)
            self.inspection_panel.setGeometry(QtCore.QRect(340, 160, 571, 321))
            self.inspection_panel.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.inspection_panel.setFrameShape(QtWidgets.QFrame.StyledPanel)
            self.inspection_panel.setFrameShadow(QtWidgets.QFrame.Raised)
            self.inspection_panel.setObjectName("inspection_panel")
            self.OGI_button = QtWidgets.QPushButton(self.inspection_panel)
            self.OGI_button.setGeometry(QtCore.QRect(210, 150, 150, 50))
            self.OGI_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.OGI_button.setText("OGI")
            self.OGI_button.setObjectName("OGI_button")
            self.solar_button = QtWidgets.QPushButton(self.inspection_panel)
            self.solar_button.setGeometry(QtCore.QRect(390, 80, 150, 50))
            self.solar_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.solar_button.setObjectName("solar_button")
            self.solar_button.setText("Solar")
            self.windTurbine_button = QtWidgets.QPushButton(self.inspection_panel)
            self.windTurbine_button.setGeometry(QtCore.QRect(30, 80, 150, 50))
            self.windTurbine_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.windTurbine_button.setText("Wind Turbine")
            self.windTurbine_button.setObjectName("windTurbine_button")
            self.coco_button = QtWidgets.QPushButton(self.inspection_panel)
            self.coco_button.setGeometry(QtCore.QRect(30, 20, 150, 50))
            self.coco_button.setLayoutDirection(QtCore.Qt.RightToLeft)
            self.coco_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.coco_button.setText("COCO")
            self.coco_button.setObjectName("coco_button")
            self.visdrone_button = QtWidgets.QPushButton(self.inspection_panel)
            self.visdrone_button.setGeometry(QtCore.QRect(210, 20, 150, 50))
            self.visdrone_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.visdrone_button.setText("VisDrone")
            self.visdrone_button.setObjectName("visdrone_button")
            self.licensePlate_button = QtWidgets.QPushButton(self.inspection_panel)
            self.licensePlate_button.setGeometry(QtCore.QRect(390, 20, 150, 50))
            self.licensePlate_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.licensePlate_button.setText("License Plate")
            self.licensePlate_button.setObjectName("licensePlate_button")
            self.powerLine_button = QtWidgets.QPushButton(self.inspection_panel)
            self.powerLine_button.setGeometry(QtCore.QRect(210, 80, 150, 50))
            self.powerLine_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.powerLine_button.setText("Power Line")
            self.powerLine_button.setObjectName("powerLine_button")
            self.label_2 = QtWidgets.QLabel(self.inspection_panel)
            self.label_2.setGeometry(QtCore.QRect(10, 290, 51, 19))
            self.label_2.setObjectName("label_2")
            self.label_2.setText("Mode:")
            self.inspection_mode = QtWidgets.QLabel(self.inspection_panel)
            self.inspection_mode.setGeometry(QtCore.QRect(60, 290, 141, 19))
            self.inspection_mode.setObjectName("inspection_mode")
            self.inspection_mode.setText(self.curr_inspection)
            self.inspection_mode.setStyleSheet("background-color: transparent;")
            self.thermal_button = QtWidgets.QPushButton(self.inspection_panel)
            self.thermal_button.setGeometry(QtCore.QRect(30, 150, 150, 50))
            self.thermal_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.thermal_button.setText("Thermal")
            self.thermal_button.setObjectName("thermal_button")
            self.fire_button = QtWidgets.QPushButton(self.inspection_panel)
            self.fire_button.setGeometry(QtCore.QRect(390, 150, 150, 50))
            self.fire_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.fire_button.setText("Fire")
            self.fire_button.setObjectName("fire_button")
            self.telecom_button = QtWidgets.QPushButton(self.inspection_panel)
            self.telecom_button.setGeometry(QtCore.QRect(390, 220, 150, 50))
            self.telecom_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.telecom_button.setText("Telecom")
            self.telecom_button.setObjectName("telecom_button")
            self.drone_button = QtWidgets.QPushButton(self.inspection_panel)
            self.drone_button.setGeometry(QtCore.QRect(30, 220, 150, 50))
            self.drone_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.drone_button.setText("Drone")
            self.drone_button.setObjectName("drone_button")
            self.container_button = QtWidgets.QPushButton(self.inspection_panel)
            self.container_button.setGeometry(QtCore.QRect(210, 220, 150, 50))
            self.container_button.setStyleSheet("font: 75 15pt \"Ubuntu Condensed\";\n"
    "background-color: rgb(186, 189, 182);")
            self.container_button.setObjectName("container_button")
            self.container_button.setText("Container")

            self.inspection_panel.show()
        else:
            self.inspection_panel.deleteLater()

    def audio_setting_pushed(self):
        if self.audio_button.isChecked():
            self.audio_setting_panel = QtWidgets.QFrame(self)
            self.audio_setting_panel.setGeometry(QtCore.QRect(480, 270, 391, 141))
            self.audio_setting_panel.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.audio_setting_panel.setObjectName("audio_setting_panel")
            self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.audio_setting_panel)
            self.horizontalLayout_3.setObjectName("horizontalLayout_3")
            self.widget_2 = QtWidgets.QWidget(self.audio_setting_panel)
            self.widget_2.setStyleSheet("background-color: transparent;")
            self.widget_2.setObjectName("widget_2")
            self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.widget_2)
            self.verticalLayout_2.setObjectName("verticalLayout_2")
            self.alert_button = QtWidgets.QPushButton(self.widget_2)
            self.alert_button.setStyleSheet("background-color: transparent;")
            self.alert_button.setText("")
            icon = QtGui.QIcon()
            icon.addPixmap(QtGui.QPixmap(":/kirins/icon/system/alert.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.alert_button.setIcon(icon)
            self.alert_button.setIconSize(QtCore.QSize(55, 55))
            self.alert_button.setCheckable(True)
            self.alert_button.setFlat(True)
            self.alert_button.setObjectName("alert_button")
            self.verticalLayout_2.addWidget(self.alert_button)
            self.alert_label = QtWidgets.QLabel(self.widget_2)
            self.alert_label.setStyleSheet("background-color: transparent;")
            self.alert_label.setAlignment(QtCore.Qt.AlignCenter)
            self.alert_label.setObjectName("alert_label")
            self.verticalLayout_2.addWidget(self.alert_label)
            self.horizontalLayout_3.addWidget(self.widget_2)
            self.widget_3 = QtWidgets.QWidget(self.audio_setting_panel)
            self.widget_3.setStyleSheet("background-color: transparent;")
            self.widget_3.setObjectName("widget_3")
            self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.widget_3)
            self.verticalLayout_3.setObjectName("verticalLayout_3")
            self.streamingVoice_button = QtWidgets.QPushButton(self.widget_3)
            self.streamingVoice_button.setStyleSheet("background-color: transparent;")
            self.streamingVoice_button.setText("")
            icon1 = QtGui.QIcon()
            icon1.addPixmap(QtGui.QPixmap(":/kirins/icon/system/micro.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.streamingVoice_button.setIcon(icon1)
            self.streamingVoice_button.setIconSize(QtCore.QSize(55, 55))
            self.streamingVoice_button.setCheckable(True)
            self.streamingVoice_button.setFlat(True)
            self.streamingVoice_button.setObjectName("streamingVoice_button")
            self.verticalLayout_3.addWidget(self.streamingVoice_button)
            self.streamvoice_label = QtWidgets.QLabel(self.widget_3)
            self.streamvoice_label.setStyleSheet("background-color: transparent;")
            self.streamvoice_label.setAlignment(QtCore.Qt.AlignCenter)
            self.streamvoice_label.setWordWrap(True)
            self.streamvoice_label.setObjectName("streamvoice_label")
            self.verticalLayout_3.addWidget(self.streamvoice_label)
            self.horizontalLayout_3.addWidget(self.widget_3)
            self.widget_4 = QtWidgets.QWidget(self.audio_setting_panel)
            self.widget_4.setStyleSheet("background-color: transparent;")
            self.widget_4.setObjectName("widget_4")
            self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.widget_4)
            self.verticalLayout_4.setObjectName("verticalLayout_4")
            self.autoplayAlert_button = QtWidgets.QPushButton(self.widget_4)
            self.autoplayAlert_button.setStyleSheet("background-color: transparent;")
            self.autoplayAlert_button.setText("")
            icon2 = QtGui.QIcon()
            icon2.addPixmap(QtGui.QPixmap(":/kirins/icon/system/autoplay.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.autoplayAlert_button.setIcon(icon2)
            self.autoplayAlert_button.setIconSize(QtCore.QSize(55, 55))
            self.autoplayAlert_button.setCheckable(True)
            self.autoplayAlert_button.setFlat(True)
            self.autoplayAlert_button.setObjectName("autoplayAlert_button")
            self.verticalLayout_4.addWidget(self.autoplayAlert_button)
            self.autoplay_label = QtWidgets.QLabel(self.widget_4)
            self.autoplay_label.setStyleSheet("background-color: transparent;")
            self.autoplay_label.setAlignment(QtCore.Qt.AlignCenter)
            self.autoplay_label.setWordWrap(True)
            self.autoplay_label.setObjectName("autoplay_label")
            self.verticalLayout_4.addWidget(self.autoplay_label)
            self.horizontalLayout_3.addWidget(self.widget_4)

            font = QtGui.QFont()
            font.setBold(True)
            font.setWeight(75)
            self.alert_label.setFont(font)
            self.streamvoice_label.setFont(font)
            self.autoplay_label.setFont(font)
            self.alert_label.setText("Alert")
            self.streamvoice_label.setText("Streaming\nVoice")
            self.autoplay_label.setText("Autoplay\nAlert")

            self.audio_setting_panel.show()

        else:
            self.audio_setting_panel.deleteLater()

    def navi_setting_pushed(self):
        if self.navi_button.isChecked():
            self.navi_setting_panel = QtWidgets.QFrame(self)
            self.navi_setting_panel.setGeometry(QtCore.QRect(330, 270, 601, 141))
            self.navi_setting_panel.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.navi_setting_panel.setObjectName("navi_setting_panel")
            self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.navi_setting_panel)
            self.horizontalLayout_3.setObjectName("horizontalLayout_3")
            self.widget_2 = QtWidgets.QWidget(self.navi_setting_panel)
            self.widget_2.setStyleSheet("background-color: transparent;")
            self.widget_2.setObjectName("widget_2")
            self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.widget_2)
            self.verticalLayout_2.setObjectName("verticalLayout_2")
            self.follow_button = QtWidgets.QPushButton(self.widget_2)
            self.follow_button.setStyleSheet("background-color: transparent;")
            self.follow_button.setText("")
            icon = QtGui.QIcon()
            icon.addPixmap(QtGui.QPixmap(":/kirins/icon/system/follow.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.follow_button.setIcon(icon)
            self.follow_button.setIconSize(QtCore.QSize(55, 55))
            self.follow_button.setCheckable(True)
            self.follow_button.setFlat(True)
            self.follow_button.setObjectName("follow_button")
            self.verticalLayout_2.addWidget(self.follow_button)
            self.follow_label = QtWidgets.QLabel(self.widget_2)
            font = QtGui.QFont()
            font.setBold(True)
            font.setWeight(75)
            self.follow_label.setFont(font)
            self.follow_label.setStyleSheet("background-color: transparent;")
            self.follow_label.setAlignment(QtCore.Qt.AlignCenter)
            self.follow_label.setObjectName("follow_label")
            self.verticalLayout_2.addWidget(self.follow_label)
            self.horizontalLayout_3.addWidget(self.widget_2)
            self.widget_3 = QtWidgets.QWidget(self.navi_setting_panel)
            self.widget_3.setStyleSheet("background-color: transparent;")
            self.widget_3.setObjectName("widget_3")
            self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.widget_3)
            self.verticalLayout_3.setObjectName("verticalLayout_3")
            self.destroy_button = QtWidgets.QPushButton(self.widget_3)
            self.destroy_button.setStyleSheet("background-color: transparent;")
            self.destroy_button.setText("")
            icon1 = QtGui.QIcon()
            icon1.addPixmap(QtGui.QPixmap(":/kirins/icon/system/destroy.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.destroy_button.setIcon(icon1)
            self.destroy_button.setIconSize(QtCore.QSize(55, 55))
            self.destroy_button.setCheckable(True)
            self.destroy_button.setFlat(True)
            self.destroy_button.setObjectName("destroy_button")
            self.verticalLayout_3.addWidget(self.destroy_button)
            self.destroy_label = QtWidgets.QLabel(self.widget_3)
            font = QtGui.QFont()
            font.setBold(True)
            font.setWeight(75)
            self.destroy_label.setFont(font)
            self.destroy_label.setStyleSheet("background-color: transparent;")
            self.destroy_label.setAlignment(QtCore.Qt.AlignCenter)
            self.destroy_label.setWordWrap(True)
            self.destroy_label.setObjectName("destroy_label")
            self.verticalLayout_3.addWidget(self.destroy_label)
            self.horizontalLayout_3.addWidget(self.widget_3)
            self.widget_4 = QtWidgets.QWidget(self.navi_setting_panel)
            self.widget_4.setStyleSheet("background-color: transparent;")
            self.widget_4.setObjectName("widget_4")
            self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.widget_4)
            self.verticalLayout_4.setObjectName("verticalLayout_4")
            self.drop_button = QtWidgets.QPushButton(self.widget_4)
            self.drop_button.setStyleSheet("background-color: transparent;")
            self.drop_button.setText("")
            icon2 = QtGui.QIcon()
            icon2.addPixmap(QtGui.QPixmap(":/kirins/icon/system/drop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.drop_button.setIcon(icon2)
            self.drop_button.setIconSize(QtCore.QSize(55, 55))
            self.drop_button.setCheckable(True)
            self.drop_button.setFlat(True)
            self.drop_button.setObjectName("drop_button")
            self.verticalLayout_4.addWidget(self.drop_button)
            self.drop_label = QtWidgets.QLabel(self.widget_4)
            font = QtGui.QFont()
            font.setBold(True)
            font.setWeight(75)
            self.drop_label.setFont(font)
            self.drop_label.setStyleSheet("background-color: transparent;")
            self.drop_label.setAlignment(QtCore.Qt.AlignCenter)
            self.drop_label.setWordWrap(True)
            self.drop_label.setObjectName("drop_label")
            self.verticalLayout_4.addWidget(self.drop_label)
            self.horizontalLayout_3.addWidget(self.widget_4)
            self.widget_5 = QtWidgets.QWidget(self.navi_setting_panel)
            self.widget_5.setStyleSheet("background-color: transparent;")
            self.widget_5.setObjectName("widget_5")
            self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.widget_5)
            self.verticalLayout_5.setObjectName("verticalLayout_5")
            self.obstacle_button = QtWidgets.QPushButton(self.widget_5)
            self.obstacle_button.setStyleSheet("background-color: transparent;")
            self.obstacle_button.setText("")
            icon3 = QtGui.QIcon()
            icon3.addPixmap(QtGui.QPixmap(":/kirins/icon/system/obstacle.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.obstacle_button.setIcon(icon3)
            self.obstacle_button.setIconSize(QtCore.QSize(55, 55))
            self.obstacle_button.setCheckable(True)
            self.obstacle_button.setFlat(True)
            self.obstacle_button.setObjectName("obstacle_button")
            self.verticalLayout_5.addWidget(self.obstacle_button)
            self.obstacle_label = QtWidgets.QLabel(self.widget_5)
            font = QtGui.QFont()
            font.setBold(True)
            font.setWeight(75)
            self.obstacle_label.setFont(font)
            self.obstacle_label.setStyleSheet("background-color: transparent;")
            self.obstacle_label.setAlignment(QtCore.Qt.AlignCenter)
            self.obstacle_label.setWordWrap(True)
            self.obstacle_label.setObjectName("obstacle_label")
            self.verticalLayout_5.addWidget(self.obstacle_label)
            self.horizontalLayout_3.addWidget(self.widget_5)
            self.widget_6 = QtWidgets.QWidget(self.navi_setting_panel)
            self.widget_6.setStyleSheet("background-color: transparent;")
            self.widget_6.setObjectName("widget_6")
            self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.widget_6)
            self.verticalLayout_6.setObjectName("verticalLayout_6")
            self.safe_button = QtWidgets.QPushButton(self.widget_6)
            self.safe_button.setStyleSheet("background-color: transparent;")
            self.safe_button.setText("")
            icon4 = QtGui.QIcon()
            icon4.addPixmap(QtGui.QPixmap(":/kirins/icon/system/safe.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            self.safe_button.setIcon(icon4)
            self.safe_button.setIconSize(QtCore.QSize(55, 55))
            self.safe_button.setCheckable(True)
            self.safe_button.setFlat(True)
            self.safe_button.setObjectName("safe_button")
            self.verticalLayout_6.addWidget(self.safe_button)
            self.safe_label = QtWidgets.QLabel(self.widget_6)
            font = QtGui.QFont()
            font.setBold(True)
            font.setWeight(75)
            self.safe_label.setFont(font)
            self.safe_label.setStyleSheet("background-color: transparent;")
            self.safe_label.setAlignment(QtCore.Qt.AlignCenter)
            self.safe_label.setWordWrap(True)
            self.safe_label.setObjectName("safe_label")
            self.verticalLayout_6.addWidget(self.safe_label)
            self.horizontalLayout_3.addWidget(self.widget_6)

            self.follow_label.setText("Follow")
            self.destroy_label.setText("Destroy")
            self.drop_label.setText("Drop")
            self.obstacle_label.setText("Obstacle\nAvoidance")
            self.safe_label.setText("Safe\nLanding")

            self.navi_setting_panel.show()

        else:
            self.navi_setting_panel.deleteLater()
  
    @pyqtSlot(np.ndarray)
    # Set image and draw point, bounding box in pixmap
    def update_rgb_frame(self, cv_img):
        """Updates the image_label with a new opencv image"""
        global curr_camera, client_socket, detect_list, drawing_detectbox, drawing_SOT_outbox
        global rgb_x, rgb_y, rgb_w, rgb_h, conf

        if curr_camera == 'rgb':
            # Check mode
            if not self.infer_button.isChecked():
                drawing_detectbox = False 
            else:
                # Get the class from class filter box
                self.class_filter = self.classes_filter_box.currentText()

            if not self.SOT_button.isChecked():
                drawing_SOT_outbox = False
                self.drawing_SOT_inbox = False

            if self.dual_button.isChecked():
                self.w = -1
                self.h = -1

            if self.loading_model:
                start_time = time.time()
                while True:
                    if self.widget_progressBar:
                        loading_time = time.time() - start_time
                        if drawing_detectbox or loading_time >= 3:  
                            # Setting value to progress bar
                            self.progressBar.setValue(100)
                            self.widget_progressBar.deleteLater()
                            self.loading_model = False
                            self.infer_button.setEnabled(True)
                            break          
                  
            # Draw detection bbox
            if drawing_detectbox:             
                new_detect_list = detect_list
                num = 0
                for i in range(len(new_detect_list)):

                    # Add new class to classes filter box
                    if new_detect_list[i]['class'] not in self.classes:
                        self.class_id += 1
                        self.classes.append(new_detect_list[i]['class'])
                        self.classes_filter_box.addItem("")
                        self.classes_filter_box.setItemText(self.class_id, new_detect_list[i]['class'])

                    if self.class_filter == 'all' or new_detect_list[i]['class'] == self.class_filter:
                        center_x = new_detect_list[i]['x'] + new_detect_list[i]['w']/2
                        center_y = new_detect_list[i]['y'] + new_detect_list[i]['h']/2
                        x1 = int(center_x - new_detect_list[i]['w']/2)
                        y1 = int(center_y - new_detect_list[i]['h']/2)
                        x2 = int(center_x + new_detect_list[i]['w']/2)
                        y2 = int(center_y + new_detect_list[i]['h']/2)

                        cv2.rectangle(cv_img, (x1, y1), (x2, y2), (255,0,0), 2)
                        cv2.rectangle(cv_img, (x1, y1-23), (x2-10, y1), (255,0,0), -1)
                        
                        if new_detect_list[i]['is_tracking'] == True:
                            label = new_detect_list[i]['class'] + " " + new_detect_list[i]['tracking_id']
                        else:
                            label = new_detect_list[i]['class']

                        cv2.putText(img = cv_img,
                                    text = label,
                                    org = (x1, y1),
                                    fontFace = cv2.FONT_HERSHEY_DUPLEX,
                                    fontScale = 1,
                                    color = (255, 255, 255),
                                    thickness = 1,
                                    lineType = cv2.FILLED)
                        num += 1

                # Show number of objects
                self.label_classFilter.setText(f"Number of Objects: {num}")  

            # Draw SOT inbox
            if self.drawing_SOT_inbox:
                if self.w != -1 and self.h != -1:
                    cv2.rectangle(cv_img,(self.x1,self.y1),(self.x1+self.w,self.y1+self.h),(180,105,255),3)
            
            # Draw point for point to move mode
            if self.drawing_point:
                cv2.circle(cv_img, (self.begin.x(), self.begin.y()), 10, (0,255,0), -1)
                self.drawing_point = False

        # Draw SOT outbox
        if drawing_SOT_outbox:
            self.autoZoom.setEnabled(True)
            if curr_camera == 'rgb' or self.syncbbox_button.isChecked():
                if conf > 0.5:
                    self.label_searching.setText("")

                    top_left1 = int(rgb_x), int(rgb_y)
                    bot_left1 = int(rgb_x), int(rgb_y) + int(rgb_h)
                    top_right1 = int(rgb_x) + int(rgb_w), int(rgb_y)
                    bot_right1 = int(rgb_x)+int(rgb_w),int(rgb_y)+int(rgb_h)
                    draw_SOT_outbox(cv_img, top_left1, bot_left1, top_right1, bot_right1)
                else:  
                    self.label_searching.setText("Searching...")

        if self.dual_button.isChecked():
            # Resize image 
            cv_img = cv2.resize(cv_img, (640, 360))
            # Convert opencv format into qt format
            qt_img = self.convert_cv2qt(cv_img, 640, 360)
            self.rgb_stream.setPixmap(qt_img)
        else:
            if curr_camera == 'rgb':
                qt_img = self.convert_cv2qt(cv_img, 1280, 720)
                self.stream.setPixmap(qt_img)
                self.stream.setAlignment(QtCore.Qt.AlignCenter)

    @pyqtSlot(np.ndarray)
    # Set image and draw point, bounding box in pixmap
    def update_thermal_frame(self, cv_img):
        """Updates the image_label with a new opencv image"""
        global curr_camera, client_socket, detect_list, drawing_detectbox, drawing_SOT_outbox
        global ther_x, ther_y, ther_w, ther_h, conf

        if curr_camera == 'thermal':
            # Check mode
            if not self.infer_button.isChecked():
                drawing_detectbox = False
            else:
                # Get the class from class filter box
                self.class_filter = self.classes_filter_box.currentText() 

            if not self.SOT_button.isChecked():
                drawing_SOT_outbox = False
                self.drawing_SOT_inbox = False

            if self.loading_model:
                start_time = time.time()
                while True:
                    if self.widget_progressBar:
                        loading_time = time.time() - start_time
                        if drawing_detectbox or loading_time >= 5:  
                            # Setting value to progress bar
                            self.progressBar.setValue(100)
                            self.widget_progressBar.deleteLater()
                            self.loading_model = False
                            self.infer_button.setEnabled(True)
                            break  

            # Draw detection bbox
            if drawing_detectbox:
                self.autoZoom.setEnabled(True)
                new_detect_list = detect_list
                num = 0
                for i in range(len(new_detect_list)):

                    # Add new class to classes filter box
                    if new_detect_list[i]['class'] not in self.classes:
                        self.class_id += 1
                        self.classes.append(new_detect_list[i]['class'])
                        self.classes_filter_box.addItem("")
                        self.classes_filter_box.setItemText(self.class_id, new_detect_list[i]['class'])

                    if self.class_filter == 'all' or new_detect_list[i]['class'] == self.class_filter:
                        center_x = new_detect_list[i]['x'] + new_detect_list[i]['w']/2
                        center_y = new_detect_list[i]['y'] + new_detect_list[i]['h']/2
                        x1 = int(center_x - new_detect_list[i]['w']/2)
                        y1 = int(center_y - new_detect_list[i]['h']/2)
                        x2 = int(center_x + new_detect_list[i]['w']/2)
                        y2 = int(center_y + new_detect_list[i]['h']/2)

                        cv2.rectangle(cv_img, (x1, y1), (x2, y2), (255,0,0), 2)
                        cv2.rectangle(cv_img, (x1, y1-23), (x2-10, y1), (255,0,0), -1)
                        
                        if new_detect_list[i]['is_tracking'] == True:
                            label = new_detect_list[i]['class'] + " " + new_detect_list[i]['tracking_id']
                        else:
                            label = new_detect_list[i]['class']

                        cv2.putText(img = cv_img,
                                text = label,
                                org = (x1, y1),
                                fontFace = cv2.FONT_HERSHEY_DUPLEX,
                                fontScale = 1,
                                color = (255, 255, 255),
                                thickness = 1,
                                lineType = cv2.FILLED)
                        num += 1
                
                # Show number of objects
                self.label_classFilter.setText(f"Number of Objects: {num}")  

            # Draw SOT inbox
            if self.drawing_SOT_inbox:
                if self.w != -1 and self.h != -1:
                    cv2.rectangle(cv_img,(self.x1,self.y1),(self.x1+self.w,self.y1+self.h),(180,105,255),3)
            
            # Draw point for point to move mode
            if self.drawing_point:
                cv2.circle(cv_img, (self.begin.x(), self.begin.y()), 10, (0,255,0), -1)
                self.drawing_point = False

        # Draw SOT outbox
        if drawing_SOT_outbox:
            if curr_camera == 'thermal' or self.syncbbox_button.isChecked():
                if conf > 0.5:
                    self.label_searching.setText("")

                    top_left2 = int(ther_x), int(ther_y)
                    bot_left2 = int(ther_x), int(ther_y) + int(ther_h)
                    top_right2 = int(ther_x) + int(ther_w), int(ther_y)
                    bot_right2 = int(ther_x)+int(ther_w),int(ther_y)+int(ther_h)
                    draw_SOT_outbox(cv_img, top_left2, bot_left2, top_right2, bot_right2)
                else:
                    self.label_searching.setText("Searching...")
        
        if self.dual_button.isChecked():
            # Resize image 
            cv_img = cv2.resize(cv_img, (640, 360))

            # Convert opencv format into qt format
            qt_img = self.convert_cv2qt(cv_img, 640, 360)
            self.thermal_stream.setPixmap(qt_img)
        else:
            if curr_camera == 'thermal':
                qt_img = self.convert_cv2qt(cv_img, 1280, 720)
                self.stream.setPixmap(qt_img)
                self.stream.setAlignment(QtCore.Qt.AlignCenter)
        
    def convert_cv2qt(self, cv_img, frame_width, frame_height):
        """Convert from an opencv image to QPixmap"""
        self.scale = frame_width/self.win_width
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        frame_height, frame_width, ch = rgb_img.shape
        bytes_per_line = ch * frame_width
        convert_to_qt_format = QtGui.QImage(rgb_img.data, frame_width, frame_height, bytes_per_line, QtGui.QImage.Format_RGB888)
        resize_img = convert_to_qt_format.scaled(frame_width, frame_height, Qt.KeepAspectRatio)
        return QPixmap(resize_img)
    
    def scale_pos(self):
        self.w = int(abs(self.begin.x()-self.destination.x())*self.scale)
        self.h = int(abs(self.begin.y()-self.destination.y())*self.scale)
        if self.begin.x() < self.destination.x():
            self.x1 = int((self.begin.x()-self.space)*self.scale)
        else:
            self.x1 = int((self.destination.x()-self.space)*self.scale)
        if self.begin.y() < self.destination.y():
            self.y1 = int((self.begin.y()-self.space)*self.scale)
        else:
            self.y1 = int((self.destination.y()-self.space)*self.scale)

    #####------------------------------------------------------#####
    #####                     MOUSE callback                   #####
    #####------------------------------------------------------#####    

    def paintEvent(self,event):
        painter = QPainter(self)
        painter.drawPixmap(QPoint(),self.pix)
        if not self.begin.isNull() and not self.destination.isNull():
            rect = QRect(self.begin, self.destination)
            painter.drawRect(rect.normalized())
            
    def mousePressEvent(self, event):
        global client_socket
        if self.SOT_button.isChecked():
            if event.buttons() == Qt.LeftButton:
                self.begin = event.pos()
                self.destination = self.begin
                if 1060 < self.begin.x() < 1151 and 110 < self.begin.y() < 521:
                    self.drawing_SOT_inbox = False
                else:
                    self.drawing_SOT_inbox = True
                    self.scale_pos()
                    self.update()
            elif event.buttons() == Qt.RightButton:
                self.start_right_mouse = event.pos()
                self.end_right_mouse = self.start_right_mouse

        if self.touch_button.isChecked():
            if event.buttons() == Qt.LeftButton:
                ## Disable SOT button
                self.begin = event.pos()

                ## Send point to move message via Socket
                msg.point_to_move['point']['data']['x'] = self.begin.x()
                msg.point_to_move['point']['data']['y'] = self.begin.y()
                client_socket.send(pickle.dumps(msg.point_to_move))
                self.scale_pos()
                self.update()
                self.drawing_point = True

        if self.infer_button.isChecked() and not self.MOT_button.isChecked():
            if event.buttons() == Qt.RightButton:
                self.begin = event.pos()
                msg.status['status']['data'] = 'SOT'
                self.SOT_button.setEnabled(True) 
                self.SOT_button.setChecked(True)   
                self.syncbbox_button.setEnabled(True) 
                client_socket.send(pickle.dumps(msg.status))

                self.label_searching = QtWidgets.QLabel(self)
                self.label_searching.setGeometry(QtCore.QRect(600, 20, 111, 21))
                font = QtGui.QFont()
                font.setPointSize(13)
                font.setBold(True)
                font.setWeight(75)
                self.label_searching.setFont(font)
                self.label_searching.setStyleSheet("color: rgb(204, 0, 0);\n")
                self.label_searching.setAlignment(QtCore.Qt.AlignCenter)
                self.label_searching.setObjectName("label_2")
                self.label_searching.setText("")
                self.label_searching.show()

                new_bbox = get_closest_bbox([self.begin.x(), self.begin.y()], detect_list)
                msg.input_bbox['input_bbox']['data']['x1'] = new_bbox[0]
                msg.input_bbox['input_bbox']['data']['y1'] = new_bbox[1]
                msg.input_bbox['input_bbox']['data']['x2'] = new_bbox[2]
                msg.input_bbox['input_bbox']['data']['y2'] = new_bbox[3]
                time.sleep(0.5)
                client_socket.send(pickle.dumps(msg.input_bbox))
                self.scale_pos()
                self.update()
                self.infer_button.setChecked(False)  
                self.infer_button.setEnabled(False)  
                self.MOT_button.setEnabled(False)     
                self.touch_button.setEnabled(False)
                self.classes_filter_box.deleteLater()
                self.label_classFilter.deleteLater()

        ## Click point to focus
        if self.curr_focus_mode == 'Spot':
            if event.buttons() == Qt.LeftButton:
                self.begin = event.pos()

                msg.focus_position['visca_command']['command_id'] = ViscaPacketsEnum.COMMAND_CAM_FOCUS_SPOT_POSITION 
                msg.focus_position['visca_command']['data']['param1'] = self.begin.x()
                msg.focus_position['visca_command']['data']['param2'] = self.begin.y()
                msg.focus_position['visca_command']['data']['param3'] = 0
                client_socket.send(pickle.dumps(msg.focus_position)) 

                self.scale_pos()
                self.update()
                self.drawing_point = True

        ## Click point to AE
        if self.curr_eps_mode == 'Spot':
            if event.buttons() == Qt.LeftButton:
                self.begin = event.pos()

                msg.AE_position['visca_command']['command_id'] = ViscaPacketsEnum.COMMAND_CAM_SPOTAE_POSITION 
                msg.AE_position['visca_command']['data']['param1'] = self.begin.x()
                msg.AE_position['visca_command']['data']['param2'] = self.begin.y()
                msg.AE_position['visca_command']['data']['param3'] = 0
                client_socket.send(pickle.dumps(msg.AE_position)) 
                self.scale_pos()
                self.update()
                self.drawing_point = True
            
    def mouseMoveEvent(self, event):
        if self.SOT_button.isChecked():
            if event.buttons() == Qt.LeftButton:
                self.destination = event.pos()
                if 1060 < self.destination.x() < 1151 and 110 < self.destination.y() < 521:
                    self.drawing_SOT_inbox = False
                else:
                    self.drawing_SOT_inbox = True
                    self.scale_pos()
                    self.update()
            elif event.buttons() == Qt.RightButton:
                self.right_mouse_move = True
                self.end_right_mouse = event.pos()
            
    def mouseReleaseEvent(self, event):
        global client_socket
        if self.SOT_button.isChecked():
            if event.button() == Qt.LeftButton:
 
                # Send SOT_input_bbox message
                if 1060 < self.begin.x() < 1151 and 110 < self.begin.y() < 521:
                    print("Draw bbox out of zoombar pls ")
                    self.drawing_SOT_inbox = False
                else:
                    rect = QRect(self.begin, self.destination)
                    ## Send boundingBox location
                    self.scale_pos()

                    dist = sqrt(self.w**2 + self.h**2)
                    if dist > 50:
                        msg.input_bbox['input_bbox']['data']['x1'] = self.x1
                        msg.input_bbox['input_bbox']['data']['y1'] = self.y1
                        msg.input_bbox['input_bbox']['data']['x2'] = self.x1 + self.w
                        msg.input_bbox['input_bbox']['data']['y2'] = self.y1 + self.h
                        client_socket.send(pickle.dumps(msg.input_bbox))

                    ## Draw bounding box
                    painter = QPainter(self.pix)
                    painter.drawRect(rect.normalized())

                    self.drawing_SOT_inbox = False

                    ## Release position and update
                    self.x1, self.y1, self.w, self.h = -1, -1, -1, -1
                    self.begin, self.destination = QPoint(), QPoint()
                    self.update()
                
            elif event.button() == Qt.RightButton:
                if self.right_mouse_move:
                    w = self.end_right_mouse.x()-self.start_right_mouse.x()
                    h = self.end_right_mouse.y()-self.start_right_mouse.y()
                    if abs(w) > abs(h):
                        if w < 0:
                            print('left')
                        else:
                            print('right')
                    else:
                        if h > 0:
                            print('bot')
                        else:
                            print('top')
                else:
                    self.x1, self.y1, self.w, self.h = -1, -1, -1, -1
                    self.start_right_mouse, self.end_right_mouse = QPoint(), QPoint()
                self.right_mouse_move = False


    #####------------------------------------------------------#####
    #####                    BUTTON callback                   #####
    #####------------------------------------------------------#####

    #============= AI =============
    def infer_pushed(self, *args):
        global client_socket
        msg.status['status']['data'] = 'infer'
        client_socket.send(pickle.dumps(msg.status))
        if self.infer_button.isChecked():
            self.MOT_button.setEnabled(True)
            self.SOT_button.setEnabled(False)
            self.camera_button.setEnabled(False)

            # Choose class filter
            self.classes_filter_box = QtWidgets.QComboBox(self)
            self.classes_filter_box.setGeometry(QtCore.QRect(16, 80, 118, 27))
            self.classes_filter_box.setObjectName("classes_filter_box")
            
            # Add a scroll bar to the combo box
            self.classes_filter_box.view().setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
            self.classes_filter_box.setStyleSheet("QComboBox { combobox-popup: 0; }")

            self.classes = ["all"]
            self.classes_filter_box.addItem("")
            self.classes_filter_box.setItemText(0, self.classes[0])
            self.class_id = 0

            self.classes_filter_box.show()

            # Loading infer model
            self.widget_progressBar = QtWidgets.QWidget(self)
            self.widget_progressBar.setGeometry(QtCore.QRect(560, 290, 161, 81))
            self.widget_progressBar.setObjectName("widget")
            self.verticalLayout = QtWidgets.QVBoxLayout(self.widget_progressBar)
            self.verticalLayout.setContentsMargins(0, 0, 0, 0)
            self.verticalLayout.setObjectName("verticalLayout")
            self.label_loading = QtWidgets.QLabel(self.widget_progressBar)
            font = QtGui.QFont()
            font.setPointSize(13)
            font.setBold(True)
            font.setWeight(75)
            self.label_loading.setFont(font)
            self.label_loading.setAlignment(QtCore.Qt.AlignCenter)
            self.label_loading.setObjectName("label")
            self.label_loading.setText("Loading...")
            self.verticalLayout.addWidget(self.label_loading)
            self.progressBar = QtWidgets.QProgressBar(self.widget_progressBar)
            font = QtGui.QFont()
            font.setPointSize(12)
            font.setBold(True)
            font.setWeight(75)
            self.progressBar.setFont(font)
            self.progressBar.setCursor(QtGui.QCursor(QtCore.Qt.BusyCursor))
            self.progressBar.setProperty("value", 1)
            self.progressBar.setObjectName("progressBar")
            self.verticalLayout.addWidget(self.progressBar)
            self.widget_progressBar.show()

            self.loading_model = True
            self.infer_button.setEnabled(False)

            self.label_classFilter = QtWidgets.QLabel(self)
            self.label_classFilter.setGeometry(QtCore.QRect(550, 20, 300, 21))
            font = QtGui.QFont()
            font.setPointSize(13)
            font.setBold(True)
            font.setWeight(75)
            self.label_classFilter.setFont(font)
            self.label_classFilter.setStyleSheet("color: rgb(0, 0, 255);\n")
            self.label_classFilter.setAlignment(QtCore.Qt.AlignCenter)
            self.label_classFilter.setObjectName("label_2")
            self.label_classFilter.setText("")
            self.label_classFilter.show()

            for i in range(100):
                # Slowing down the loop
                time.sleep(0.02)
                self.progressBar.setValue(i)

        else:
            self.MOT_button.setEnabled(False)
            self.SOT_button.setEnabled(True)
            self.camera_button.setEnabled(True)

            self.classes_filter_box.deleteLater()
            self.label_classFilter.deleteLater()


    def MOT_pushed(self, *args):
        global client_socket
        msg.status['status']['data'] = 'MOT'
        client_socket.send(pickle.dumps(msg.status))
        if self.MOT_button.isChecked():
            self.infer_button.setEnabled(False)
        else:
            self.infer_button.setEnabled(True)

    def SOT_pushed(self, *args):
        global client_socket
        msg.status['status']['data'] = 'SOT'
        client_socket.send(pickle.dumps(msg.status))
        if self.SOT_button.isChecked():
            self.syncbbox_button.setEnabled(True)
            self.infer_button.setEnabled(False)
            self.MOT_button.setEnabled(False)
            self.touch_button.setEnabled(False)
            # self.autoZoom.setEnabled(True)
            self.camera_button.setEnabled(False)

            self.label_searching = QtWidgets.QLabel(self)
            self.label_searching.setGeometry(QtCore.QRect(600, 20, 111, 21))
            font = QtGui.QFont()
            font.setPointSize(13)
            font.setBold(True)
            font.setWeight(75)
            self.label_searching.setFont(font)
            self.label_searching.setStyleSheet("color: rgb(204, 0, 0);\n")
            self.label_searching.setAlignment(QtCore.Qt.AlignCenter)
            self.label_searching.setObjectName("label_2")
            self.label_searching.setText("")
            self.label_searching.show()
        else:
            self.syncbbox_button.setChecked(False)
            self.syncbbox_button.setEnabled(False)
            self.infer_button.setEnabled(True)
            self.touch_button.setEnabled(True)
            self.autoZoom.setChecked(False)
            self.autoZoom.setEnabled(False)
            self.camera_button.setEnabled(True)

            self.label_searching.deleteLater()

    def syncbbox_pushed(self, *args):
        if self.syncbbox_button.isChecked():
            self.SOT_button.setEnabled(False)
        else:
            self.SOT_button.setEnabled(True)
    
    #============= CAMERA =============
    def reset_pushed(self, *args):
        global client_socket

        self.curr_autozoom = False
        msg.zoom2bbox['zoom2bbox']['data'] = False
        client_socket.send(pickle.dumps(msg.zoom2bbox))

        msg.reset['visca_command']['command_id'] = ViscaPacketsEnum.RESET
        msg.reset['visca_command']['data']['param1'] = 0
        msg.reset['visca_command']['data']['param2'] = 0
        msg.reset['visca_command']['data']['param3'] = 0
        client_socket.send(pickle.dumps(msg.reset))

        ## Reset all camera mode
        if self.curr_focus_mode != 'Auto':
            self.curr_focus_mode = 'Auto'
        if self.curr_eps_mode != 'Full Auto':
            self.curr_eps_mode = 'Full Auto'

        self.focus_manual_val = 0
        self.gain_val = 1
        self.iris_val = 0
        self.shutter_val = 6
        self.zoom_bar.setValue(1)

        self.curr_stabilize = False
        self.curr_defog = False
        self.curr_record = False
        self.curr_backlight = False
        self.curr_sensitivity = False

        self.curr_wb_mode = 'Auto'
        self.curr_monitor_mode = '2160p/29.97'

        self.autoZoom.setChecked(False)
        
    def dual_pushed(self, *args):
        if self.dual_button.isChecked():
            self.stream.clear()
            # self.zoom_bar.setEnabled(False)
            self.focus_button.setEnabled(False)
            self.EPS_button.setEnabled(False)
            self.setting_button.setEnabled(False)
            self.touch_button.setChecked(False)
            self.touch_button.setEnabled(False)
            self.reset_button.setEnabled(False)
            self.SOT_button.setEnabled(False)

            self.lock_button.setEnabled(False)
            # self.lock_button.hide()

            self.up_button.setEnabled(False)
            self.down_button.setEnabled(False)
            self.right_button.setEnabled(False)
            self.left_button.setEnabled(False)
            self.home_button.setEnabled(False)
            self.syncgimbal_button.setEnabled(False)

        else:
            self.rgb_stream.clear()
            self.thermal_stream.clear()
            # self.zoom_bar.setEnabled(True)
            self.focus_button.setEnabled(True)
            self.EPS_button.setEnabled(True)
            self.setting_button.setEnabled(True)
            self.touch_button.setEnabled(True)
            self.reset_button.setEnabled(True)
            self.SOT_button.setEnabled(True)   

            self.lock_button.setEnabled(True)
            # self.lock_button.show()

            self.up_button.setEnabled(True)
            self.down_button.setEnabled(True)
            self.right_button.setEnabled(True)
            self.left_button.setEnabled(True)
            self.home_button.setEnabled(True) 
            self.syncgimbal_button.setEnabled(True)

    def camera_pushed(self, *args):
        global client_socket, curr_camera
        switcher={
            'rgb': 'thermal',
            'thermal': 'rgb'
        }
        curr_camera = switcher.get(curr_camera)
        msg.cam_mode['mode_camera']['data'] = curr_camera
        client_socket.send(pickle.dumps(msg.cam_mode))

    def focus_mode_msg(self, command_id, param1=0, param2=0):
        msg.focus_mode['visca_command']['command_id'] = command_id
        msg.focus_mode['visca_command']['data']['param1'] = param1
        msg.focus_mode['visca_command']['data']['param2'] = param2
        msg.focus_mode['visca_command']['data']['param3'] = 0
        return msg.focus_mode
    
    def focus_manual_pushed(self, *args):
        global client_socket
        self.focus_manual_bar.setEnabled(True)
        self.curr_focus_mode = "Manual"
        self.focus_mode.setText(self.curr_focus_mode)
        client_socket.send(pickle.dumps(self.focus_mode_msg(ViscaPacketsEnum.COMMAND_CAM_FOCUS_MANUAL_FOCUS_ON)))

    def focus_auto_pushed(self, *args):
        global client_socket
        self.focus_manual_val = 0
        self.focus_manual_bar.setValue(self.focus_manual_val)
        self.focus_manual_bar.setEnabled(False)
        self.curr_focus_mode = "Auto"
        self.focus_mode.setText(self.curr_focus_mode)
        client_socket.send(pickle.dumps(self.focus_mode_msg(ViscaPacketsEnum.COMMAND_CAM_FOCUS_AUTO_FOCUS_ON)))
    
    def focus_spot_pushed(self, *args):
        global client_socket
        self.focus_manual_val = 0
        self.focus_manual_bar.setValue(self.focus_manual_val)
        self.focus_manual_bar.setEnabled(False)
        self.curr_focus_mode = "Spot"
        self.focus_mode.setText(self.curr_focus_mode)
        client_socket.send(pickle.dumps(self.focus_mode_msg(ViscaPacketsEnum.COMMAND_CAM_FOCUS_SPOT_ON)))

    def exposure_mode_msg(self, command_id, param1=0, param2=0):
        msg.exposure_mode['visca_command']['command_id'] = command_id
        msg.exposure_mode['visca_command']['data']['param1'] = param1
        msg.exposure_mode['visca_command']['data']['param2'] = param2
        msg.exposure_mode['visca_command']['data']['param3'] = 0
        return msg.exposure_mode

    def eps_fullAuto_pushed(self, *args):
        global client_socket
        self.gain_val = 1
        self.iris_val = 0
        self.shutter_val = 6
        self.gain_bar.setValue(self.gain_val)
        self.iris_bar.setValue(self.iris_val)
        self.shutter_bar.setValue(self.shutter_val)
        self.gain_bar.setEnabled(False)
        self.iris_bar.setEnabled(False)
        self.shutter_bar.setEnabled(False)
        self.curr_eps_mode = "Full Auto"
        self.exposure_mode.setText(self.curr_eps_mode)
        client_socket.send(pickle.dumps(self.exposure_mode_msg(ViscaPacketsEnum.COMMAND_CAM_AE_FULL_AUTO)))

    def eps_manual_pushed(self, *args):
        global client_socket
        self.gain_bar.setEnabled(True)
        self.iris_bar.setEnabled(True)
        self.shutter_bar.setEnabled(True)
        self.curr_eps_mode = "Manual"
        self.exposure_mode.setText(self.curr_eps_mode)
        client_socket.send(pickle.dumps(self.exposure_mode_msg(ViscaPacketsEnum.COMMAND_CAM_AE_MANUAL)))

    def shutter_pushed(self, *args):
        global client_socket
        self.gain_val = 1
        self.iris_val = 0
        self.shutter_val = 6
        self.gain_bar.setValue(self.gain_val)
        self.iris_bar.setValue(self.iris_val)
        self.shutter_bar.setValue(self.shutter_val)
        self.gain_bar.setEnabled(False)
        self.iris_bar.setEnabled(False)
        self.shutter_bar.setEnabled(True)
        self.curr_eps_mode = "Shutter Priority"
        self.exposure_mode.setText(self.curr_eps_mode)
        client_socket.send(pickle.dumps(self.exposure_mode_msg(ViscaPacketsEnum.COMMAND_CAM_AE_SHUTTER_PRIORITY)))
    
    def iris_pushed(self, *args):
        global client_socket
        self.gain_val = 1
        self.iris_val = 0
        self.shutter_val = 6
        self.gain_bar.setValue(self.gain_val)
        self.iris_bar.setValue(self.iris_val)
        self.shutter_bar.setValue(self.shutter_val)
        self.gain_bar.setEnabled(False)
        self.iris_bar.setEnabled(True)
        self.shutter_bar.setEnabled(False)
        self.curr_eps_mode = "IRIS Priority"
        self.exposure_mode.setText(self.curr_eps_mode)
        client_socket.send(pickle.dumps(self.exposure_mode_msg(ViscaPacketsEnum.COMMAND_CAM_AE_IRIS_PRIORITY)))

    def gain_pushed(self, *args):
        global client_socket
        self.gain_val = 1
        self.iris_val = 0
        self.shutter_val = 6
        self.gain_bar.setValue(self.gain_val)
        self.iris_bar.setValue(self.iris_val)
        self.shutter_bar.setValue(self.shutter_val)
        self.gain_bar.setEnabled(True)
        self.iris_bar.setEnabled(False)
        self.shutter_bar.setEnabled(False)
        self.curr_eps_mode = "Gain Priority"
        self.exposure_mode.setText(self.curr_eps_mode)
        client_socket.send(pickle.dumps(self.exposure_mode_msg(ViscaPacketsEnum.COMMAND_CAM_AE_GAIN_PRIORITY)))

    def bright_pushed(self, *args):
        global client_socket
        self.gain_val = 1
        self.iris_val = 0
        self.shutter_val = 6
        self.gain_bar.setValue(self.gain_val)
        self.iris_bar.setValue(self.iris_val)
        self.shutter_bar.setValue(self.shutter_val)
        self.gain_bar.setEnabled(True)
        self.iris_bar.setEnabled(True)
        self.shutter_bar.setEnabled(True)
        self.curr_eps_mode = "Bright"
        self.exposure_mode.setText(self.curr_eps_mode)
        client_socket.send(pickle.dumps(self.exposure_mode_msg(ViscaPacketsEnum.COMMAND_CAM_AE_BRIGHT)))

    def eps_spot_pushed(self, *args):
        global client_socket
        self.gain_val = 1
        self.iris_val = 0
        self.shutter_val = 6
        self.gain_bar.setValue(self.gain_val)
        self.iris_bar.setValue(self.iris_val)
        self.shutter_bar.setValue(self.shutter_val)
        self.gain_bar.setEnabled(False)
        self.iris_bar.setEnabled(False)
        self.shutter_bar.setEnabled(False)
        self.curr_eps_mode = "Spot"
        self.exposure_mode.setText(self.curr_eps_mode)
        client_socket.send(pickle.dumps(self.exposure_mode_msg(ViscaPacketsEnum.COMMAND_CAM_SPOTAE_ON)))

    #============= CAMERA SETTING =============
    def autozoom_pushed(self, *args):
        global client_socket
        if self.autoZoom.isChecked():
            self.zoom_bar.setEnabled(False)
            self.zoom_bar.setValue(1)
            self.curr_autozoom = True
            msg.zoom2bbox['zoom2bbox']['data'] = True
        else: 
            self.zoom_bar.setEnabled(True)
            self.curr_autozoom = False
            msg.zoom2bbox['zoom2bbox']['data'] = False
        client_socket.send(pickle.dumps(msg.zoom2bbox))

    def stabilize_pushed(self, *args):
        global client_socket
        if self.stabilize_button.isChecked():
            self.curr_stabilize = True
            command_id = ViscaPacketsEnum.COMMAND_CAM_STABILIZER_ON
        else:
            self.curr_stabilize = False
            command_id = ViscaPacketsEnum.COMMAND_CAM_STABILIZER_OFF
        msg.stabilize = {
            "visca_command": 
            {
                "command_id": command_id,
                "data": 
                {
                    "param1": 0,
                    "param2": 0,
                    "param3": 0
                }
            }
        }
        client_socket.send(pickle.dumps(msg.stabilize))

    def defog_pushed(self, *args):
        global client_socket
        if self.defog_button.isChecked():
            self.curr_defog = True
            command_id = ViscaPacketsEnum.COMMAND_CAM_DEFOG_ON
        else:
            self.curr_defog = False
            command_id = ViscaPacketsEnum.COMMAND_CAM_DEFOG_OFF
        msg.defog = {
            "visca_command": 
            {
                "command_id": command_id,
                "data": 
                {
                    "param1": 0,
                    "param2": 0,
                    "param3": 0
                }
            }
        }        
        client_socket.send(pickle.dumps(msg.defog))

    def backlight_pushed(self, *args):
        global client_socket
        if self.backlight_button.isChecked():
            self.curr_backlight = True
            command_id = ViscaPacketsEnum.COMMAND_CAM_BACKLIGHT_ON
        else:
            self.curr_backlight = False
            command_id = ViscaPacketsEnum.COMMAND_CAM_BACKLIGHT_OFF
        msg.defog = {
            "visca_command": 
            {
                "command_id": command_id,
                "data": 
                {
                    "param1": 0,
                    "param2": 0,
                    "param3": 0
                }
            }
        }  
        client_socket.send(pickle.dumps(msg.backlight))

    def sensitivity_pushed(self, *args):
        global client_socket
        if self.sensitivity_button.isChecked():
            self.curr_sensitivity = True
            command_id = ViscaPacketsEnum.COMMAND_CAM_HIGHSENSITIVITY_ON
        else:
            self.curr_sensitivity = False
            command_id = ViscaPacketsEnum.COMMAND_CAM_HIGHSENSITIVITY_OFF
        msg.sensitivity = {
            "visca_command": 
            {
                "command_id": command_id,
                "data": 
                {
                    "param1": 0,
                    "param2": 0,
                    "param3": 0
                }
            }
        }  
        client_socket.send(pickle.dumps(msg.sensitivity))
    
    def white_balance_msg(self, command_id):
        msg.white_blance['visca_command']['command_id'] = command_id
        msg.white_blance['visca_command']['data']['param1'] = 0
        msg.white_blance['visca_command']['data']['param2'] = 0
        msg.white_blance['visca_command']['data']['param3'] = 0
        return msg.white_blance

    def wb_auto_pushed(self, *args):
        global client_socket
        self.red_gain_val = 0
        self.blue_gain_val = 0
        self.red_gain_bar.setValue(self.red_gain_val)
        self.blue_gain_bar.setValue(self.blue_gain_val)
        self.red_gain_bar.setEnabled(False)
        self.blue_gain_bar.setEnabled(False)
        self.curr_wb_mode = "Auto"
        self.wb_mode.setText(self.curr_wb_mode)
        client_socket.send(pickle.dumps(self.white_balance_msg(ViscaPacketsEnum.COMMAND_CAM_WB_AUTO)))

    def wb_manual_pushed(self, *args):
        global client_socket
        self.red_gain_bar.setEnabled(True)
        self.blue_gain_bar.setEnabled(True)
        self.curr_wb_mode = "Manual"
        self.wb_mode.setText(self.curr_wb_mode)
        client_socket.send(pickle.dumps(self.white_balance_msg(ViscaPacketsEnum.COMMAND_CAM_WB_MANUAL)))

    def indoor_pushed(self, *args):
        global client_socket
        self.red_gain_val = 0
        self.blue_gain_val = 0
        self.red_gain_bar.setValue(self.red_gain_val)
        self.blue_gain_bar.setValue(self.blue_gain_val)
        self.red_gain_bar.setEnabled(False)
        self.blue_gain_bar.setEnabled(False)
        self.curr_wb_mode = "Indoor"
        self.wb_mode.setText(self.curr_wb_mode)
        client_socket.send(pickle.dumps(self.white_balance_msg(ViscaPacketsEnum.COMMAND_CAM_WB_INDOOR)))
    
    def outdoor_pushed(self, *args):
        global client_socket
        self.red_gain_val = 0
        self.blue_gain_val = 0
        self.red_gain_bar.setValue(self.red_gain_val)
        self.blue_gain_bar.setValue(self.blue_gain_val)
        self.red_gain_bar.setEnabled(False)
        self.blue_gain_bar.setEnabled(False)        
        self.curr_wb_mode = "Outdoor"
        self.wb_mode.setText(self.curr_wb_mode)
        client_socket.send(pickle.dumps(self.white_balance_msg(ViscaPacketsEnum.COMMAND_CAM_WB_OUTDOOR)))

    def outdoorAuto_pushed(self, *args):
        global client_socket
        self.red_gain_val = 0
        self.blue_gain_val = 0
        self.red_gain_bar.setValue(self.red_gain_val)
        self.blue_gain_bar.setValue(self.blue_gain_val)
        self.red_gain_bar.setEnabled(False)
        self.blue_gain_bar.setEnabled(False)        
        self.curr_wb_mode = "Outdoor Auto"
        self.wb_mode.setText(self.curr_wb_mode)
        client_socket.send(pickle.dumps(self.white_balance_msg(ViscaPacketsEnum.COMMAND_CAM_WB_OUTDOOR_AUTO)))

    def onePush_pushed(self, *args):
        global client_socket
        self.red_gain_bar.setEnabled(False)
        self.blue_gain_bar.setEnabled(False)        
        self.wb_mode.setText(self.curr_wb_mode)
        client_socket.send(pickle.dumps(self.white_balance_msg(ViscaPacketsEnum.COMMAND_CAM_WB_ONE_PUSH_TRIGGER)))

    def autoTracing_pushed(self, *args):
        global client_socket
        self.red_gain_val = 0
        self.blue_gain_val = 0
        self.red_gain_bar.setValue(self.red_gain_val)
        self.blue_gain_bar.setValue(self.blue_gain_val)
        self.red_gain_bar.setEnabled(False)
        self.blue_gain_bar.setEnabled(False)        
        self.curr_wb_mode = "Auto Tracing"
        self.wb_mode.setText(self.curr_wb_mode)
        client_socket.send(pickle.dumps(self.white_balance_msg(ViscaPacketsEnum.COMMAND_CAM_WB_ATW)))

    def sodiumLamp_pushed(self, *args):
        global client_socket
        self.red_gain_val = 0
        self.blue_gain_val = 0
        self.red_gain_bar.setValue(self.red_gain_val)
        self.blue_gain_bar.setValue(self.blue_gain_val)
        self.red_gain_bar.setEnabled(False)
        self.blue_gain_bar.setEnabled(False)        
        self.curr_wb_mode = "Sodium Lamp"
        self.wb_mode.setText(self.curr_wb_mode)
        client_socket.send(pickle.dumps(self.white_balance_msg(ViscaPacketsEnum.COMMAND_CAM_WB_SODIUM_LAMP)))

    def monitor_modes_msg(self, param2):
        msg.monitor['visca_command']['command_id'] = ViscaPacketsEnum.COMMAND_CAM_REGISTERVALUE
        msg.monitor['visca_command']['data']['param1'] = 0
        msg.monitor['visca_command']['data']['param2'] = param2
        msg.monitor['visca_command']['data']['param3'] = 0
        return msg.monitor

    def monitor_2160p2997_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "2160p/29.97"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(29)))

    def monitor_2160p25_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "2160p/25"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(30)))

    def monitor_2160p2398_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "2160p/23.98"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(32)))

    def monitor_1080p5994_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "2108p/59.94"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(19)))

    def monitor_1080p50_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "1080p/50"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(20)))

    def monitor_1080p2997_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "1080p/29.97"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(6)))

    def monitor_1080p25_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "1080p/25"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(8)))

    def monitor_1080p23_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "1080p/23"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(31)))

    def monitor_1080i5994_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "1080i/59.94"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(1)))

    def monitor_1080i50_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "1080i/50"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(4)))

    def monitor_720p5994_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "720p/59.94"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(9)))

    def monitor_720p50_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "720p/50"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(12)))

    def monitor_480p5994_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "480p/59.94"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(27)))

    def monitor_576p50_pushed(self, *args):
        global client_socket
        self.curr_monitor_mode = "576p/50"
        self.monitor_mode.setText(self.curr_monitor_mode)
        client_socket.send(pickle.dumps(self.monitor_modes_msg(28)))

    def sensitivity_pushed(self, *args):
        global client_socket
 
        if self.sensitivity_button == True:
            command_id = ViscaPacketsEnum.COMMAND_CAM_HIGHSENSITIVITY_ON
        else:
            command_id = ViscaPacketsEnum.COMMAND_CAM_HIGHSENSITIVITY_OFF
        msg.sensitivity = {
            "visca_command": 
            {
                "command_id": command_id,
                "data": 
                {
                    "param1": 0,
                    "param2": 0,
                    "param3": 0
                }
            }
        }  
        client_socket.send(pickle.dumps(msg.sensitivity))

    #============= GIMBAL =============
    def lock_pushed(self, *args):
        global client_socket
        if self.lock_button.isChecked():
            self.syncgimbal_button.setEnabled(False)
            self.angle_button.setEnabled(False)
            self.touch_button.setEnabled(False)
            self.home_button.setEnabled(False)
            self.left_button.setEnabled(False)
            self.right_button.setEnabled(False)
            self.up_button.setEnabled(False)
            self.down_button.setEnabled(False)
            self.lock_gimbal = True
        else:
            self.syncgimbal_button.setEnabled(True)
            self.angle_button.setEnabled(True)
            self.touch_button.setEnabled(True)
            self.home_button.setEnabled(True)
            self.left_button.setEnabled(True)
            self.right_button.setEnabled(True)
            self.up_button.setEnabled(True)
            self.down_button.setEnabled(True)
            self.lock_gimbal = False
        msg.lock_gimbal['lock']['data'] = self.lock_gimbal
        client_socket.send(pickle.dumps(msg.lock_gimbal))

    def syncgimbal_pushed(self, *args):
        global client_socket
        pass

    def send_angle_pushed(self, *args):
        global client_socket
        msg.angle_gimbal['angle']['data']['tilt'] = self.tilt_box.value()
        msg.angle_gimbal['angle']['data']['pan'] = self.pan_box.value()
        msg.angle_gimbal['angle']['data']['roll'] = self.roll_box.value()
        client_socket.send(pickle.dumps(msg.angle_gimbal))

    def gimbal_angle_pushed(self, *args):
        if self.angle_button.isChecked():
            self.gimbal_angle_setting = QtWidgets.QFrame(self)
            self.gimbal_angle_setting.setGeometry(QtCore.QRect(440, 240, 331, 151))
            self.gimbal_angle_setting.setFrameShape(QtWidgets.QFrame.StyledPanel)
            self.gimbal_angle_setting.setFrameShadow(QtWidgets.QFrame.Raised)
            self.gimbal_angle_setting.setObjectName("gimbal_angle_setting")
            self.gimbal_angle_setting.setStyleSheet("background-color: rgba(211, 215, 207,0.85);")
            self.tilt_box = QtWidgets.QSpinBox(self.gimbal_angle_setting)
            self.tilt_box.setGeometry(QtCore.QRect(20, 40, 81, 41))
            font = QtGui.QFont()
            font.setPointSize(20)
            self.tilt_box.setFont(font)
            self.tilt_box.setObjectName("tilt_box")
            self.tilt_box.setMinimum(-45)
            self.tilt_box.setMaximum(135)
            self.pan_box = QtWidgets.QSpinBox(self.gimbal_angle_setting)
            self.pan_box.setGeometry(QtCore.QRect(130, 40, 81, 41))
            font = QtGui.QFont()
            font.setPointSize(20)
            self.pan_box.setFont(font)
            self.pan_box.setObjectName("pan_box")
            self.pan_box.setMinimum(-330)
            self.pan_box.setMaximum(330)
            self.roll_box = QtWidgets.QSpinBox(self.gimbal_angle_setting)
            self.roll_box.setGeometry(QtCore.QRect(230, 40, 81, 41))
            font = QtGui.QFont()
            font.setPointSize(20)
            self.roll_box.setFont(font)
            self.roll_box.setObjectName("roll_box")
            self.roll_box.setMinimum(-45)
            self.roll_box.setMaximum(45)            
            self.label_8 = QtWidgets.QLabel(self.gimbal_angle_setting)
            self.label_8.setGeometry(QtCore.QRect(20, 10, 72, 19))
            font = QtGui.QFont()
            font.setPointSize(15)
            font.setBold(True)
            font.setWeight(75)
            self.label_8.setFont(font)
            self.label_8.setAlignment(QtCore.Qt.AlignCenter)
            self.label_8.setObjectName("label_8")
            self.label_9 = QtWidgets.QLabel(self.gimbal_angle_setting)
            self.label_9.setGeometry(QtCore.QRect(130, 10, 72, 19))
            font = QtGui.QFont()
            font.setPointSize(15)
            font.setBold(True)
            font.setWeight(75)
            self.label_9.setFont(font)
            self.label_9.setAlignment(QtCore.Qt.AlignCenter)
            self.label_9.setObjectName("label_9")
            self.label_10 = QtWidgets.QLabel(self.gimbal_angle_setting)
            self.label_10.setGeometry(QtCore.QRect(230, 10, 72, 19))
            font = QtGui.QFont()
            font.setPointSize(15)
            font.setBold(True)
            font.setWeight(75)
            self.label_10.setFont(font)
            self.label_10.setAlignment(QtCore.Qt.AlignCenter)
            self.label_10.setObjectName("label_10")
            self.start_button = QtWidgets.QPushButton(self.gimbal_angle_setting)
            self.start_button.setGeometry(QtCore.QRect(130, 100, 81, 31))
            font = QtGui.QFont()
            font.setPointSize(16)
            font.setBold(False)
            font.setWeight(50)
            self.start_button.setFont(font)
            self.start_button.setObjectName("start_button")

            self.label_8.setText("TILT")
            self.label_8.setStyleSheet("background-color: transparent;")
            self.label_9.setText("PAN")
            self.label_9.setStyleSheet("background-color: transparent;")
            self.label_10.setText("ROLL")
            self.label_10.setStyleSheet("background-color: transparent;")
            self.start_button.setText("Start")

            self.start_button.clicked.connect(self.send_angle_pushed)

            self.gimbal_angle_setting.show()
        else:
            self.gimbal_angle_setting.deleteLater()

    def home_pushed(self, *args):
        global client_socket
        msg.home['home']['data'] = True
        client_socket.send(pickle.dumps(msg.home))

    def up_pushed(self, *args):
        global client_socket
        msg.manual_gimbal['manual']['data'] = "up"
        client_socket.send(pickle.dumps(msg.manual_gimbal))

    def down_pushed(self, *args):
        global client_socket
        msg.manual_gimbal['manual']['data'] = "down"
        client_socket.send(pickle.dumps(msg.manual_gimbal))

    def left_pushed(self, *args):
        global client_socket
        msg.manual_gimbal['manual']['data'] = "left"
        client_socket.send(pickle.dumps(msg.manual_gimbal))

    def right_pushed(self, *args):
        global client_socket
        msg.manual_gimbal['manual']['data'] = "right"
        client_socket.send(pickle.dumps(msg.manual_gimbal))

    def touch_pushed(self, *args):
        if self.touch_button.isChecked():
            self.SOT_button.setEnabled(False)
        else:
            self.SOT_button.setEnabled(True)

    #####------------------------------------------------------#####
    #####                   TRACKBAR callback                  #####
    #####------------------------------------------------------#####

    def zoom_valchange(self):
        global client_socket
        ## Get current position of trackbar
        msg.zoom['visca_command']['command_id'] = ViscaPacketsEnum.COMMAND_CAM_ZOOM_DIRECT
        msg.zoom['visca_command']['data']['param1'] = self.zoom_bar.value()
        msg.zoom['visca_command']['data']['param2'] = 0
        msg.zoom['visca_command']['data']['param3'] = 0
        client_socket.send(pickle.dumps(msg.zoom))

    def gain_valchange(self):
        global client_socket
        self.gain_val = self.gain_bar.value()
        msg.gain['visca_command']['command_id'] = ViscaPacketsEnum.COMMAND_CAM_GAIN_DIRECT
        msg.gain['visca_command']['data']['param1'] = self.gain_val
        msg.gain['visca_command']['data']['param2'] = 0
        msg.gain['visca_command']['data']['param3'] = 0
        client_socket.send(pickle.dumps(msg.gain))

    def shutter_valchange(self):
        global client_socket
        self.shutter_val = self.shutter_bar.value()
        msg.shutter['visca_command']['command_id'] = ViscaPacketsEnum.COMMAND_CAM_SHUTTER_DIRECT
        msg.shutter['visca_command']['data']['param1'] = self.shutter_val
        msg.shutter['visca_command']['data']['param2'] = 0
        msg.shutter['visca_command']['data']['param3'] = 0
        client_socket.send(pickle.dumps(msg.shutter))

    def iris_valchange(self):
        global client_socket
        self.iris_val = self.iris_bar.value()
        msg.iris['visca_command']['command_id'] = ViscaPacketsEnum.COMMAND_CAM_IRIS_DIRECT
        msg.iris['visca_command']['data']['param1'] = self.iris_val
        msg.iris['visca_command']['data']['param2'] = 0
        msg.iris['visca_command']['data']['param3'] = 0
        client_socket.send(pickle.dumps(msg.iris))

    def focus_manual_valchange(self):
        global client_socket
        self.focus_manual_val = self.focus_manual_bar.value()
        msg.focus_manual['visca_command']['command_id'] = ViscaPacketsEnum.COMMAND_CAM_FOCUS_DIRECT
        msg.focus_manual['visca_command']['data']['param1'] = self.focus_manual_val
        msg.focus_manual['visca_command']['data']['param2'] = 0
        msg.focus_manual['visca_command']['data']['param3'] = 0
        client_socket.send(pickle.dumps(msg.focus_manual))

    def red_gain_valchange(self):
        global client_socket
        self.red_gain_val = self.red_gain_bar.value()
        msg.red_gain['visca_command']['command_id'] = ViscaPacketsEnum.COMMAND_CAM_RGAIN_DIRECT
        msg.red_gain['visca_command']['data']['param1'] = self.red_gain_val
        msg.red_gain['visca_command']['data']['param2'] = 0
        msg.red_gain['visca_command']['data']['param3'] = 0
        client_socket.send(pickle.dumps(msg.red_gain))

    def blue_gain_valchange(self):
        global client_socket
        self.blue_gain_val = self.blue_gain_bar.value()
        msg.blue_gain['visca_command']['command_id'] = ViscaPacketsEnum.COMMAND_CAM_RGAIN_DIRECT
        msg.blue_gain['visca_command']['data']['param1'] = self.blue_gain_val
        msg.blue_gain['visca_command']['data']['param2'] = 0
        msg.blue_gain['visca_command']['data']['param3'] = 0
        client_socket.send(pickle.dumps(msg.blue_gain))

       
#####------------------------------------------------------#####
#####                    UTIL FUNCTIONS                    #####
#####------------------------------------------------------#####

def gst_pipeline(stream):
    return(f'rtspsrc location={stream} latency=0 \
            ! rtph265depay  \
            ! avdec_h265  \
            ! videoconvert  \
            ! appsink drop=1 sync=0')

def get_closest_bbox(mouse_coord, infer_bboxes):
    ''''
    Find closest bounding box with double-clicked coordinate
    '''
    print(mouse_coord)
    distances = [(mouse_coord[0]-(infer_bboxes[i]['x'] + infer_bboxes[i]['w']/2))**2 + \
                (mouse_coord[1]-(infer_bboxes[i]['y'] + infer_bboxes[i]['h']/2))**2 \
                for i in range(len(infer_bboxes))]

    for i in range(len(infer_bboxes)):
        center_x = infer_bboxes[i]['x'] + infer_bboxes[i]['w']/2
        center_y = infer_bboxes[i]['y'] + infer_bboxes[i]['h']/2
        print('xc yc w h: ', center_x, center_y, 
                            infer_bboxes[i]['w'], infer_bboxes[i]['h'])

    bbox_id = distances.index(min(distances))
    center_x_result = infer_bboxes[bbox_id]['x'] + infer_bboxes[bbox_id]['w']/2
    center_y_result = infer_bboxes[bbox_id]['y'] + infer_bboxes[bbox_id]['h']/2
    print(bbox_id)

    return [int(center_x_result - infer_bboxes[bbox_id]['w']/2),
            int(center_y_result - infer_bboxes[bbox_id]['h']/2),
            int(center_x_result + infer_bboxes[bbox_id]['w']/2),
            int(center_y_result + infer_bboxes[bbox_id]['h']/2)]
        
def draw_SOT_outbox(img, point1, point2, point3, point4):

    x1, y1 = point1
    x2, y2 = point2
    x3, y3 = point3
    x4, y4 = point4    

    h_img, w_img, _ = img.shape

    x_c = int((x4 - x2)/2 + x2)
    y_c = int((y2 - y1)/2 + y1)

    width_length = int((x4 - x2)/5)
    height_length = int((y2 - y1)/5)

    # Draw target
    cv2.line(img, (x_c,0), (x_c,h_img), (0,0,255), 1)
    cv2.line(img, (0,y_c), (w_img,y_c), (0,0,255), 1)

    # Draw rectangle with 4 corners
    cv2.line(img, (x1, y1), (x1 , y1 + height_length), (0, 255, 0), 3)  #-- top-left
    cv2.line(img, (x1, y1), (x1 + width_length , y1), (0, 255, 0), 3)

    cv2.line(img, (x2, y2), (x2 , y2 - height_length), (0, 255, 0), 3)  #-- bottom-left
    cv2.line(img, (x2, y2), (x2 + width_length , y2), (0, 255, 0), 3)

    cv2.line(img, (x3, y3), (x3 - width_length, y3), (0, 255, 0), 3)  #-- top-right
    cv2.line(img, (x3, y3), (x3, y3 + height_length), (0, 255, 0), 3)

    cv2.line(img, (x4, y4), (x4 , y4 - height_length), (0, 255, 0), 3)  #-- bottom-right
    cv2.line(img, (x4, y4), (x4 - width_length , y4), (0, 255, 0), 3)

    return img

#####------------------------------------------------------#####
#####                    SOCKET CONNECTION                 #####
#####------------------------------------------------------#####

def socket_gcs():
    global client_socket, detect_list, drawing_detectbox, drawing_SOT_outbox
    global rgb_x, rgb_y, rgb_w, rgb_h, ther_x, ther_y, ther_w, ther_h, conf

    while True:
        try:
            # create socket
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((HOST_IP, PORT))

            socket_time = time.time()
            seqID = 0
        except:
            continue

        while True:
            try:
                # send heartbeat
                if int(time.time() - socket_time) >= 1:
                    heartbeat = {
                        'heartbeat':
                        {
                            'data':
                            {
                                'sequenceID': seqID,
                                'device': 'GCS'
                            }
                        }
                    }
                    client_socket.send(pickle.dumps(heartbeat))
                    seqID = seqID + 1 if (seqID < 100) else 0
                    socket_time = time.time()
                    # print(round(time.time() - socket_time, 2), heartbeat)

                # Receive data
                recv_data = client_socket.recv(4*1024)
                data = pickle.loads(recv_data)

                # print(data)

                if 'infer_bbox' in data.keys():
                    detect_list = data['infer_bbox']['data']
                    drawing_detectbox = True
                
                if 'SOT_bbox' in data.keys():
                    rgb_x = data['SOT_bbox']['rgb']['data']['x']
                    rgb_y = data['SOT_bbox']['rgb']['data']['y']
                    rgb_w = data['SOT_bbox']['rgb']['data']['w']
                    rgb_h = data['SOT_bbox']['rgb']['data']['h']
                 
                    ther_x = data['SOT_bbox']['thermal']['data']['x']
                    ther_y = data['SOT_bbox']['thermal']['data']['y']
                    ther_w = data['SOT_bbox']['thermal']['data']['w']
                    ther_h = data['SOT_bbox']['thermal']['data']['h']

                    conf = data['SOT_bbox']['conf']
                    drawing_SOT_outbox = True

            except Exception as e:
                print(e)
                break

if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    socket_thread = Thread(target=socket_gcs, args=[], daemon=True)
    socket_thread.start()

    sys.exit(app.exec_())

