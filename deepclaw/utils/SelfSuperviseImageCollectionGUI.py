# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/1/12
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""

import PySide2
from PySide2 import QtCore, QtGui, QtWidgets
from PySide2.QtCore import Qt, Slot
from PySide2.QtGui import QPainter
# from PySide2.QtWidgets import (QAction, QApplication, QHeaderView, QHBoxLayout, QLabel, QLineEdit,
#                                QMainWindow, QPushButton, QTableWidget, QTableWidgetItem,
#                                QVBoxLayout, QWidget)
from PySide2.QtCharts import QtCharts

import sys
import os
# append the DeepClawDev directory to python path and set it as working directory
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)
os.chdir(_root_path)
# print('work_dir: ', _root_path)

import cv2
import threading
from deepclaw.utils.SelfSuperviseImageCollection import ImageCollection
from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, widget):
        QtWidgets.QMainWindow.__init__(self)
        self.setWindowTitle("Waste Data Collector")

        # Menu
        self.menu = self.menuBar()
        self.file_menu = self.menu.addMenu("File")

        # Exit QAction
        exit_action = QtWidgets.QAction("Exit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.exit_app)

        self.file_menu.addAction(exit_action)
        self.setCentralWidget(widget)

    @Slot()
    def exit_app(self, checked):
        QtWidgets.QApplication.quit()


class Widget(QtWidgets.QWidget):
    def __init__(self):
        QtWidgets.QWidget.__init__(self)

        self.items = 0
        """left"""
        # saved path box layout
        self.btn_dialog = QtWidgets.QPushButton('Open')
        self.saved_line_edit = QtWidgets.QLineEdit()
        self.saved_line_edit.setText("Saved Path")

        self.saved_path_box = QtWidgets.QHBoxLayout()
        self.saved_path_box.addWidget(self.saved_line_edit)
        self.saved_path_box.addWidget(self.btn_dialog)
        # choose camera to show
        # self.flash_button = QtWidgets.QPushButton("Flash")
        self.camera_select = QtWidgets.QComboBox()
        self.camera_select_text = QtWidgets.QLabel()
        self.camera_select_text.setText("Showed Camera")

        self.camera_select_box = QtWidgets.QHBoxLayout()
        self.camera_select_box.addWidget(self.camera_select_text)
        self.camera_select_box.addWidget(self.camera_select)
        # self.camera_select_box.addWidget(self.flash_button)
        self.camera_select_box.setStretchFactor(self.camera_select_text, 1)
        self.camera_select_box.setStretchFactor(self.camera_select, 3)
        # self.camera_select_box.setStretchFactor(self.flash_button, 1)
        """camera list, table"""
        self.add_camera_button = QtWidgets.QPushButton("Add Camera")
        self.table = QtWidgets.QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Camera", "Cfg file"])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)

        self.camera_list_box = QtWidgets.QVBoxLayout()
        self.camera_list_box.addWidget(self.add_camera_button)
        self.camera_list_box.addWidget(self.table)

        self.camera_list_box.addLayout(self.camera_select_box)
        self.camera_list_box.addLayout(self.saved_path_box)

        """right"""
        self.button = QtWidgets.QPushButton('Show pictures')
        self.button_start = QtWidgets.QPushButton('Start')
        button_top = QtWidgets.QHBoxLayout()
        button_top.addWidget(self.button_start)
        # button_top.addWidget(self.button)

        # image layout
        self.img_left = QtWidgets.QLabel()
        self.img_left.setScaledContents(True)
        self.img_left.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.img_right = QtWidgets.QLabel()
        self.img_right.setScaledContents(True)
        self.img_right.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        # image label
        image_label_left = QtWidgets.QLineEdit()
        image_label_left.setText('Origin')
        image_label_left.setEnabled(False)
        image_label_right = QtWidgets.QLineEdit()
        image_label_right.setText('Mask')
        image_label_right.setEnabled(False)
        # image and label combines
        self.img_box_left = QtWidgets.QVBoxLayout()
        self.img_box_left.addWidget(self.img_left)
        self.img_box_left.addWidget(image_label_left)
        self.img_box_right = QtWidgets.QVBoxLayout()
        self.img_box_right.addWidget(self.img_right)
        self.img_box_right.addWidget(image_label_right)
        # combine 2 image box layout
        self.mid_view = QtWidgets.QHBoxLayout()
        self.mid_view.setMargin(10)
        self.mid_view.addLayout(self.img_box_left)
        self.mid_view.addLayout(self.img_box_right)

        """right show area"""
        self.show_right = QtWidgets.QVBoxLayout()
        # self.show_right.addWidget(self.button_start)
        # self.show_right.addWidget(self.button)
        self.show_right.addLayout(button_top)
        self.show_right.addLayout(self.mid_view)

        """main layout"""
        self.layout = QtWidgets.QHBoxLayout()
        self.layout.addLayout(self.camera_list_box)
        self.layout.addLayout(self.show_right)
        # set scalar of each layout
        self.layout.setStretchFactor(self.camera_list_box, 1)
        self.layout.setStretchFactor(self.show_right, 2)

        # Set the layout to the QWidget
        self.setLayout(self.layout)

        """Signals and Slots"""
        # self.button.clicked.connect(self.showImg)
        self.btn_dialog.clicked.connect(self.openFolderDialog)
        self.add_camera_button.clicked.connect(self.add_element)
        # self.flash_button.clicked.connect(self.flash_camera_list)
        # self.camera_select.currentIndexChanged[str].connect(self.test)  # 条目发生改变，发射信号，传递条目内容
        self.button_start.clicked.connect(self.start_collection)

    @Slot()
    def test(self, i):
        # print('xx', i)
        camera_index = self.table.findItems(i, Qt.MatchFlag.MatchExactly)
        camera_row = self.table.indexFromItem(camera_index[0]).row()
        # print(camera_row)

    @Slot()
    def flash_camera_list(self):
        """ flash camera list """
        self.camera_select.clear()
        for i in range(self.items):
            self.camera_select.addItem(self.table.item(i, 0).text())

    @Slot()
    def start_collection(self):
        """ click start button and collect images"""
        print('\033[1;35m', 'Start Collection!', '\033[0m')
        camera_index = self.camera_select.currentIndex()
        show_img = True
        if camera_index == -1:
            show_img = False
            print('\033[1;31m', 'Select a camera!', '\033[0m')

        saved_dir = self.saved_line_edit.text()
        if not os.path.isdir(saved_dir):
            show_img = False
            print('\033[1;31m', 'Select saved directory!', '\033[0m')

        if show_img:
            # set widgets unable
            self.button_start.setEnabled(False)
            self.btn_dialog.setEnabled(False)
            self.saved_line_edit.setEnabled(False)
            self.add_camera_button.setEnabled(False)
            self.camera_select.setEnabled(False)

            # Instantiate cameras class
            camera_driver_list = []
            for i in range(self.items):
                # self.camera_select.addItem(self.table.item(i, 0).text())
                print(self.table.item(i, 1).text())
                camera_cfg = self.table.item(i, 1).text()
                temp = Realsense(camera_cfg)
                camera_driver_list.append(temp)

            self.test = ImageCollection(camera_driver_list)
            self.test.setObjectClass("paper")
            # collection thread
            collect_thread = threading.Thread(target=self.test.run,
                                              kwargs={"saved_folder_path": saved_dir, "auto_label": True,
                                                      "ref_img": [None, None], "camera_showed_id": camera_index},
                                              daemon=True)
            collect_thread.start()

            # show thread
            show_thread = threading.Thread(target=self.showImg, daemon=True)
            show_thread.start()

    @Slot()
    def showImg(self):
        """ show images in the right area"""
        # xx = self.camera_select.currentText()
        # yy = self.camera_select.currentData()
        # print(xx, yy)
        camera_index = self.camera_select.currentIndex()
        show_img = True
        if camera_index == -1:
            show_img = False
            print('\033[1;31m', 'Select a camera!', '\033[0m')

        saved_dir = self.saved_line_edit.text()
        if not os.path.isdir(saved_dir):
            show_img = False
            print('\033[1;31m', 'Select saved directory!', '\033[0m')

        while True and show_img:
            # if self.test.showed_clc is None or self.test.showed_clc_label is None:
            if self.test.showed_clc_label.empty() or self.test.showed_clc.empty():
                continue
            else:
                left_img = self.test.showed_clc.get()
                right_img = self.test.showed_clc_label.get()
                # transfer np.array to QImage
                left_img = QtGui.QImage(left_img.data, left_img.shape[1], left_img.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
                right_img = QtGui.QImage(right_img.data, right_img.shape[1], right_img.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()

                self.img_left.setPixmap(QtGui.QPixmap.fromImage(left_img))
                self.img_right.setPixmap(QtGui.QPixmap.fromImage(right_img))

    @Slot()
    def openFolderDialog(self):
        """ select a folder, and display its path in a text_edit"""
        dialog = QtWidgets.QFileDialog()
        fileNames = dialog.getExistingDirectory(self, "Select a directory")
        self.saved_line_edit.setText(fileNames)

    @Slot()
    def add_element(self):
        """ add table items"""
        dialog = QtWidgets.QFileDialog()
        # set file filter
        # dialog.setFileMode(QtWidgets.QFileDialog.AnyFile)
        dialog.setNameFilters(["Text files (*.txt)", "Images (*.png *.jpg)", "Cfg (*.yaml *.xml *.json)", "All files (*.*)"])
        dialog.selectNameFilter("Cfg (*.yaml *.xml *.json)")
        # set file view model
        dialog.setViewMode(QtWidgets.QFileDialog.Detail)
        if dialog.exec_():
            fileNames = dialog.selectedFiles()
            # temp_name = fileNames[0].split("/")[-1]
            cfg_item = QtWidgets.QTableWidgetItem(fileNames[0])
            camera_item = QtWidgets.QTableWidgetItem("Camera%d" % self.items)

            self.table.insertRow(self.items)
            self.table.setItem(self.items, 0, camera_item)
            self.table.setItem(self.items, 1, cfg_item)

            self.items += 1

        # update showed camera
        self.camera_select.addItem(self.table.item(self.items-1, 0).text())


if __name__ == "__main__":
    # Qt Application
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)    # QWidget
    widget = Widget()
    # QMainWindow using QWidget as central widget
    window = MainWindow(widget)
    window.resize(800, 600)
    window.show()

    # Execute application
    sys.exit(app.exec_())