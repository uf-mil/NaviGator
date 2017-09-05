# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

import rospy
from std_msgs.msg import Float32

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        rospy.init_node('GUI', anonymous=True)
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(829, 588)
        font = QtGui.QFont()
        font.setFamily("STIX")
        font.setPointSize(6)
        MainWindow.setFont(font)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/MIL.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        MainWindow.setStyleSheet("background: url(MFW.png)")
        MainWindow.setTabShape(QtWidgets.QTabWidget.Triangular)
        self.centralWidget = QtWidgets.QWidget(MainWindow)
        self.centralWidget.setAutoFillBackground(True)
        self.centralWidget.setObjectName("centralWidget")
        self.tabs = QtWidgets.QTabWidget(self.centralWidget)
        self.tabs.setGeometry(QtCore.QRect(0, 0, 841, 591))
        font = QtGui.QFont()
        font.setPointSize(6)
        self.tabs.setFont(font)
        self.tabs.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.tabs.setAutoFillBackground(False)
        self.tabs.setStyleSheet("background-color: rgb(222, 220, 220);")
        self.tabs.setIconSize(QtCore.QSize(20, 20))
        self.tabs.setObjectName("tabs")
        self.tab = QtWidgets.QWidget()
        self.tab.setAutoFillBackground(False)
        self.tab.setObjectName("tab")
        self.formLayoutWidget = QtWidgets.QWidget(self.tab)
        self.formLayoutWidget.setGeometry(QtCore.QRect(0, 10, 421, 401))
        self.formLayoutWidget.setObjectName("formLayoutWidget")
        self.formLayout = QtWidgets.QFormLayout(self.formLayoutWidget)
        self.formLayout.setContentsMargins(11, 11, 11, 11)
        self.formLayout.setSpacing(6)
        self.formLayout.setObjectName("formLayout")
        self.label = QtWidgets.QLabel(self.formLayoutWidget)
        self.label.setFrameShadow(QtWidgets.QFrame.Raised)
        self.label.setTextFormat(QtCore.Qt.RichText)
        self.label.setObjectName("label")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label)
        self.voltage_display = QtWidgets.QLabel(self.formLayoutWidget)
        self.voltage_display.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.voltage_display.setFrameShape(QtWidgets.QFrame.Box)
        self.voltage_display.setObjectName("voltage_display")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.voltage_display)
        self.label_3 = QtWidgets.QLabel(self.formLayoutWidget)
        self.label_3.setFrameShadow(QtWidgets.QFrame.Plain)
        self.label_3.setTextFormat(QtCore.Qt.RichText)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_3)
        self.voltage_display_2 = QtWidgets.QLabel(self.formLayoutWidget)
        self.voltage_display_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.voltage_display_2.setFrameShape(QtWidgets.QFrame.Box)
        self.voltage_display_2.setObjectName("voltage_display_2")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.voltage_display_2)
        self.voltage_display_3 = QtWidgets.QLabel(self.formLayoutWidget)
        self.voltage_display_3.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.voltage_display_3.setFrameShape(QtWidgets.QFrame.Box)
        self.voltage_display_3.setObjectName("voltage_display_3")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.voltage_display_3)
        self.label_4 = QtWidgets.QLabel(self.formLayoutWidget)
        self.label_4.setFrameShadow(QtWidgets.QFrame.Plain)
        self.label_4.setTextFormat(QtCore.Qt.RichText)
        self.label_4.setObjectName("label_4")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_4)
        self.tabs.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.tabs.addTab(self.tab_2, "")
        MainWindow.setCentralWidget(self.centralWidget)

        self.retranslateUi(MainWindow)
        self.tabs.setCurrentIndex(0)

        rospy.Subscriber('battery_monitor', Float32, self.callback)

        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def callback(self,data):
        #rospy.loginfo(data.data)
        #dont forget to add diffrent color text based off of voltage value
        _translate = QtCore.QCoreApplication.translate
        if data.data < 20:
            self.label.setText(_translate("MainWindow", "<b>Battery Voltage<b> "))

        self.voltage_display.setText(str(data.data))

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MIL GUI"))
        self.label.setText(_translate("MainWindow", "Battery Voltage "))
        self.voltage_display.setText(_translate("MainWindow", "N/A"))
        self.label_3.setText(_translate("MainWindow", "Nothing"))
        self.voltage_display_2.setText(_translate("MainWindow", "N/A"))
        self.voltage_display_3.setText(_translate("MainWindow", "N/A"))
        self.label_4.setText(_translate("MainWindow", "Nothing"))
        self.tabs.setTabText(self.tabs.indexOf(self.tab), _translate("MainWindow", "Diagnostics"))
        self.tabs.setTabText(self.tabs.indexOf(self.tab_2), _translate("MainWindow", "Future expansion"))

import images_rc

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

