from PySide6.QtCore import (QCoreApplication,
    QMetaObject, QRect, Slot, QSize, Qt)
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QApplication, QDialogButtonBox, QDoubleSpinBox,
    QHBoxLayout, QLCDNumber, QLayout,
    QPushButton, QSizePolicy, QSpacerItem, QSpinBox,
    QTextBrowser, QVBoxLayout, QWidget)
import multiprocessing.shared_memory as mpsm
import sys


class Ui_Form(object):
    def setupUi(self, Widget):
        if not Widget.objectName():
            Widget.setObjectName(u"Widget")
        Widget.setWindowModality(Qt.WindowModal)
        Widget.setEnabled(True)
        Widget.resize(700, 200)
        sizePolicy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Widget.sizePolicy().hasHeightForWidth())
        Widget.setSizePolicy(sizePolicy)
        Widget.setMinimumSize(QSize(700, 200))
        Widget.setMaximumSize(QSize(700, 200))
        Widget.setBaseSize(QSize(700, 420))
        self.verticalLayoutWidget_4 = QWidget(Widget)
        self.verticalLayoutWidget_4.setObjectName(u"verticalLayoutWidget_4")
        self.verticalLayoutWidget_4.setGeometry(QRect(20, 10, 661, 171))
        self.verticalLayout_4 = QVBoxLayout(self.verticalLayoutWidget_4)
        self.verticalLayout_4.setSpacing(7)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setSizeConstraint(QLayout.SetDefaultConstraint)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setSizeConstraint(QLayout.SetNoConstraint)
        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer)

        self.lcdNumber = QLCDNumber(self.verticalLayoutWidget_4)
        self.lcdNumber.setObjectName(u"lcdNumber")
        sizePolicy1 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.lcdNumber.sizePolicy().hasHeightForWidth())
        self.lcdNumber.setSizePolicy(sizePolicy1)
        self.lcdNumber.setMinimumSize(QSize(64, 24))
        font = QFont()
        font.setPointSize(12)
        self.lcdNumber.setFont(font)

        self.horizontalLayout_4.addWidget(self.lcdNumber)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_2)


        self.verticalLayout_2.addLayout(self.horizontalLayout_4)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.textBrowser = QTextBrowser(self.verticalLayoutWidget_4)
        self.textBrowser.setObjectName(u"textBrowser")
        sizePolicy2 = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.MinimumExpanding)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.textBrowser.sizePolicy().hasHeightForWidth())
        self.textBrowser.setSizePolicy(sizePolicy2)
        self.textBrowser.setMinimumSize(QSize(140, 34))

        self.horizontalLayout_2.addWidget(self.textBrowser)

        self.spinBox = QSpinBox(self.verticalLayoutWidget_4)
        self.spinBox.setObjectName(u"spinBox")
        sizePolicy3 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.MinimumExpanding)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.spinBox.sizePolicy().hasHeightForWidth())
        self.spinBox.setSizePolicy(sizePolicy3)
        self.spinBox.setMinimumSize(QSize(46, 29))
        font1 = QFont()
        font1.setPointSize(18)
        self.spinBox.setFont(font1)
        self.spinBox.setMinimum(1)
        self.spinBox.setMaximum(100)

        self.horizontalLayout_2.addWidget(self.spinBox)


        self.verticalLayout_2.addLayout(self.horizontalLayout_2)


        self.horizontalLayout_5.addLayout(self.verticalLayout_2)

        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.textBrowser_2 = QTextBrowser(self.verticalLayoutWidget_4)
        self.textBrowser_2.setObjectName(u"textBrowser_2")
        sizePolicy2.setHeightForWidth(self.textBrowser_2.sizePolicy().hasHeightForWidth())
        self.textBrowser_2.setSizePolicy(sizePolicy2)
        self.textBrowser_2.setMinimumSize(QSize(77, 29))

        self.horizontalLayout.addWidget(self.textBrowser_2)

        self.textBrowser_3 = QTextBrowser(self.verticalLayoutWidget_4)
        self.textBrowser_3.setObjectName(u"textBrowser_3")
        sizePolicy2.setHeightForWidth(self.textBrowser_3.sizePolicy().hasHeightForWidth())
        self.textBrowser_3.setSizePolicy(sizePolicy2)
        self.textBrowser_3.setMinimumSize(QSize(77, 29))

        self.horizontalLayout.addWidget(self.textBrowser_3)

        self.textBrowser_4 = QTextBrowser(self.verticalLayoutWidget_4)
        self.textBrowser_4.setObjectName(u"textBrowser_4")
        sizePolicy2.setHeightForWidth(self.textBrowser_4.sizePolicy().hasHeightForWidth())
        self.textBrowser_4.setSizePolicy(sizePolicy2)
        self.textBrowser_4.setMinimumSize(QSize(77, 29))

        self.horizontalLayout.addWidget(self.textBrowser_4)

        self.textBrowser_5 = QTextBrowser(self.verticalLayoutWidget_4)
        self.textBrowser_5.setObjectName(u"textBrowser_5")
        sizePolicy2.setHeightForWidth(self.textBrowser_5.sizePolicy().hasHeightForWidth())
        self.textBrowser_5.setSizePolicy(sizePolicy2)
        self.textBrowser_5.setMinimumSize(QSize(77, 29))

        self.horizontalLayout.addWidget(self.textBrowser_5)


        self.verticalLayout.addLayout(self.horizontalLayout)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.a_doubleSpinBox = QDoubleSpinBox(self.verticalLayoutWidget_4)
        self.a_doubleSpinBox.setObjectName(u"a_doubleSpinBox")
        sizePolicy3.setHeightForWidth(self.a_doubleSpinBox.sizePolicy().hasHeightForWidth())
        self.a_doubleSpinBox.setSizePolicy(sizePolicy3)
        self.a_doubleSpinBox.setMinimumSize(QSize(77, 20))
        self.a_doubleSpinBox.setMinimum(-360.000000000000000)
        self.a_doubleSpinBox.setMaximum(360.000000000000000)

        self.horizontalLayout_3.addWidget(self.a_doubleSpinBox)

        self.b_doubleSpinBox = QDoubleSpinBox(self.verticalLayoutWidget_4)
        self.b_doubleSpinBox.setObjectName(u"b_doubleSpinBox")
        sizePolicy3.setHeightForWidth(self.b_doubleSpinBox.sizePolicy().hasHeightForWidth())
        self.b_doubleSpinBox.setSizePolicy(sizePolicy3)
        self.b_doubleSpinBox.setMinimumSize(QSize(77, 20))
        self.b_doubleSpinBox.setMinimum(-360.000000000000000)
        self.b_doubleSpinBox.setMaximum(360.000000000000000)

        self.horizontalLayout_3.addWidget(self.b_doubleSpinBox)

        self.dx_doubleSpinBox = QDoubleSpinBox(self.verticalLayoutWidget_4)
        self.dx_doubleSpinBox.setObjectName(u"dx_doubleSpinBox")
        sizePolicy3.setHeightForWidth(self.dx_doubleSpinBox.sizePolicy().hasHeightForWidth())
        self.dx_doubleSpinBox.setSizePolicy(sizePolicy3)
        self.dx_doubleSpinBox.setMinimumSize(QSize(77, 20))
        self.dx_doubleSpinBox.setMinimum(-500.000000000000000)
        self.dx_doubleSpinBox.setMaximum(500.000000000000000)

        self.horizontalLayout_3.addWidget(self.dx_doubleSpinBox)

        self.dy_doubleSpinBox = QDoubleSpinBox(self.verticalLayoutWidget_4)
        self.dy_doubleSpinBox.setObjectName(u"dy_doubleSpinBox")
        sizePolicy3.setHeightForWidth(self.dy_doubleSpinBox.sizePolicy().hasHeightForWidth())
        self.dy_doubleSpinBox.setSizePolicy(sizePolicy3)
        self.dy_doubleSpinBox.setMinimumSize(QSize(77, 20))
        self.dy_doubleSpinBox.setMinimum(-500.000000000000000)
        self.dy_doubleSpinBox.setMaximum(500.000000000000000)

        self.horizontalLayout_3.addWidget(self.dy_doubleSpinBox)


        self.verticalLayout.addLayout(self.horizontalLayout_3)


        self.horizontalLayout_5.addLayout(self.verticalLayout)


        self.verticalLayout_4.addLayout(self.horizontalLayout_5)

        self.verticalSpacer = QSpacerItem(20, 4, QSizePolicy.Minimum, QSizePolicy.MinimumExpanding)

        self.verticalLayout_4.addItem(self.verticalSpacer)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.horizontalSpacer_4 = QSpacerItem(40, 20, QSizePolicy.MinimumExpanding, QSizePolicy.Minimum)

        self.horizontalLayout_6.addItem(self.horizontalSpacer_4)

        self.pushButton = QPushButton(self.verticalLayoutWidget_4)
        self.pushButton.setObjectName(u"pushButton")
        sizePolicy4 = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.pushButton.sizePolicy().hasHeightForWidth())
        self.pushButton.setSizePolicy(sizePolicy4)
        self.pushButton.setMinimumSize(QSize(102, 52))
        self.pushButton.setMaximumSize(QSize(102, 52))
        self.pushButton.setFont(font1)

        self.horizontalLayout_6.addWidget(self.pushButton)

        self.horizontalSpacer_3 = QSpacerItem(165, 20, QSizePolicy.MinimumExpanding, QSizePolicy.Minimum)

        self.horizontalLayout_6.addItem(self.horizontalSpacer_3)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.textBrowser_6 = QTextBrowser(self.verticalLayoutWidget_4)
        self.textBrowser_6.setObjectName(u"textBrowser_6")
        sizePolicy2.setHeightForWidth(self.textBrowser_6.sizePolicy().hasHeightForWidth())
        self.textBrowser_6.setSizePolicy(sizePolicy2)
        self.textBrowser_6.setMinimumSize(QSize(150, 34))

        self.verticalLayout_3.addWidget(self.textBrowser_6)

        self.buttonBox = QDialogButtonBox(self.verticalLayoutWidget_4)
        self.buttonBox.setObjectName(u"buttonBox")
        sizePolicy2.setHeightForWidth(self.buttonBox.sizePolicy().hasHeightForWidth())
        self.buttonBox.setSizePolicy(sizePolicy2)
        self.buttonBox.setMinimumSize(QSize(150, 24))
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)

        self.verticalLayout_3.addWidget(self.buttonBox)


        self.horizontalLayout_6.addLayout(self.verticalLayout_3)

        self.horizontalSpacer_5 = QSpacerItem(165, 20, QSizePolicy.MinimumExpanding, QSizePolicy.Minimum)

        self.horizontalLayout_6.addItem(self.horizontalSpacer_5)


        self.verticalLayout_4.addLayout(self.horizontalLayout_6)

        self.verticalSpacer_2 = QSpacerItem(20, 4, QSizePolicy.Minimum, QSizePolicy.MinimumExpanding)

        self.verticalLayout_4.addItem(self.verticalSpacer_2)

        self.verticalLayout_4.setStretch(0, 4)
        self.verticalLayout_4.setStretch(1, 7)

        self.retranslateUi(Widget)
        self.pushButton.clicked.connect(Widget.record)
        self.buttonBox.accepted.connect(Widget.load_pos)

        QMetaObject.connectSlotsByName(Widget)
    # setupUi

    def retranslateUi(self, Widget):
        Widget.setWindowTitle(QCoreApplication.translate("Widget", u"温室作物表型信息采集系统", None))
        self.textBrowser.setHtml(QCoreApplication.translate("Widget", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt;\">\u6d4b\u91cf\u6b21\u6570</span></p></body></html>", None))
        self.textBrowser_2.setHtml(QCoreApplication.translate("Widget", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">\u89d2\u5ea6a</span></p></body></html>", None))
        self.textBrowser_3.setHtml(QCoreApplication.translate("Widget", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">\u89d2\u5ea6b</span></p></body></html>", None))
        self.textBrowser_4.setHtml(QCoreApplication.translate("Widget", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">\u4f4d\u7f6ex</span></p></body></html>", None))
        self.textBrowser_5.setHtml(QCoreApplication.translate("Widget", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">\u4f4d\u7f6ey</span></p></body></html>", None))
        self.pushButton.setText(QCoreApplication.translate("Widget", u"\u5f00\u59cb\u5f55\u5236", None))
        self.textBrowser_6.setHtml(QCoreApplication.translate("Widget", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">\u8f7d\u5165\u4f4d\u7f6e\u4fe1\u606f</span></p></body></html>", None))
    # retranslateUi


class MyWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self) #这里传递的是MyWidget
        self.__count_flag = True
        self.__count = -1
        self.__counts = -2
        # 录制次数, 录制开关, a, b, x, y, 位置确认开关
        self.__info = mpsm.ShareableList([1, False, 0.0, 0.0, 0.0, 0.0, False], name="info")

    @Slot()
    def record(self):
        if self.__counts == self.__count or not self.__info[1]: return
        if self.__count_flag:
            self.__counts = self.ui.spinBox.value()
            self.__info[0] = self.__counts
            self.__count_flag = False
        self.__count += 1
        self.ui.lcdNumber.display(self.__count)
        self.__info[1] = True  

    @Slot()
    def load_pos(self):
        if self.__count != self.__counts:
            self.__info[1], self.__info[2], self.__info[3], self.__info[4] = self.pos 
            self.__info[5] = True

    @property
    def pos(self) -> list:
        return [self.ui.a_doubleSpinBox.value(),
                self.ui.b_doubleSpinBox.value(),
                self.ui.dx_doubleSpinBox.value(),
                self.ui.dy_doubleSpinBox.value()]


if __name__ == "__main__":
    app = QApplication([])
    mywidget = MyWidget()
    mywidget.show()
    sys.exit(app.exec())