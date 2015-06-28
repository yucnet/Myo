# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_plot.ui'
#
# Created: Wed May 08 10:02:53 2013
#      by: PyQt4 UI code generator 4.9.5
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui, Qt

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_win_plot(object):
    def setupUi(self, win_plot):
        win_plot.setObjectName(_fromUtf8("win_plot"))
        win_plot.resize(800, 600)
#        win_plot.setStyleSheet("background-color:white");
        self.centralwidget = QtGui.QWidget(win_plot)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))

        self.qwtPlot = Qwt5.QwtPlot(self.centralwidget)
        self.qwtPlot.setTitle("Right and Left Arm Accelerometer Readings")
        self.qwtPlot.setCanvasBackground(Qt.Qt.white) 
        self.qwtPlot.plotLayout().setAlignCanvasToScales(True)
        
        # Setting up Axis labels
        self.qwtPlot.setAxisTitle(Qwt5.QwtPlot.xBottom, "Time (millis)")
        self.qwtPlot.setAxisTitle(Qwt5.QwtPlot.yLeft, "Acceleration Magnitude")

        # Setting up legends
        legend = Qwt5.QwtLegend()
        #self.insertLegend(legend, Qwt.QwtPlot.RightLegend)
        #self.qwtPlot.insertLegend(legend, Qwt5.QwtPlot.BottomLegend)

        # Setting up plot colors
        grid = Qwt5.QwtPlotGrid()
        pen = Qt.QPen(Qt.Qt.DotLine)
        pen.setColor(Qt.Qt.blue)
        pen.setWidth(1)
        grid.setPen(pen)
        grid.attach(self.qwtPlot)


        self.qwtPlot.setObjectName(_fromUtf8("qwtPlot"))
        self.verticalLayout.addWidget(self.qwtPlot)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setContentsMargins(6, 0, 6, 0)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.verticalLayout.addLayout(self.horizontalLayout)
        
        win_plot.setCentralWidget(self.centralwidget)

        self.retranslateUi(win_plot)
        QtCore.QMetaObject.connectSlotsByName(win_plot)

    def retranslateUi(self, win_plot):
        win_plot.setWindowTitle(QtGui.QApplication.translate("win_plot", "MainWindow", None, QtGui.QApplication.UnicodeUTF8))

from PyQt4 import Qwt5

if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    win_plot = QtGui.QMainWindow()
    ui = Ui_win_plot()
    ui.setupUi(win_plot)
    win_plot.show()
    sys.exit(app.exec_())

