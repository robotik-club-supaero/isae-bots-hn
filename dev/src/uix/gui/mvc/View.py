# -*- coding: utf-8 -*-

'''
View class of the GUI node
It contains all the display functions (and no logic)
'''
import os
PATH = os.path.dirname(os.path.abspath(__file__))

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QMenuBar, QStatusBar, QShortcut, QGridLayout, QPushButton
from PyQt5.QtCore import Qt, QTimer, QRect, QMetaObject,QRectF, QPoint
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QPixmap, QKeySequence, QBrush

from .components.MatchBoard import MatchBoard

'''
Default coordinate system (can be translated, rotated and scaled)
--------> x
|
|
|
˅
y

'''

SCREEN_MARGIN = 200

REFRESH_FREQ = 10  # Hz

class View(QMainWindow):
	def __init__(self):
		super().__init__()


		self.setObjectName("Main Window")
		#self.resize(800, 600)

		self.mainLayoutWidget = QWidget()

		self.mainLayout = QGridLayout(self.mainLayoutWidget)
		self.mainLayout.setObjectName("Main layout")

		self.matchBoard = MatchBoard()
		self.mainLayout.addWidget(self.matchBoard)

		self.pushButton_2 = QPushButton()
		self.pushButton_2.setObjectName("pushButton_2")
		self.pushButton_2.setText("Bonsoir")


		self.mainLayout.addWidget(self.pushButton_2, 1, 1, 1, 1)


		self.pixmap_image = QPixmap(os.path.join(PATH, 'image_files/vinyle.gif'))


		# key bindings
		self.shortcut_close = QShortcut(QKeySequence('Ctrl+C'), self)
		self.shortcut_close.activated.connect(self.closeApp)



		self.timer = QTimer(self)
		self.timer.timeout.connect(self.refresh)
		self.timer.start(1/REFRESH_FREQ)

		self.isRefreshNeeded = False

		self.setCentralWidget(self.mainLayoutWidget)
		



	def closeApp(self):
		print("Closing main window")
		self.close()
		return


	def refreshNeeded(self):
		self.isRefreshNeeded = True

	def refresh(self):
		if self.refreshNeeded:
			self.update()
			self.isRefreshNeeded = False


	def setOrientation(self, isVertical):
		if isVertical:
			self.setGeometry(0+SCREEN_MARGIN, 0+SCREEN_MARGIN,
							 self.screenWidth-SCREEN_MARGIN, self.screenHeight-SCREEN_MARGIN)

		else: 
			self.setGeometry(100, 100, 900, 600)
			


	def setupScreen(self, dims):
		self.screenWidth = dims.width()
		self.screenHeight = dims.height()
		print("lol", self.screenWidth, self.screenHeight)

		self.setOrientation(True)  # vertical by default


	
	def blinkLED(self, led, col):
		return

	def setLED(self, led, col):
		return



# # don't auto scale when drag app to a different monitor.
# # QApplication.setAttribute(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)

# def main():
# 	app = QApplication(sys.argv)
# 	view = View()
# 	view.show()
# 	app.exec_()

# 	print("End")

# try:
#     sys.exit(app.exec_())
# except SystemExit:
#     print('Closing Window...')