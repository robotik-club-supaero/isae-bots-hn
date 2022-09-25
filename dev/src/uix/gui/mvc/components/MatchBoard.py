# -*- coding: utf-8 -*-

'''
Board
'''
import os
PATH = os.path.dirname(os.path.abspath(__file__))

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QMenuBar, QStatusBar, QShortcut
from PyQt5.QtCore import Qt, QTimer, QRect, QMetaObject,QRectF, QPoint
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QPixmap, QKeySequence, QBrush, QTransform



# CONSTANTS #

X_MARGIN = 5
Y_MARGIN = 5


class MatchBoard(QWidget):
	def __init__(self):
		super().__init__()


		self.bg_image = QPixmap(os.path.join(PATH, '../../image_files/vinyle_2023.png'))
		self.bg_image = self.bg_image.transformed(QTransform().rotate(-90))

		self.bg_dims = (self.bg_image.size().width(), self.bg_image.size().height())

		self.robot_rect = QRect(0, 0, 150, 100)  # TODO : robot dimensions



	def refresh(self):
		'''Ordered by the View object'''

		self.robot_rect.moveTop(self.robot_rect.top() + 1)

		self.update()



	def setupScreen(self, dims):
		self.screenWidth = dims.width()
		self.screenHeight = dims.height()
		print("lol", self.screenWidth, self.screenHeight)

		self.setOrientation(True)  # vertical by default


			
	def opensomething(self):
		print("Open something")
		self.setGeometry(100,100,300,500)

	def closeApp(self):
		print("Closing the interface")




	
	def paintEvent(self, event):
		painter = QPainter()
		painter.begin(self)

		# painter.translate(QPoint(100,10))
		# painter.rotate(10.0)  # can be used to flip the table (vertical or horizontal), it flips the whole coordinate system
		# painter.scale(0.8, 0.8)  # idem with scale, can be ued to zoom in or out

		pen =QPen(QColor(250, 0, 0), 5.0)
		painter.setPen(pen)
		#painter.setFont(QFont('Open Sans', 12))
		# painter.drawText(event.rect(), Qt.AlignCenter, self.text)



		# vertical
		# painter.drawPixmap(QRect(X_MARGIN, Y_MARGIN, 500 - X_MARGIN, 750 - Y_MARGIN),
		# 				   self.bg_image)

		# horizontal
		painter.drawPixmap(QRect(X_MARGIN, Y_MARGIN, 1125 - X_MARGIN, 750 - Y_MARGIN),
						   self.bg_image)

		#painter.drawEllipse(100, 100, 56, 56)

		# painter.drawRect(self.robot_rect)
		painter.fillRect(self.robot_rect, QColor(250, 0, 0))


		# painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))
		# painter.setBrush(QBrush(Qt.green, Qt.SolidPattern))
		# painter.drawRect(self.shape)

		painter.end()



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