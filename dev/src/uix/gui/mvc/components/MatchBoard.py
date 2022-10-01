# -*- coding: utf-8 -*-

'''
Board
'''

import os
PATH = os.path.dirname(os.path.abspath(__file__))

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QMenuBar, QStatusBar, QShortcut
from PyQt5.QtCore import Qt, QTimer, QRect, QMetaObject,QRectF, QPoint, QLine
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QPixmap, QKeySequence, QBrush, QTransform, QCursor, QPolygonF, QPainterPath

import numpy as np
from numpy import pi, sqrt, cos, sin, arctan


# CONSTANTS #

X_MARGIN = 0
Y_MARGIN = 0

XBORDERLEFT = 0
YBORDER = 0


# TODO : read from config file
ROBOT_WIDTH = 195
ROBOT_HEIGHT = 160

ROBOT_DIAG = np.linalg.norm([ROBOT_WIDTH, ROBOT_HEIGHT])
ROBOT_ANGLE = arctan(ROBOT_HEIGHT / ROBOT_WIDTH)


class MatchBoard(QWidget):

	matchBoardSizeFactor = 0.2
	matchBoardDims = (2000*matchBoardSizeFactor, 3000*matchBoardSizeFactor)  # px

	def __init__(self):
		super().__init__()


		self.bg_image = QPixmap(os.path.join(PATH, '../../image_files/vinyle_2023.png'))
		self.bg_dims = (self.bg_image.size().width(), self.bg_image.size().height())


		self.bgOrientation = None
		self.setVerticalOrientation()  # by default

		# self.robot_rect = QRect(0, 0, 150, 100)  # TODO : robot dimensions
		self.robot_shape = QPolygonF([QPoint(0,0), QPoint(0,0), QPoint(0,0), QPoint(0,0)])
		self.robot_line = QLine(QPoint(0,0), QPoint(0,0))

		# init robot position (start pos)
		# TODO

		self.lastClickedPosition = None
		self.lastReleasedPosition = None
		self.matchBoardClicked = False




	def switchOrientation(self):
		'''Called by button press'''

		if self.bgOrientation == "vertical": self.setHorizontalOrientation()
		elif self.bgOrientation == "horizontal": self.setVerticalOrientation()
		else: print("ERROR : No orientation defined")


	def setVerticalOrientation(self):

		self.matchBoardSizeFactor = 0.2
		self.matchBoardDims = (2000*self.matchBoardSizeFactor, 3000*self.matchBoardSizeFactor)

		self.setFixedSize(self.matchBoardDims[0], self.matchBoardDims[1])
		if self.bgOrientation is not None: self.bg_image = self.bg_image.transformed(QTransform().rotate(-90))

		self.bgOrientation = "vertical"


	def setHorizontalOrientation(self):

		self.matchBoardSizeFactor = 0.375
		self.matchBoardDims = (3000*self.matchBoardSizeFactor, 2000*self.matchBoardSizeFactor)

		self.setFixedSize(self.matchBoardDims[1], self.matchBoardDims[0])
		if self.bgOrientation is not None: self.bg_image = self.bg_image.transformed(QTransform().rotate(90))

		self.bgOrientation = "horizontal"




			
	def mousePressEvent(self, QMouseEvent):

		inputPos = (QMouseEvent.pos().x(), QMouseEvent.pos().y())
		self.lastClickedPosition = self.toBoardPos(inputPos)

		self.matchBoardClicked = True

	def mouseReleaseEvent(self, QMouseEvent):

		inputPos = (QMouseEvent.pos().x(), QMouseEvent.pos().y())
		self.lastReleasedPosition = self.toBoardPos(inputPos)

		self.matchBoardClicked = False

	def isMatchBoardClicked(self):return self.matchBoardClicked

	def getLastClickedPosition(self): return self.lastClickedPosition

	def getLastReleasedPosition(self): return self.lastReleasedPosition


	
	def paintEvent(self, event):
		painter = QPainter()
		painter.begin(self)

		# painter.translate(QPoint(100,10))
		# painter.rotate(10.0)  # can be used to flip the table (vertical or horizontal), it flips the whole coordinate system
		# painter.scale(0.8, 0.8)  # idem with scale, can be ued to zoom in or out


		#painter.setFont(QFont('Open Sans', 12))
		# painter.drawText(event.rect(), Qt.AlignCenter, self.text)



		# vertical
		# painter.drawPixmap(QRect(X_MARGIN, Y_MARGIN, 500 - X_MARGIN, 750 - Y_MARGIN),
		# 				   self.bg_image)

		# horizontal
		painter.drawPixmap(QRect(X_MARGIN, Y_MARGIN, self.matchBoardDims[0] - X_MARGIN, self.matchBoardDims[1] - Y_MARGIN),
						   self.bg_image)

		#painter.drawEllipse(100, 100, 56, 56)

		# painter.drawRect(self.robot_rect)
		# painter.fillRect(self.robot_rect, QColor(250, 0, 0))



		# poly = QPolygonF([QPoint(100,200), QPoint(600,200), QPoint(0,0)])

		# poly.replace(0, QPoint(100,500))

		#### ROBOT ###

		painter.setPen(QPen(Qt.black, 3, Qt.SolidLine))
		brush = QBrush(QColor(200, 20, 20), Qt.SolidPattern)

		path = QPainterPath()
		path.addPolygon(self.robot_shape)

		painter.drawPolygon(self.robot_shape)
		painter.fillPath(path, brush)

		painter.drawLine(self.robot_line)

		painter.end()



	def blinkLED(self, led, col):
		return

	def setLED(self, led, col):
		return


	def toPixelPos(self, pos):
		return (self.matchBoardSizeFactor*(pos[0] + XBORDERLEFT), self.matchBoardDims[1] - self.matchBoardSizeFactor*(pos[1] + YBORDER))

	def toBoardPos(self, pos):
		return (pos[0]/self.matchBoardSizeFactor - XBORDERLEFT, YBORDER - (pos[1] - self.matchBoardDims[1])/self.matchBoardSizeFactor)


	def plotRobot(self, k, pos):

		# TODO : multiple robot rects
		
		# update robot k
		(x, y, theta) = pos

		pointNb = 0
		for i in range (-1, 2, 2):
			for j in range (-1, 2, 2):
				anglePos = (x + ROBOT_DIAG/2*cos(theta - i*pi/2 + j*ROBOT_ANGLE),
							y + ROBOT_DIAG/2*sin(theta - i*pi/2 + j*ROBOT_ANGLE))

				# conversion to pixel points
				anglePos_px = self.toPixelPos(anglePos)

				self.robot_shape.replace(pointNb, QPoint(int(anglePos_px[0]), int(anglePos_px[1])))
				pointNb += 1

		centerPos_px = self.toPixelPos( (x, y) )

		# Coordonnees du milieu du cote avant du robot
		centerLinePos = (x + ROBOT_HEIGHT/2 * cos(theta),
						 y + ROBOT_HEIGHT/2 * sin(theta))
		
		centerLinePos_px = self.toPixelPos(centerLinePos)

		self.robot_line.setPoints(QPoint(int(centerPos_px[0]), int(centerPos_px[1])),
								  QPoint(int(centerLinePos_px[0]), int(centerLinePos_px[1])))



		self.update()




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