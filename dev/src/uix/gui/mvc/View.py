
'''
View class of the GUI node
It contains all the display functions (and no logic)
'''

from turtle import width
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QMenuBar, QStatusBar, QShortcut
from PyQt5.QtCore import QRect, QMetaObject,QRectF, QPoint
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QPixmap, QKeySequence, QBrush


'''
Default coordinate system (can be translated, rotated and scaled)
--------> x
|
|
|
˅
y

'''

class View(QWidget):
	def __init__(self):
		super().__init__()
		self.text = 'Hello World'
		self.setGeometry(300, 300, 500, 500)

		PATH = "images/test_1.jpg"  # 640x480
		self.pixmap_image = QPixmap(PATH)


		self.shortcut_open = QShortcut(QKeySequence('Ctrl+O'), self)
		self.shortcut_open.activated.connect(self.on_open)

		self.shortcut_close = QShortcut(QKeySequence('Ctrl+Q'), self)
		self.shortcut_close.activated.connect(self.closeApp)

		self.shortcut_close = QShortcut(QKeySequence('Ctrl+R'), self)
		self.shortcut_close.activated.connect(self.randomEvent)

		self.shape = QRect(int(self.width()/2), 0, 20, 40) # x, y, width, height


			
	def on_open(self):
		print("Open something")
		self.setGeometry(100,100,300,500)

	def closeApp(self):
		print("Closing the interface")

		
	def randomEvent(self):
		self.shape.moveTop(self.shape.top() + 5)
		self.update()


	
	def paintEvent(self, event):
		painter = QPainter()
		painter.begin(self)
		print('y')

		painter.translate(QPoint(100,10))
		painter.rotate(10.0)  # can be used to flip the table (vertical or horizontal), it flips the whole coordinate system
		painter.scale(0.8, 0.8)  # idem with scale, can be ued to zoom in or out

		pen =QPen(QColor(250, 0, 0), 5.0)
		painter.setPen(pen)
		#painter.setFont(QFont('Open Sans', 12))
		# painter.drawText(event.rect(), Qt.AlignCenter, self.text)


		painter.drawPixmap(QRect(0,0,640//2,480//2), self.pixmap_image, QRect(0,0,640,480))

		painter.drawEllipse(100, 100, 56, 56)

		x = 250
		y = 100
		self.robot_rect = QRect(x, y, 100, 200)
		# painter.drawRect(self.robot_rect)
		painter.fillRect(self.robot_rect, QColor(250, 0, 0))


		painter.setPen(QPen(Qt.black, 5, Qt.SolidLine))
		painter.setBrush(QBrush(Qt.green, Qt.SolidPattern))
		painter.drawRect(self.shape)

		painter.end()






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