# pforg0.py
# draw map

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

landmarks = [[200.0, 450.0], [400.0, 450.0], [600.0, 450.0],
            [200.0, 300.0],                  [600.0, 300.0],
            [200.0, 150.0], [400.0, 150.0], [600.0, 150.0]]

size_x = 800
size_y = 600
# convert pos Cartesian to Graphics
def cX(x):
    return x

def cY(y):
    return size_y - y

class MapUI(QWidget):
    def __init__(self, parent=None):
        super(MapUI, self).__init__(parent)
        self.setMinimumSize(self.minimumSizeHint())
        #self.image = QImage()
        #self.image.load('map1.bmp')

    def minimumSizeHint(self):
        return QSize(800, 600)    

    def paintEvent(self, event=None):
        painter = QPainter(self)    
        #painter.drawImage(0,0, self.image)

        pen = QPen(QColor(20,20,20))
        painter.setPen(pen);
        painter.drawRect(0, 0, size_x-1, size_y-1)

        brush = QBrush(QColor(200,20,20), Qt.SolidPattern);
        painter.setBrush(brush)

        for i in range(len(landmarks)):
            pos_x = cX(landmarks[i][0])
            pos_y = cY(landmarks[i][1])
            painter.drawEllipse(pos_x-8, pos_y-8, 16, 16)
       
if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    form = QDialog()

    button = QPushButton("Quit")
    button.clicked.connect(QCoreApplication.instance().quit)
 
    map = MapUI()
    layout = QVBoxLayout()
    layout.addWidget(map)
    layout.addWidget(button, stretch = 0)
    form.setLayout(layout)
 
    form.setWindowTitle("particle filter Localization")
    form.show()
    app.exec_()

