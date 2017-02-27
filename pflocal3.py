# pflocal3.py
# add particle filter moving

from math import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import random

landmarks = [[200.0, 450.0], [400.0, 450.0], [600.0, 450.0],
            [200.0, 300.0],                  [600.0, 300.0],
            [200.0, 150.0], [400.0, 150.0], [600.0, 150.0]]

size_x = 800
size_y = 600
robot = 0
pf = []
num_pf = 1000

# convert pos Cartesian to Graphics
def cX(x):
    return x

def cY(y):
    return size_y - y

def deg(r):
    return 360.0 * r /(2.0 * pi)

def rad(d):
    return (2.0 * pi)* d/360.0

class Robot:

    def __init__(self):
        self.x = random.random() * size_x
        self.y = random.random() * size_y
        self.theta = random.random() * 2.0 * pi

        self.forward_noise = 0.0
        self.turn_nise = 0.0
        self.sense_noise = 0.0

    # Set robot's initial position and orientation
    def setPos(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= size_x:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= size_y:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')
 
        self.x = float(new_x)
        self.y = float(new_y)
        self.theta = float(new_orientation)

    # Set the noise parameters, changing them is often useful in particle filters
    def setNoise(self, new_forward_noise, new_turn_noise, new_sense_noise):
        self.forward_noise = float(new_forward_noise)
        self.turn_noise = float(new_turn_noise)
        self.sense_noise = float(new_sense_noise)

    # move some distance with given angle 
    def move(self, dist, deg):
        if dist < 0:
            raise ValueError('Robot cannot move backwards')

        # add noise
        dist += dist * random.gauss(0.0, self.forward_noise)/100.0

        if deg == 0:
            deg = 360.0
        deg += deg * random.gauss(0.0, self.turn_noise)/100.0
        deg %= 360.0 
 
        theta = self.theta + rad(deg)
        x = self.x + (cos(self.theta) * dist)
        y = self.y + (sin(self.theta) * dist)

        x %= size_x
        y %= size_y
        #print("dist: ",dist,"deg: ", deg, "result:", x, y)

        self.theta = theta
        self.x = x
        self.y = y

    def sense(self):
        z = []
 
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            z.append(dist)
 
        return z        


class MapUI(QWidget):
    def __init__(self, parent=None):
        super(MapUI, self).__init__(parent)
        self.setMinimumSize(self.minimumSizeHint())

    def minimumSizeHint(self):
        return QSize(800, 600)    

    def paintEvent(self, event=None):
        painter = QPainter(self)    

        pen = QPen(QColor(20,20,20))
        painter.setPen(pen);
        painter.drawRect(0, 0, size_x-1, size_y-1)

        brush = QBrush(QColor(200,20,20), Qt.SolidPattern);
        painter.setBrush(brush)

        # draw landmarks
        for i in range(len(landmarks)):
            pos_x = cX(landmarks[i][0])
            pos_y = cY(landmarks[i][1])
            painter.drawEllipse(pos_x-8, pos_y-8, 16, 16)

        # draw particle filters
        for i in range(num_pf):
            p = pf[i]
            pos_x = cX(p.x)
            pos_y = cY(p.y)
            pos_theta = p.theta

            pen = QPen(QColor(170,100,20))
            painter.setPen(pen);

            brush = QBrush(QColor(220,150,10), Qt.SolidPattern);
            painter.setBrush(brush)
            painter.drawEllipse(pos_x-8, pos_y-8, 16, 16)

            pen.setWidth(2);
            painter.setPen(pen);

            x = cX(cos(pos_theta) * 16.0) + pos_x
            y = cY(sin(pos_theta) * 16.0 + p.y)

            painter.drawLine(pos_x, pos_y, x ,y)

        # draw robot
        pos_x = cX(robot.x)
        pos_y = cY(robot.y)
        pos_theta = robot.theta
        pen = QPen(QColor(20,20,100))
        painter.setPen(pen);        

        brush = QBrush(QColor(50,80,240), Qt.SolidPattern);        
        painter.setBrush(brush)
        painter.drawEllipse(pos_x-8, pos_y-8, 16, 16)

        pen.setWidth(2);
        painter.setPen(pen);

        x = cX(cos(pos_theta) * 16.0) + pos_x
        y = cY(sin(pos_theta) * 16.0 + robot.y)

        painter.drawLine(pos_x, pos_y, x ,y)

    def moveButtonClicked(self):
        robot.move(10.0, 5.0)
        robot.sense()

        for i in range(num_pf):
            pf[i].move(10.0, 5.0)
        self.update()


if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    form = QDialog()
    map = MapUI()

    button1 = QPushButton("Quit")
    button1.clicked.connect(QCoreApplication.instance().quit)
    button2 = QPushButton("Move")
    button2.clicked.connect(map.moveButtonClicked)
 
    layout1 = QVBoxLayout()
    layout1.addWidget(map)

    layout2 = QHBoxLayout()
    layout2.addWidget(button1)
    layout2.addWidget(button2)

    layout = QVBoxLayout()
    layout.addLayout(layout1)
    layout.addLayout(layout2)
    form.setLayout(layout)
 
    form.setWindowTitle("particle filter Localization")

    robot = Robot()
    robot.setPos(400.0, 300.0, pi/2.0) 
    robot.setNoise(5.0, 10.0, 5.0)

    # make particle filter
    for i in range(num_pf):
        p = Robot()
        p.setNoise(5.0, 10.0, 5.0)
        pf.append(p)

    form.show()
    app.exec_()

