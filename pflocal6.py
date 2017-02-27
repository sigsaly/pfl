# pflocal6.py
# add weightGraph and more

from math import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import random
import time

landmarks = [[200.0, 450.0], [400.0, 450.0], [600.0, 450.0],
            [200.0, 300.0],                  [600.0, 300.0],
            [200.0, 150.0], [400.0, 150.0], [600.0, 150.0]]

size_x = 800
size_y = 600
robot = 0
graph = 0
pf = []
spf = []
weight = []
num_pf = 1000
run_mode = False
move_done = True

# defalut value for noise and movement
forward_noise = 1.0
turn_noise = 0.05
sensor_noise = 5.0
move_distance = 2.0
move_theta = 0.0

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
        self.turn_noise = 0.0
        self.sensor_noise = 0.0

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
    def setNoise(self, new_forward_noise, new_turn_noise, new_sensor_noise):
        self.forward_noise = float(new_forward_noise)
        self.turn_noise = float(new_turn_noise)
        self.sensor_noise = float(new_sensor_noise)

    # move some distance with given angle 
    def move(self, dist, deg):
        if dist < 0:
            raise ValueError('Robot cannot move backwards')

        theta = self.theta + float(rad(deg)) + random.gauss(0.0, self.turn_noise)
        theta %= 2 * pi

        dist = float(dist) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(self.theta) * dist)
        y = self.y + (sin(self.theta) * dist)

        x %= size_x
        y %= size_y

        res = Robot()
        res.setPos(x, y, theta)
        res.setNoise(forward_noise, turn_noise, sensor_noise)

        return res

    def sense(self):
        global sensor_noise
        z = []
 
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, sensor_noise)
            z.append(dist)
 
        return z        

    @staticmethod
    def calcGaussianProbability(mu, sigma, x):
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    def getProbability(self, measurement):
        
        prob = 1.0
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.calcGaussianProbability(dist, self.sensor_noise, measurement[i])
        return prob

class MapUI(QWidget):

    sig_done = pyqtSignal()

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

        # draw resampled particle filters
        if len(spf) != 0:
            for i in range(num_pf):
                p = spf[i]
                pos_x = cX(p.x)
                pos_y = cY(p.y)
                pos_theta = p.theta
    
                pen = QPen(QColor(100,170,20))
                painter.setPen(pen);

                brush = QBrush(QColor(150,220,10), Qt.SolidPattern);
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
        global pf, spf, robot, move_done, weight
        global move_distance, move_theta
        
        if len(spf) != 0:
            pf = spf

        robot = robot.move(move_distance, move_theta)
        z = robot.sense()

        p2 = []

        for i in range(num_pf):
            p2.append( pf[i].move(move_distance, move_theta))

        p = p2                

        w = []
 
        for i in range(num_pf):
            w.append(p[i].getProbability(z))
        
        weight = w

        spf = []
 
        index = int(random.random() * num_pf)
        beta = 0.0
        mw = max(w)
 
        for i in range(num_pf):
            beta += random.random() * 2.0 * mw
 
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % num_pf
            spf.append(p[index])

        self.update()
        graph.update()
        move_done = True

    @staticmethod
    def disableRun():
        global button3, button4

        button3.setEnabled(False)
        button4.setEnabled(True)

    @staticmethod
    def enableRun():
        global button3, button4

        button3.setEnabled(True)
        button4.setEnabled(False)

    def runButtonClicked(self):
        global run_mode
        self.disableRun()
        run_mode = True


    def stopButtonClicked(self):
        global run_mode        
        run_mode = False
        self.enableRun()

class runThread(QThread):
    sig_run = pyqtSignal()
    
    def __init__(self, mapui, parent=None):
        super(runThread,self).__init__(parent)
        self.sig_run.connect(mapui.moveButtonClicked)

    def run(self):
        global run_mode, move_done
        while True:
            if run_mode:
                move_done = False
                self.sig_run.emit() 
                while move_done == False:
                    time.sleep(0.1)
                time.sleep(0.1)
            else:
                time.sleep(0.1)


class WeightGraph(QWidget):

    def __init__(self, parent=None):
        super(WeightGraph, self).__init__(parent)
        self.setMinimumSize(self.minimumSizeHint())

    def minimumSizeHint(self):
        return QSize(1000, 100)    

    def paintEvent(self, event=None):
        global weight

        painter = QPainter(self)    

        pen = QPen(QColor(20,20,20))
        painter.setPen(pen);
        painter.drawRect(0, 0, 1000-1, 100-1)

        if len(weight)==0:
            return

        wmax = max(weight)

        if wmax == 0.0:
            pen = QPen(QColor(235,130,10))
            painter.setPen(pen);
            painter.setFont(QFont('Arial', 15))
            painter.drawText(10,30, "Max: 0.0")            
            return
      
        pos_x = 0
        pos_y = 100

        # draw weight graph
        for i in range(num_pf):
            w = weight[i]
            w = w/wmax*100

            pen = QPen(QColor(60,10,10))
            painter.setPen(pen);
            #if w > 0:
            #    print(pos_x, pos_y, w, 100-w)
            painter.drawLine(pos_x, pos_y, pos_x, 100-w)
            pos_x += 1

        pen = QPen(QColor(235,130,10))
        painter.setPen(pen);
        painter.setFont(QFont('Arial', 15))
        painter.drawText(10,30, "Max: "+str(wmax))



if __name__ == "__main__":
    import sys

    def ForwardNoiseChanged(): # default 1.0, range 0.0 to 10
        global forward_noise
        value = slider1.value()
        # convert using simple 2 line graph
        if value < 51:
            forward_noise = value / 50.0
        else:
            forward_noise = value * 9.0/49.0 - 9.0 * 50.0/49.0 + 1
        label1_1.setText(str(forward_noise))

    def TurnNoiseChanged(): # default 0.05, range 0.0 to 0.099
        global turn_noise
        value = slider2.value()
        turn_noise = value / 1000.0
        label2_1.setText(str(turn_noise))

    def SensorNoiseChanged(): # default 5.0, range 0 to 40
        global sensor_noise
        value = slider3.value()
        # convert using simple 2 line graph
        if value < 51:
            sensor_noise = value / 10.0
        else:
            sensor_noise = value * 35.0/49.0 - 35.0 * 50.0/49.0 + 5
        if sensor_noise == 0.0:
            sensor_noise = 0.001
        label3_1.setText(str(sensor_noise))
        
        
    def MoveDistanceChanged(): # default 2.0, range 0.1 to 10.0
        global move_distance
        value = slider4.value()
        # convert using simple 2 line graph
        if value < 51:
            move_distance = value * 1.9/50.0 + 0.1
        else:
            move_distance = value * 8.0/49.0 - 8.0 * 50.0/49.0 + 2
        label4_1.setText(str(move_distance))

    def MoveThetaChanged(): # default 0.0, range -pi/8 to pi/8 (22.5deg)
        global move_theta
        value = slider5.value()
        # convert using simple 2 line graph
        move_theta = value * (pi/8.0)/50.0 - pi/8.0
        label5_1.setText(str(move_theta))

    app = QApplication(sys.argv)
    form = QDialog()
    map = MapUI()
    graph = WeightGraph()

    label1 = QLabel("Forward")
    label1_1 = QLabel(str(forward_noise))
    label2 = QLabel("Turn")
    label2_1 = QLabel(str(turn_noise))
    label3 = QLabel("Sensor")
    label3_1 = QLabel(str(sensor_noise))

    label4 = QLabel("Distance")
    label4_1 = QLabel(str(move_distance))
    label5 = QLabel("Theta")
    label5_1 = QLabel(str(move_theta))

    slider1 = QSlider(Qt.Horizontal) # Forward
    slider1.setValue(50)
    slider1.valueChanged.connect(ForwardNoiseChanged)
    slider2 = QSlider(Qt.Horizontal) # Turn
    slider2.setValue(50)
    slider2.valueChanged.connect(TurnNoiseChanged)
    slider3 = QSlider(Qt.Horizontal) # Sensor
    slider3.setValue(50)
    slider3.valueChanged.connect(SensorNoiseChanged)

    slider4 = QSlider(Qt.Horizontal) # Distance
    slider4.setValue(50)
    slider4.valueChanged.connect(MoveDistanceChanged)
    slider5 = QSlider(Qt.Horizontal) # Theta
    slider5.setValue(50)
    slider5.valueChanged.connect(MoveThetaChanged)

    group1 = QGroupBox("Setup Noise")
    group2 = QGroupBox("Setup Move")

    ll1 = QHBoxLayout()
    ll1.addWidget(label1)
    ll1.addWidget(label1_1)
    ll2 = QHBoxLayout()
    ll2.addWidget(label2)
    ll2.addWidget(label2_1)
    ll3 = QHBoxLayout()
    ll3.addWidget(label3)
    ll3.addWidget(label3_1)
    ll4 = QHBoxLayout()
    ll4.addWidget(label4)
    ll4.addWidget(label4_1)
    ll5 = QHBoxLayout()
    ll5.addWidget(label5)
    ll5.addWidget(label5_1)

    layout_s1 = QVBoxLayout()
    layout_s1.addLayout(ll1)
    layout_s1.addWidget(slider1)
    layout_s1.addLayout(ll2)
    layout_s1.addWidget(slider2)
    layout_s1.addLayout(ll3)
    layout_s1.addWidget(slider3)

    layout_s2 = QVBoxLayout()
    layout_s2.addLayout(ll4)
    layout_s2.addWidget(slider4)
    layout_s2.addLayout(ll5)
    layout_s2.addWidget(slider5)

    group1.setLayout(layout_s1)
    group2.setLayout(layout_s2)

    button1 = QPushButton("Quit")
    button1.clicked.connect(QCoreApplication.instance().quit)
    button2 = QPushButton("Move")
    button2.clicked.connect(map.moveButtonClicked)
    button3 = QPushButton("Run")
    button3.clicked.connect(map.runButtonClicked)
    button4 = QPushButton("Stop")
    button4.setEnabled(False)
    button4.clicked.connect(map.stopButtonClicked)

    spacer1 = QSpacerItem(20,20,QSizePolicy.Minimum, QSizePolicy.Expanding)
    spacer2 = QSpacerItem(20,20,QSizePolicy.Minimum, QSizePolicy.Expanding)
    spacer3 = QSpacerItem(20,20,QSizePolicy.Minimum, QSizePolicy.Expanding)
    spacer4 = QSpacerItem(20,20,QSizePolicy.Minimum, QSizePolicy.Expanding)

    layout1 = QVBoxLayout()
    layout1.addWidget(group1)
    layout1.addItem(spacer1)
    layout1.addWidget(group2)
    layout1.addItem(spacer2)
    layout1.addWidget(button2)
    layout1.addItem(spacer3)
    layout1.addWidget(button3)
    layout1.addWidget(button4)
    layout1.addItem(spacer4)
    layout1.addWidget(button1)

    layout2 = QHBoxLayout()
    layout2.addWidget(map)
    layout2.addLayout(layout1)

    layout = QVBoxLayout()
    layout.addLayout(layout2)
    layout.addWidget(graph)
    form.setLayout(layout)

    form.setWindowTitle("particle filter Localization")

    robot = Robot()
    robot.setPos(400.0, 300.0, pi/2.0) 
    robot.setNoise(forward_noise, turn_noise, sensor_noise)

    # make particle filter
    for i in range(num_pf):
        p = Robot()
        p.setNoise(forward_noise, turn_noise, sensor_noise)
        pf.append(p)

    r_th = runThread(map)
    r_th.start()    

    form.show()
    app.exec_()

